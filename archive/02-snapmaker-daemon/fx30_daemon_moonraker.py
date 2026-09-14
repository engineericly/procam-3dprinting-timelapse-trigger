#!/usr/bin/env python3
"""
fx30_daemon_moonraker.py - Sony FX30 PTP/IP trigger daemon, triggered over
Moonraker instead of a Klipper shell macro.

WHY THIS VERSION EXISTS
fx30_daemon.py (the sibling script) is triggered by a Klipper macro calling
out through gcode_shell_command to a local Unix socket - which means it has
to run ON the printer's own Klipper host, with SSH/shell access to install
it and a systemd unit, plus the gcode_shell_command extra present.

This version needs none of that. It runs on ANY machine on your LAN - your
PC, a Raspberry Pi, whatever's convenient and stays on during the print -
and watches the printer over Moonraker's own network API (the same API
Fluidd/Mainsail use for their web UI), which the Snapmaker U1 already runs
and exposes on the network without any SSH. The only thing that has to
exist on the printer is a single "[respond]" line in printer.cfg, which you
can add through Fluidd/Mainsail's own web-based config editor - still no
SSH, no filesystem access, no shell.

HOW IT WORKS
The Timelapse G-code box (see timelapse_gcode_moonraker.txt) sends a stock
RESPOND command with a distinctive message ("FX30_TRIGGER") once per layer.
Moonraker mirrors every gcode console line to any connected websocket
client as a "notify_gcode_response" notification. This daemon holds one
websocket connection open to Moonraker, watches for that message, and when
it sees it, fires the camera over the SAME persistent PTP/IP session logic
as fx30_daemon.py (ported from the ESP32 firmware) - already open and
warmed up, so there's no per-shot handshake delay.

DEPENDENCIES
    pip install websocket-client

USAGE
    export FX30_MOONRAKER_HOST=192.168.1.50   # the printer's IP address
    python3 fx30_daemon_moonraker.py

CONFIG (env vars)
    FX30_MOONRAKER_HOST   REQUIRED - printer's IP/hostname on your LAN
    FX30_MOONRAKER_PORT   default 80 (Moonraker is proxied through nginx
                           on the U1's normal web port; if that doesn't
                           connect, try 7125 - Moonraker's own default port,
                           in case your build exposes it unproxied too)
    FX30_TRIGGER_TOKEN    default "FX30_TRIGGER" - must match the RESPOND
                           message in the Timelapse G-code box
    FX30_IFACE            default "wlan0" - the network interface on THIS
                           machine (wherever you run this script) that's on
                           the same LAN as the camera
    FX30_CAMERA_IP        blank = scan the subnet for the camera's open
                           PTP/IP port. Set this (with a DHCP reservation
                           on your router) for faster, more reliable startup.
"""

import json
import os
import socket
import struct
import sys
import threading
import time
import logging

try:
    import websocket   # pip install websocket-client
except ImportError:
    print("Missing dependency - run: pip install websocket-client",
          file=sys.stderr)
    raise

# --------------------------------------------------------------- CONFIG ---
MOONRAKER_HOST = os.environ.get("FX30_MOONRAKER_HOST", "")
MOONRAKER_PORT = int(os.environ.get("FX30_MOONRAKER_PORT", "80"))
TRIGGER_TOKEN = os.environ.get("FX30_TRIGGER_TOKEN", "FX30_TRIGGER")

IFACE = os.environ.get("FX30_IFACE", "wlan0")
CAMERA_IP = os.environ.get("FX30_CAMERA_IP", "")
PTPIP_PORT = 15740

SHUTTER_HOLD_MS = 300
KEEPALIVE_IDLE_S = 8.0
RECONNECT_RETRY_S = 5.0
CMD_TIMEOUT_S = 4.0

CLIENT_GUID = bytes([0x4f, 0x70, 0x65, 0x6e, 0x43, 0x6c, 0x69, 0x63,
                      0x6b, 0x43, 0x33, 0x00, 0x01, 0x02, 0x03, 0x04])
CLIENT_NAME = "fx30-moonraker-daemon"

PIP_INIT_CMD_REQ = 1
PIP_INIT_CMD_ACK = 2
PIP_INIT_EVT_REQ = 3
PIP_INIT_EVT_ACK = 4
PIP_OP_REQUEST = 6
PIP_OP_RESPONSE = 7
PIP_START_DATA = 9
PIP_EVENT = 8
PIP_END_DATA = 0x0C

OP_GetDeviceInfo = 0x1001
OP_OpenSession = 0x1002
OP_GetStorageIDs = 0x1004
OP_SDIOConnect = 0x9201
OP_SDIOGetExtDeviceInfo = 0x9202
OP_SetControlDeviceB = 0x9207

PROP_Capture = 0xD2C2

EVT_ObjectAdded = 0xC201

BTN_DOWN = 0x0002
BTN_UP = 0x0001

RESP_OK = 0x2001

logging.basicConfig(level=logging.INFO,
                     format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("fx30-moonraker")


def put32(v):
    return struct.pack("<I", v & 0xFFFFFFFF)


def get32(buf, off=0):
    return struct.unpack_from("<I", buf, off)[0]


def get16(buf, off=0):
    return struct.unpack_from("<H", buf, off)[0]


def utf16_name(s):
    return s.encode("utf-16-le") + b"\x00\x00"


class PTPIPError(Exception):
    pass


class FX30Session:
    """One persistent PTP/IP session to the camera - identical protocol
    logic to fx30_daemon.py, duplicated here so this file can run standalone
    on a different machine than the printer."""

    def __init__(self, camera_ip):
        self.camera_ip = camera_ip
        self.cmd_sock = None
        self.evt_sock = None
        self.txn = 1
        self.conn_num = 0
        self.ready = False
        self.last_activity = 0.0
        self.lock = threading.RLock()

    def _read_exact(self, sock, n, deadline):
        buf = b""
        while len(buf) < n:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise PTPIPError("timeout reading %d bytes" % n)
            sock.settimeout(max(remaining, 0.01))
            chunk = sock.recv(n - len(buf))
            if not chunk:
                raise PTPIPError("connection closed mid-read")
            buf += chunk
        return buf

    def _read_packet(self, sock, timeout=CMD_TIMEOUT_S):
        deadline = time.monotonic() + timeout
        hdr = self._read_exact(sock, 4, deadline)
        length = get32(hdr)
        if length < 8:
            raise PTPIPError("bad packet length %d" % length)
        rest = self._read_exact(sock, length - 4, deadline)
        return hdr + rest

    def _send_packet(self, sock, ptype, payload):
        pkt = put32(8 + len(payload)) + put32(ptype) + payload
        sock.sendall(pkt)

    def connect(self):
        self.close()
        log.info("connecting to camera at %s:%d", self.camera_ip, PTPIP_PORT)
        self.cmd_sock = socket.create_connection((self.camera_ip, PTPIP_PORT), timeout=5)
        self.evt_sock = socket.create_connection((self.camera_ip, PTPIP_PORT), timeout=5)
        self.txn = 1

        payload = CLIENT_GUID + utf16_name(CLIENT_NAME) + put32(0x00010000)
        self._send_packet(self.cmd_sock, PIP_INIT_CMD_REQ, payload)
        pkt = self._read_packet(self.cmd_sock)
        if get32(pkt, 4) != PIP_INIT_CMD_ACK:
            raise PTPIPError("no Init Command Ack")
        self.conn_num = get32(pkt, 8)
        log.info("Init Command Ack, conn %d", self.conn_num)

        self._send_packet(self.evt_sock, PIP_INIT_EVT_REQ, put32(self.conn_num))
        pkt = self._read_packet(self.evt_sock)
        if get32(pkt, 4) != PIP_INIT_EVT_ACK:
            raise PTPIPError("no Init Event Ack")
        log.info("Init Event Ack")

        self._op(OP_OpenSession, [1], required=True, label="OpenSession")
        self._op(OP_GetDeviceInfo, [], required=False, label="GetDeviceInfo")
        self._op(OP_GetStorageIDs, [], required=False, label="GetStorageIDs (opt)")
        self._op(OP_SDIOConnect, [1, 0, 0], required=True, label="SDIOConnect(1)")
        self._op(OP_SDIOConnect, [2, 0, 0], required=True, label="SDIOConnect(2)")
        self._op(OP_SDIOGetExtDeviceInfo, [0x12C, 0, 0], required=False, label="GetExtDeviceInfo")
        self._op(OP_SDIOConnect, [3, 0, 0], required=True, label="SDIOConnect(3)")
        self._op(OP_SDIOGetExtDeviceInfo, [0x12C, 0, 0], required=False, label="GetExtDeviceInfo")

        self.ready = True
        self.last_activity = time.monotonic()
        log.info("*** CAMERA READY ***")

    def close(self):
        self.ready = False
        for s in (self.cmd_sock, self.evt_sock):
            if s:
                try:
                    s.close()
                except OSError:
                    pass
        self.cmd_sock = None
        self.evt_sock = None

    def _op(self, opcode, params, data=None, required=True, label=""):
        with self.lock:
            txn = self.txn
            self.txn += 1
            body = put32(2 if data is not None else 1)
            body += struct.pack("<H", opcode)
            body += put32(txn)
            for p in params:
                body += put32(p)
            self._send_packet(self.cmd_sock, PIP_OP_REQUEST, body)

            if data is not None:
                start = put32(txn) + put32(len(data)) + put32(0)
                self._send_packet(self.cmd_sock, PIP_START_DATA, start)
                end_payload = put32(txn) + data
                self._send_packet(self.cmd_sock, PIP_END_DATA, end_payload)

            code = self._wait_response()
            self.last_activity = time.monotonic()
            ok = (code == RESP_OK)
            log.debug("%-22s op=0x%04X -> 0x%04X %s", label, opcode, code,
                      "OK" if ok else "FAIL")
            if code is None and required:
                raise PTPIPError("no response to %s" % label)
            if required and not ok:
                raise PTPIPError("%s failed with 0x%04X" % (label, code))
            return ok

    def _wait_response(self):
        deadline = time.monotonic() + CMD_TIMEOUT_S
        while time.monotonic() < deadline:
            pkt = self._read_packet(self.cmd_sock,
                                     timeout=max(deadline - time.monotonic(), 0.05))
            ptype = get32(pkt, 4)
            if ptype == PIP_OP_RESPONSE:
                return get16(pkt, 8)
        return None

    def button(self, prop, value):
        data = struct.pack("<H", value)
        return self._op(OP_SetControlDeviceB, [prop], data=data,
                         required=True, label="prop 0x%04X=%d" % (prop, value))

    def capture(self):
        self.button(PROP_Capture, BTN_DOWN)
        time.sleep(SHUTTER_HOLD_MS / 1000.0)
        self.button(PROP_Capture, BTN_UP)
        time.sleep(0.06)

    def keepalive(self):
        self._op(OP_SDIOGetExtDeviceInfo, [0x12C, 0, 0], required=True,
                  label="keepalive")

    def pump_events(self):
        try:
            self.evt_sock.settimeout(0.01)
            while True:
                hdr = self.evt_sock.recv(4)
                if len(hdr) < 4:
                    return
                length = get32(hdr)
                if length < 8:
                    return
                rest = b""
                self.evt_sock.settimeout(0.3)
                while len(rest) < length - 4:
                    chunk = self.evt_sock.recv(length - 4 - len(rest))
                    if not chunk:
                        return
                    rest += chunk
                pkt = hdr + rest
                if get32(pkt, 4) == PIP_EVENT and len(pkt) >= 10:
                    evt = get16(pkt, 8)
                    if evt == EVT_ObjectAdded:
                        log.debug("event: ObjectAdded (capture confirmed)")
        except socket.timeout:
            return
        except OSError:
            return


def local_subnet_hosts(iface):
    out = __import__("subprocess").run(
        ["ip", "-o", "-4", "addr", "show", "dev", iface],
        capture_output=True, text=True, check=False).stdout
    line = out.strip().splitlines()[0] if out.strip() else ""
    if not line:
        raise PTPIPError("interface %s has no IPv4 address - is it up and "
                          "connected to the router yet?" % iface)
    cidr = line.split()[3]
    ip_str, prefix_str = cidr.split("/")
    prefix = int(prefix_str)
    if prefix < 22:
        prefix = 24
    ip_int = struct.unpack("!I", socket.inet_aton(ip_str))[0]
    mask = (0xFFFFFFFF << (32 - prefix)) & 0xFFFFFFFF
    network = ip_int & mask
    host_bits = 32 - prefix
    count = min((1 << host_bits) - 2, 254)
    base = network + 1
    me = ip_int
    return [socket.inet_ntoa(struct.pack("!I", base + i))
            for i in range(count) if (base + i) != me]


def scan_for_camera(iface, port=PTPIP_PORT, timeout=0.35):
    hosts = local_subnet_hosts(iface)
    found = []
    lock = threading.Lock()

    def probe(ip):
        try:
            with socket.create_connection((ip, port), timeout=timeout):
                with lock:
                    found.append(ip)
        except OSError:
            pass

    threads = [threading.Thread(target=probe, args=(h,)) for h in hosts]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    if not found:
        raise PTPIPError("no device answering on port %d found on %s's "
                          "subnet - make sure the camera is powered on and "
                          "joined to the same router" % (port, iface))
    if len(found) > 1:
        log.warning("more than one device answers on port %d (%s) - "
                     "using %s; set FX30_CAMERA_IP explicitly if this picks "
                     "the wrong one", port, ", ".join(found), found[0])
    return found[0]


class Daemon:
    def __init__(self):
        self.session = None
        self.state_lock = threading.Lock()
        self.running = True

    def resolve_camera_ip(self):
        if CAMERA_IP:
            return CAMERA_IP
        ip = scan_for_camera(IFACE)
        log.info("found camera at %s", ip)
        return ip

    def connector_loop(self):
        while self.running:
            with self.state_lock:
                ready = self.session is not None and self.session.ready
            if not ready:
                try:
                    ip = self.resolve_camera_ip()
                    sess = FX30Session(ip)
                    sess.connect()
                    with self.state_lock:
                        self.session = sess
                except (PTPIPError, OSError, socket.timeout) as e:
                    log.warning("connect failed: %s (retrying in %.0fs)",
                                e, RECONNECT_RETRY_S)
                    if self.session:
                        self.session.close()
                    time.sleep(RECONNECT_RETRY_S)
                    continue
            else:
                sess = self.session
                try:
                    sess.pump_events()
                    if time.monotonic() - sess.last_activity > KEEPALIVE_IDLE_S:
                        sess.keepalive()
                except (PTPIPError, OSError, socket.timeout) as e:
                    log.warning("connection lost: %s", e)
                    sess.close()
            time.sleep(0.5)

    def fire(self):
        with self.state_lock:
            sess = self.session
        if sess is None or not sess.ready:
            raise PTPIPError("camera not ready")
        sess.capture()

    def fire_safe(self):
        t0 = time.monotonic()
        try:
            self.fire()
            log.info("shutter fired (%.0f ms)", (time.monotonic() - t0) * 1000)
        except PTPIPError as e:
            log.warning("fire failed: %s", e)


class MoonrakerBridge:
    """Holds a websocket connection to Moonraker and watches the gcode
    console stream for our trigger token."""

    def __init__(self, daemon):
        self.daemon = daemon
        self._req_id = 1

    def _url(self):
        return "ws://%s:%d/websocket" % (MOONRAKER_HOST, MOONRAKER_PORT)

    def _on_open(self, ws):
        log.info("connected to Moonraker at %s", self._url())
        ident = {
            "jsonrpc": "2.0",
            "method": "server.connection.identify",
            "params": {
                "client_name": "fx30-trigger",
                "version": "1.0.0",
                "type": "agent",
                "url": "https://github.com/local/fx30-trigger",
            },
            "id": self._req_id,
        }
        self._req_id += 1
        ws.send(json.dumps(ident))

    def _on_message(self, ws, message):
        try:
            msg = json.loads(message)
        except ValueError:
            return
        if msg.get("method") != "notify_gcode_response":
            return
        params = msg.get("params") or []
        if not params:
            return
        line = str(params[0])
        if TRIGGER_TOKEN in line:
            log.info("trigger seen: %r", line)
            threading.Thread(target=self.daemon.fire_safe, daemon=True).start()

    def _on_error(self, ws, error):
        log.warning("moonraker websocket error: %s", error)

    def _on_close(self, ws, code, reason):
        log.warning("moonraker connection closed (%s %s)", code, reason)

    def run_forever(self):
        if not MOONRAKER_HOST:
            log.error("FX30_MOONRAKER_HOST is not set - point it at your "
                       "printer's IP address, e.g. 192.168.1.50")
            sys.exit(1)
        while True:
            app = websocket.WebSocketApp(
                self._url(),
                on_open=self._on_open,
                on_message=self._on_message,
                on_error=self._on_error,
                on_close=self._on_close,
            )
            app.run_forever(ping_interval=20, ping_timeout=10)
            log.warning("moonraker connection dropped - retrying in %.0fs",
                        RECONNECT_RETRY_S)
            time.sleep(RECONNECT_RETRY_S)


if __name__ == "__main__":
    log.info("fx30 moonraker-trigger daemon starting - moonraker=%s:%d "
              "token=%r iface=%s camera_ip=%s",
              MOONRAKER_HOST or "(unset!)", MOONRAKER_PORT, TRIGGER_TOKEN,
              IFACE, CAMERA_IP or "(auto-scan)")
    d = Daemon()
    t = threading.Thread(target=d.connector_loop, daemon=True)
    t.start()
    MoonrakerBridge(d).run_forever()
