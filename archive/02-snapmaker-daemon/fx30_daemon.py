#!/usr/bin/env python3
"""
fx30_daemon.py - persistent Sony FX30 PTP/IP trigger daemon for a Klipper host

This is a Python port of the protocol logic from fx30_remote_trigger.ino
(the ESP32-C3 "OpenClick" firmware). It keeps ONE long-lived PTP/IP session
open to the camera (exactly like the ESP32 does, with the same keepalive),
and exposes a tiny local Unix socket that a Klipper macro can hit to fire
the shutter instantly - no per-shot handshake, no per-shot Wi-Fi work.

WHY A DAEMON AND NOT A ONE-SHOT SCRIPT
The PTP/IP handshake (OpenSession + three SDIOConnect round trips +
GetExtDeviceInfo calls) takes real time - multiple TCP round trips. Doing
that fresh for every single layer-change trigger would add real,
compounding delay to every layer of the whole print. The ESP32 avoids this
by opening the session once at boot and holding it open with a keepalive;
this daemon does the same thing, running continuously on the Klipper host.

WHAT THIS DOES NOT DO
It does not join a Wi-Fi network for you. This version assumes the camera
and the printer are both client devices on the SAME router (shared LAN),
not the camera's own Wi-Fi Direct hotspot - so there's no radio-sharing
conflict with the printer's normal network connection. Point the camera's
Wi-Fi settings at your router instead of "PC Remote"/Wi-Fi Direct mode,
and make sure the printer's Klipper host is on that same router/network.
If you'd rather use the camera's own Wi-Fi Direct AP after all, see the
default_gateway() helper below and swap it back into resolve_camera_ip().

USAGE
    python3 fx30_daemon.py
Reads configuration from the CONFIG block below, or from environment
variables of the same names (FX30_IFACE, FX30_CAMERA_IP, FX30_SOCK_PATH).
Logs to stdout - run it under systemd (see fx30-trigger.service) so it
starts on boot and restarts itself if it ever crashes.

CONTROL PROTOCOL (what the Klipper-side client sends)
Connect to the Unix socket, write one line, read one line back:
    "FIRE\n"   -> "OK\n"                 (shutter fired)
                  "ERR: <reason>\n"       (camera not ready / write failed)
    "STATUS\n" -> "READY\n" / "LOST\n" / "CONNECTING\n"
"""

import os
import socket
import struct
import subprocess
import threading
import time
import logging
import sys

# --------------------------------------------------------------- CONFIG ---
# Shared-router setup: camera and printer are both plain clients on the same
# network, so there's no Wi-Fi Direct SSID to join. Set FX30_CAMERA_IP to a
# fixed address (give the camera a DHCP reservation on the router so this
# never changes), or leave it blank to let the daemon find the camera by
# scanning the subnet for the open PTP/IP port - convenient, but it does
# mean start-up takes a few seconds longer and could find the wrong device
# if something else on your network happens to have port 15740 open.
IFACE = os.environ.get("FX30_IFACE", "wlan0")           # interface on the shared network
CAMERA_IP = os.environ.get("FX30_CAMERA_IP", "")         # blank = scan the subnet for it
PTPIP_PORT = 15740
SOCK_PATH = os.environ.get("FX30_SOCK_PATH", "/run/fx30-trigger/trigger.sock")

SHUTTER_HOLD_MS = 300     # matches OC_SHUTTER_HOLD_MS in the .ino - manual focus, shutter only
KEEPALIVE_IDLE_S = 8.0    # matches the ESP32's 8s idle keepalive
RECONNECT_RETRY_S = 5.0
CMD_TIMEOUT_S = 4.0

CLIENT_GUID = bytes([0x4f, 0x70, 0x65, 0x6e, 0x43, 0x6c, 0x69, 0x63,
                      0x6b, 0x43, 0x33, 0x00, 0x01, 0x02, 0x03, 0x04])
CLIENT_NAME = "fx30-klipper-daemon"

# PTP/IP packet types
PIP_INIT_CMD_REQ = 1
PIP_INIT_CMD_ACK = 2
PIP_INIT_EVT_REQ = 3
PIP_INIT_EVT_ACK = 4
PIP_OP_REQUEST = 6
PIP_OP_RESPONSE = 7
PIP_START_DATA = 9
PIP_EVENT = 8
PIP_END_DATA = 0x0C

# PTP opcodes (verified against alpha-fairy's ptpsonycodes.h, per the .ino)
OP_GetDeviceInfo = 0x1001
OP_OpenSession = 0x1002
OP_GetStorageIDs = 0x1004
OP_SDIOConnect = 0x9201
OP_SDIOGetExtDeviceInfo = 0x9202
OP_SetControlDeviceB = 0x9207

PROP_Capture = 0xD2C2   # S2 / full press

EVT_ObjectAdded = 0xC201
EVT_PropertyChanged = 0xC203

BTN_DOWN = 0x0002
BTN_UP = 0x0001

RESP_OK = 0x2001

logging.basicConfig(level=logging.INFO,
                     format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("fx30")


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
    """One persistent PTP/IP session to the camera, mirroring the ESP32
    firmware's state machine (WIFI -> HANDSHAKE -> READY -> LOST)."""

    def __init__(self, camera_ip):
        self.camera_ip = camera_ip
        self.cmd_sock = None
        self.evt_sock = None
        self.txn = 1
        self.conn_num = 0
        self.ready = False
        self.last_activity = 0.0
        self.lock = threading.RLock()   # serializes all cmd-socket traffic

    # ---- low level framing --------------------------------------------
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

    # ---- handshake -------------------------------------------------------
    def connect(self):
        self.close()
        log.info("connecting to camera at %s:%d", self.camera_ip, PTPIP_PORT)
        self.cmd_sock = socket.create_connection((self.camera_ip, PTPIP_PORT), timeout=5)
        self.evt_sock = socket.create_connection((self.camera_ip, PTPIP_PORT), timeout=5)
        self.txn = 1

        # Init Command Request
        payload = CLIENT_GUID + utf16_name(CLIENT_NAME) + put32(0x00010000)
        self._send_packet(self.cmd_sock, PIP_INIT_CMD_REQ, payload)
        pkt = self._read_packet(self.cmd_sock)
        if get32(pkt, 4) != PIP_INIT_CMD_ACK:
            raise PTPIPError("no Init Command Ack")
        self.conn_num = get32(pkt, 8)
        log.info("Init Command Ack, conn %d", self.conn_num)

        # Init Event Request
        self._send_packet(self.evt_sock, PIP_INIT_EVT_REQ, put32(self.conn_num))
        pkt = self._read_packet(self.evt_sock)
        if get32(pkt, 4) != PIP_INIT_EVT_ACK:
            raise PTPIPError("no Init Event Ack")
        log.info("Init Event Ack")

        # Sony connect sequence (order taken from alpha-fairy's init_table,
        # same as the .ino)
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

    # ---- operations --------------------------------------------------
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
            # anything else (data-phase packets) - keep reading
        return None

    def button(self, prop, value):
        data = struct.pack("<H", value)
        return self._op(OP_SetControlDeviceB, [prop], data=data,
                         required=True, label="prop 0x%04X=%d" % (prop, value))

    def capture(self):
        """Single shutter press, no AF, no confirmation - matches the
        ESP32's default tuning (OC_PHOTO_USE_AF=0, OC_PHOTO_VERIFY=0)."""
        self.button(PROP_Capture, BTN_DOWN)
        time.sleep(SHUTTER_HOLD_MS / 1000.0)
        self.button(PROP_Capture, BTN_UP)
        time.sleep(0.06)

    def keepalive(self):
        self._op(OP_SDIOGetExtDeviceInfo, [0x12C, 0, 0], required=True,
                  label="keepalive")

    def pump_events(self):
        """Drain the event socket so it never backs up. Non-blocking."""
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


def default_gateway(iface):
    """Only relevant if you go back to the camera's own Wi-Fi Direct AP
    (mirrors WiFi.gatewayIP() on the ESP32 - the camera is the gateway of
    its own hotspot). Not used in the shared-router setup."""
    out = subprocess.run(["ip", "route", "show", "dev", iface],
                          capture_output=True, text=True, check=False).stdout
    for line in out.splitlines():
        if line.startswith("default"):
            parts = line.split()
            return parts[2]
    raise PTPIPError("could not find default gateway on %s" % iface)


def local_subnet_hosts(iface):
    """Every host address in the IPv4 /24 (or narrower) the given interface
    is on, as dotted strings, camera's likely address included."""
    out = subprocess.run(["ip", "-o", "-4", "addr", "show", "dev", iface],
                          capture_output=True, text=True, check=False).stdout
    line = out.strip().splitlines()[0] if out.strip() else ""
    if not line:
        raise PTPIPError("interface %s has no IPv4 address - is it up and "
                          "connected to the router yet?" % iface)
    cidr = line.split()[3]                     # e.g. "192.168.1.42/24"
    ip_str, prefix_str = cidr.split("/")
    prefix = int(prefix_str)
    if prefix < 22:                             # refuse to scan anything huge
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
    """Find the camera by probing every host on the local subnet for the
    open PTP/IP command port. Runs the probes in parallel so a /24 scan
    takes a fraction of a second rather than 254 x timeout."""
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

    def status(self):
        with self.state_lock:
            sess = self.session
        if sess is None:
            return "CONNECTING"
        return "READY" if sess.ready else "LOST"

    # ---- control socket -----------------------------------------------
    def serve_control(self):
        sock_dir = os.path.dirname(SOCK_PATH)
        os.makedirs(sock_dir, exist_ok=True)
        if os.path.exists(SOCK_PATH):
            os.remove(SOCK_PATH)
        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        srv.bind(SOCK_PATH)
        os.chmod(SOCK_PATH, 0o666)
        srv.listen(4)
        log.info("control socket listening at %s", SOCK_PATH)
        while self.running:
            conn, _ = srv.accept()
            threading.Thread(target=self._handle_client, args=(conn,),
                              daemon=True).start()

    def _handle_client(self, conn):
        with conn:
            try:
                data = conn.recv(64).strip().upper()
                if data == b"FIRE":
                    t0 = time.monotonic()
                    self.fire()
                    conn.sendall(b"OK\n")
                    log.info("fired (%.0f ms)", (time.monotonic() - t0) * 1000)
                elif data == b"STATUS":
                    conn.sendall((self.status() + "\n").encode())
                else:
                    conn.sendall(b"ERR: unknown command\n")
            except PTPIPError as e:
                conn.sendall(("ERR: %s\n" % e).encode())
            except OSError as e:
                conn.sendall(("ERR: socket error: %s\n" % e).encode())

    def run(self):
        t = threading.Thread(target=self.connector_loop, daemon=True)
        t.start()
        try:
            self.serve_control()
        except KeyboardInterrupt:
            self.running = False


if __name__ == "__main__":
    log.info("fx30 trigger daemon starting - iface=%s camera_ip=%s sock=%s",
              IFACE, CAMERA_IP or "(auto-scan)", SOCK_PATH)
    Daemon().run()
