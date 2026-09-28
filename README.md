# PROCAM Timelapse Trigger

Fire a camera's shutter at every print layer, entirely over WiFi. An ESP32 joins
the printer's network and the camera's, watches Moonraker's websocket for a
per-layer message, and trips the shutter over Sony PTP/IP. No SSH on the
printer, no wiring to it.

"PROCAM" is deliberately generic — the camera body may be swapped later, so it
is not named after one.

## Layout

| Path | What |
|---|---|
| `firmware/` | ESP32-C3 firmware, with its own web page at `http://procam.local/` |
| `console/` | Printer profiles, park-point picker, and generators for the slicer blocks |
| `slicer/` | The time lapse G-code box and machine end G-code, as pasted into the slicer |
| `NOTES.md` | Project memory: current state, next step, decisions, and the pitfalls |

**Read `NOTES.md` before changing anything.** Most of this project's rules were
learned the hard way on real hardware, and several are not obvious from the code.

## Firmware

```bash
cd firmware && cp secrets.example.h secrets.h   # then fill it in
```

`secrets.h` holds the WiFi, camera address, default printer and OTA password,
and is gitignored. Build for ESP32-C3 with the partition scheme **Minimal SPIFFS
(1.9MB APP with OTA)**. After the first USB flash, updates can go over WiFi.
Build and flash details are in `NOTES.md`.

## Console

One static HTML file, no build step. Serve `console/` over **plain HTTP** on the
printers' network:

```bash
python3 -m http.server 8080 --directory console
```

Open it by **IP address** - `http://127.0.0.1:8080`, not `localhost`. Creality's
Moonraker only answers pages opened by IP; by name (`localhost`, `*.local`) or as
a file, the live controls fail with "Failed to fetch". HTTPS works for generating
G-code, but browsers block an HTTPS page from calling a plain-HTTP printer.
