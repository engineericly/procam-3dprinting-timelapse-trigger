# Project notes

The memory for this project: where it stands, what to do next, why it is built
this way, and what has bitten before. Keep it current when state changes.

Last updated: 2026-09-28

## Goal

Trigger a camera's shutter at every layer of a 3D print over WiFi only - no SSH
into the printer and no wiring between printer, camera and trigger. Done means an
unattended print finishes with a frame at every layer, the head parked clear of
the shot each time, and no print aborted by an out-of-range move.

Out of scope: assembling frames into video, camera exposure automation, the
physical camera mount.

## Current state

**Proven on real hardware:** the ESP32 joins WiFi, completes the Sony PTP/IP
handshake, subscribes to Moonraker and fires the shutter on the trigger. A full
print on the K2 Pro completed with a frame per layer (parked at X280 Y280 at the
time). `[layer_num]` substitution was checked against a real 266,663-line
sliced file.

**Built but not yet proven on hardware:**

- Park point moved to the back-left corner, **X0 Y300**, in both the time lapse
  box and a new machine end G-code. It is inside the K2 Pro's measured travel
  limits and the head was driven there idle - but no print has run with it.
- Machine end G-code that ends the print in the pose of the last frame and takes
  one final frame. Untested on a real print.
- Firmware web page (`http://procam.local/`): photo, record, camera address,
  printer list, errors explained, and OTA updates. **Compiled cleanly, not yet
  flashed.** The board still runs the previous build and was stuck at LINKING,
  most likely because the camera was still holding the session from before the
  reflash.

## Next action

1. Flash the current firmware **over USB**, with partition scheme **Minimal
   SPIFFS (1.9MB APP with OTA)** and `secrets.h` next to the `.ino`. After that,
   updates can go over WiFi.
2. Power-cycle the camera so it drops any old PTP session, then check the web
   page or the OLED. They now say exactly what is wrong if it still will not link.
3. Run one watched test print on the K2 Pro, By layer, with all three slicer
   blocks. Confirm: the head parks at X0 Y300 every layer; the final frame fires;
   **nothing moves after the final frame** (if it does, it is the CFS steps
   `BOX_END` / `BOX_END_PRINT` inside `END_PRINT`); the bed does not sag once
   `M84` turns the motors off; and `max_z_position` reads 300.0 again afterwards.

## Printers

| Printer | Moonraker | Bed | Travel limits | Status |
|---|---|---|---|---|
| Creality K2 Pro | 192.168.100.199:7125 | 300 x 300 x 300 | X -8.9..302, Y -6.5..302 | Verified: limits and `END_PRINT` read from the machine |
| Creality SparkX i7 | 192.168.100.30:7125 (also 4408) | 260 x 260 x 255 | X -16..279, Y -7..280 | Verified the same way. Its `END_PRINT` is a different kind - see below |
| Creality K2 Plus | 192.168.100.254:7125 | 350 x 350 x 350 | X -7.7..352.5, Y -6.2..352 | Verified: limits read from the machine; its end macros are identical to the K2 Pro's |

All three answer Moonraker on 7125 (checked 2026-09-28 with all three on).

The K2 Plus's `END_PRINT` differs from the K2 Pro's in one useful way: it ends
without `M84`, so the motors stay powered after a print and the bed cannot sag.
Its `END_PRINT_Z_SAFE` and `END_PRINT_POINT` are line-for-line identical to the
K2 Pro's, so the same end-G-code override applies.

Camera: Sony FX30 at 192.168.100.240, PTP/IP port 15740, joined to the same
router in PC Remote mode.

The addresses are private LAN IPs. Give every device a DHCP reservation so they
never move.

## Build and flash

The Arduino IDE bundles a command-line compiler, which is how this is built
outside the IDE:

```bash
CLI="/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"
"$CLI" compile --fqbn esp32:esp32:esp32c3:PartitionScheme=min_spiffs,CDCOnBoot=cdc <sketch-folder>
```

- ESP32 core 3.3.12 (Espressif "esp32" package, board **ESP32C3 Dev Module** -
  not the separate "Arduino ESP32 Boards" package), libraries **WebSockets**
  (Links2004) and **U8g2**. Also set Tools > USB CDC On Boot > Enabled, or the
  Serial Monitor stays blank on this board. After a core update the IDE's board
  picker can go empty until the IDE is fully quit and reopened.
- The sketch is `firmware/firmware.ino` because the Arduino IDE requires the
  `.ino` to match its folder name. Do not rename either one: on a mismatch the IDE
  offers to move the `.ino` into a new folder, leaves `secrets.h` behind, and the
  build then succeeds with placeholder WiFi, so the board never joins the network.
- The last build was 1,323,413 bytes: 67% of the 1.9 MB OTA app slot. The
  default partition layout (1.25 MB) is too small, and "Huge APP" has no OTA slot.
- Apple Silicon needs Rosetta 2 for the IDE's bundled `ctags`, or the build dies
  with "bad CPU type in executable":
  `softwareupdate --install-rosetta --agree-to-license`
- OTA: the IDE shows a network port "procam", or upload a `.bin` at
  `http://procam.local/update` (user `admin`). The password is `OTA_PASSWORD` in
  `secrets.h`; empty turns OTA off. Do not update mid-print.

## Pitfalls - read before editing

**Slicer blocks**

- **Square brackets are placeholders.** Creality Print template-parses the whole
  custom G-code box, comments included. Any `[...]` that is not a real slicer
  variable fails the slice with "Variable does not exist". `[layer_num]` on the
  M117 line must be the only pair; the end and start blocks must have none. The
  console enforces this.
- **Klipper only pushes a value when it changes.** Identical `M117` text twice
  sends nothing the second time, so the ESP32 never hears it. That is why the box
  appends `[layer_num]`, why the start G-code begins with a bare `M117` (so a
  manual test message cannot swallow a print's first frame), and why test
  messages should always contain `TEST` plus something unique.
- **M117, not RESPOND.** RESPOND needs a `[respond]` section the K2 Pro's stock
  config lacks ("Unknown command:RESPOND"). M117 writes `display_status`, which is
  always active. The SparkX i7 is the same.
- **Ooze goes wherever the head goes next.** The nozzle drips during the dwell.
  Creality Print runs the prime tower before returning to the model, so the ooze
  is wiped there - verified against real sliced G-code. A slicer that returns
  straight to the object puts the blob on the print; that is what forced a purge
  pad on the Snapmaker U1. Check this on any new printer or slicer, and keep the
  prime tower enabled.
- **Print By layer.** The time lapse box only lifts 2 mm, which does not clear a
  taller part already finished in *By object* mode.

**Printers**

- **Test every park or timing change by hand first.** An out-of-range move
  mid-print aborts the whole job, not one frame. Klipper checks moves against the
  axis limits, not the bed - and inside the limits is not the same as physically
  clear: off the bed the nozzle can meet the carrier, a wiper or the chamber wall.
- **`END_PRINT` macros differ, dangerously.** On the K2 Pro, zeroing
  `PRINTER_PARAM.max_z_position` around `END_PRINT` stops it lifting and dropping
  the bed. On the SparkX i7 the same variable is a *ceiling*: zeroing it makes the
  macro drive Z to 1 and push the nozzle into the print. That printer's stock
  macro always lifts 50 mm and parks at X267 Y199, so it cannot hold the pose.
  Before using the override on any printer, read its `END_PRINT_POINT` body
  (`/printer/objects/query?configfile=config`). The variable merely existing proves
  nothing.
- **Not every printer can do this.** It needs Moonraker. Anycubic stock firmware
  hides it (rooting needed); Bambu never exposes it.
- **Macro names: no letters directly followed by digits.** Klipper's parser read
  `FX30_TRIGGER_SHOT` as the G-code `FX30` - the reason for the PROCAM rename.

**Firmware**

- **The camera takes one PTP client.** A board that resets or is reflashed
  mid-session leaves the camera holding the old one, and it refuses the next
  (shown as `CAM BUSY`). Power-cycle the camera or toggle its network function.
- **The ESP32-C3 has one core.** The web page, the camera and the Moonraker link
  share one loop, so nothing may block for long. An offline printer used to block
  it for about 5 s every 4 s, starving the page and camera retries - hence the
  400 ms probe and 15 s back-off. `OC_USE_WEBUI 0` removes the server entirely.
- **Moonraker needs a one-shot token on every websocket connect**, and a real
  `Origin` header - the library's default `file://` origin gets a 403.
- **There is no VIDEO/PHOTO mode on the ESP32.** Stills vs movie is set on the
  camera body; in movie mode the shutter command is accepted and ignored.

**Console**

- **Serve it over plain HTTP.** An HTTPS page cannot call a plain-HTTP printer
  (mixed content), so the live controls only work from the LAN.
- **Open it by IP address, not by name.** Creality's Moonraker (K2 Pro, SparkX
  i7) returns CORS headers only for pages whose origin is an IP address:
  `http://127.0.0.1:8080` and `http://192.168.100.114:8080` work, while
  `http://localhost:8080`, `http://<mac>.local:8080` and `file://` are refused -
  the request reaches the printer and the browser discards the reply ("Failed
  to fetch"). No `cors_domains` change is needed; the console warns and links to
  the IP version when opened by name. The page also
  declares UTF-8 itself; without that, nginx and `http.server` mangled every
  non-ASCII character, including inside copied G-code comments.

## Open items

- The top-of-file header in the firmware still describes the rejected design: it
  calls the API "the U1's own Fluidd/Mainsail web UI" and says a `[respond]` line
  is needed. Comment-only, but misleading.
- A temporary diagnostic `Serial.printf("   WS TEXT ...")` in `webSocketEvent()`
  logs every websocket frame. Harmless noise; remove once triggering is confirmed
  on the new build.
- The K2 Plus and SparkX i7 park points (X0 Y350, X0 Y260) are inside their
  measured travel limits but have not been jogged to by hand, unlike the K2 Pro's.
- The camera's stills/movie mode cannot be switched remotely; no verified PTP
  property for it on the FX30.

## Decisions

Newest last. Dates are 2026.

- **09-08 - BLE dropped for PTP/IP over WiFi.** BLE probing never reached a
  workable trigger. Sony's PTP/IP handshake (Init Command/Event Ack,
  OpenSession, SDIOConnect x3, GetExtDeviceInfo) and button properties
  (SetControlDeviceB on AutoFocus/Capture/Movie) proved reliable.
- **09-09 - First integration used a microswitch** on a Snapmaker U1, to prove the
  camera side before adding printer-side complexity.
- **09-09/10 - Switch replaced by a Moonraker websocket trigger.** Removed the last
  wiring to the printer, with no SSH. This is still the architecture.
- **09-10 - Macro renamed FX30 to PROCAM** after the letters-then-digits parser bug.
- **09-10/11 - Moved from the Snapmaker U1 to a Creality K2 Pro**, and all naming
  genericised to PROCAM. The U1 oozed during the dwell and then returned straight
  to the object, landing the ooze on the model - the reason it needed a purge pad.
- **09-11 - M117 plus a `display_status` subscription instead of RESPOND**,
  settled by testing on the K2 Pro.
- **09-11 - Creality Print's built-in smooth timelapse ruled out.** It gives no
  control over park position and trigger timing for an external camera.
- **09-11 - Custom purge pad dropped on the K2 Pro.** The prime tower fires in the
  right order, verified against sliced G-code.
- **09-13 - Console built**: printer profiles, a scale bed picker, and generated
  slicer blocks, with the bracket rule enforced at runtime.
- **09-14 - Console hosted on the LAN over plain HTTP.** Netlify and other HTTPS
  hosts cannot reach the printers (mixed content); tunnels were considered and
  rejected as too much for a button used twice a year.
- **09-14 - Code moved out of the Obsidian vault into this repo.** WiFi
  credentials found hardcoded in the firmware were moved to a gitignored
  `secrets.h`, and history was rewritten before the first public push, so they
  were never exposed.
- **09-22 - Park moved to the back-left corner, X0 Y300**, in both the time lapse
  box and a new end G-code, so a print ends in the pose of its last frame for a
  clean cut to video. Creality's `END_PRINT` bed drop is neutralised on the K2 Pro.
- **09-25 - SparkX i7 added**, specs read from the printer. Its `END_PRINT` works
  differently, so end styles are now per printer and the override is never
  generated for it.
- **09-25 - ESP32 got its own web page.** Choosing the printer and the camera
  controls moved to the board, so switching printers no longer needs a reflash;
  the console's firmware-constants panel was removed as redundant.
- **09-28 - Offline printers no longer freeze the ESP32, and OTA added.** Failures
  are explained on the page and the OLED; the camera address is editable there.
- **09-28 - Repo trimmed to code only; project notes moved here** from the vault.
  The superseded code (BLE probes, Snapmaker daemon, firmware, Klipper macros,
  every G-code iteration) and the nginx/Docker/Netlify hosting configs were
  removed from the tree. They are still in git history - see below.

## Recovering removed files

Everything removed on 2026-09-28 is intact in commit `9ceae28`:

```bash
git show 9ceae28 --stat -- archive deploy netlify.toml     # list it
git checkout 9ceae28 -- archive                             # bring a folder back
```

`archive/` holds the BLE and PTP/IP probe sketches, the Snapmaker U1 daemon,
firmware and Klipper macros, and every G-code box iteration. Read the decisions
above before building anything from it - each was dropped for a reason.
