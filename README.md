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
| `firmware/` | ESP32-C3 firmware, the current working build |
| `slicer/` | The Creality Print time lapse G-code box, as pasted into the slicer |
| `console/` | Web dashboard: printer profiles, park-point picker, G-code generator |
| `archive/` | Every superseded approach, in five dated eras. Read `archive/README.md` before reusing anything from it. |

## Project continuity lives in the AI-OS vault

This repo holds the code. Status, next actions and the decision log stay in the
Obsidian vault, because the agent tooling there depends on them:

`~/Documents/Obsidian/AI-OS/Projects/procam-timelapse-trigger/`

- `NOW.md` — current state and the exact next action
- `decisions.md` — why each earlier approach was dropped
- `state.md` — what is done, in progress, and unverified

Check `NOW.md` before changing anything here.

## Credentials

Wi-Fi and host settings are not in the repo. Before flashing:

```bash
cd firmware && cp secrets.example.h secrets.h   # then edit it
```

`secrets.h` is gitignored. The sketch compiles without it using placeholder
defaults, so a missing secrets.h shows up as a failed Wi-Fi join, not a build
error.

## Running the console

It is one static HTML file with no build step and no backend. Open it directly,
or serve the `console/` directory:

```bash
python3 -m http.server 8080 --directory console
```

**Serve it over plain HTTP on the printer's network** if you want the live
Moonraker controls (jog test, test fire, homed-state readout). A page served
over HTTPS cannot reach a `http://192.168.x.x` printer — browsers block it as
mixed content — so on Netlify or any HTTPS host the generator still works but
the live controls stay disabled and say why. Serving locally also needs
`cors_domains` in `moonraker.conf` to allow the page's origin.

### On the TrueNAS

`deploy/truenas/` has a Docker Compose stack that serves `console/` over plain
HTTP on the LAN, which is what makes the live Moonraker controls work. See
`deploy/truenas/README.md`. LAN only, deliberately - the page can move the
toolhead.

A published copy lives at:
https://claude.ai/code/artifact/b2bf91ae-2da3-4bde-854e-c1375d9bcb47

To update that copy rather than create a second one, publish with its URL
explicitly — the file path moved when this repo was created, and publishing a
new path silently makes a new artifact.

## Things that will bite you

- **Square brackets.** Creality Print template-parses the whole G-code box,
  comments included, and reads any `[...]` as a slicer variable. `[layer_num]`
  on the M117 line must be the only one in the file, or the slice fails with
  "Variable does not exist". The console enforces this at runtime.
- **M117, not RESPOND.** RESPOND needs a `respond` section the K2 Pro's stock
  `printer.cfg` does not have — confirmed by "Unknown command:RESPOND" in
  Fluidd. M117 writes to `display_status`, which is already active.
- **Jog before you print.** Send `G1 X<park> Y<park> F6000` with the printer
  idle and the nozzle clear of the bed first. An out-of-range move mid-print
  aborts the whole job, not just one photo.
- **Not every printer can do this.** The method needs Moonraker. Anycubic's
  stock firmware does not expose it (needs rooting); Bambu never does.
- **Token must match.** `TRIGGER_TOKEN` in the firmware has to equal the token
  in the M117 line, or nothing fires.
