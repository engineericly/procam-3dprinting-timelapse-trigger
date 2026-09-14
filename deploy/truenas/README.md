# Hosting the console on TrueNAS

Serves `console/` as a static site on the LAN, so the page and the printer are
both on plain HTTP and the live Moonraker controls work.

## 1. Get the files onto the NAS

Either clone the repo to a dataset:

```bash
git clone https://github.com/engineericly/procam-3dprinting-timelapse-trigger.git \
  /mnt/tank/apps/procam-console
```

or copy the `console/` folder over SMB. Updating later is `git pull`, or
re-copying the one file.

## 2. Create the stack

In Dockge, new stack named `procam-console`, paste `docker-compose.yml`, and
set `CONSOLE_DIR` to the absolute path of the folder containing `index.html`.
Deploy.

Check the port first — `8088` is a guess. Change the left-hand side of
`"8088:80"` if something else on the NAS already uses it.

## 3. Tell Moonraker to accept the page

On the printer, in `moonraker.conf`:

```
[authorization]
cors_domains:
    http://<nas-ip>:8088
```

Restart Moonraker. Without this the browser blocks the response even though the
request reaches the printer.

## 4. Bookmark it

`http://<nas-ip>:8088` — works from any device on the LAN, phone included.

In the console's Printer link panel, enter the printer's own address
(`192.168.x.x:7125`) and hit **Check link**.

## Deliberately not exposed

This stack is LAN-only. It is not behind Nginx Proxy Manager and not published
through Cloudflare. The page can jog the toolhead and start a test fire, and
Moonraker's API executes arbitrary G-code, so it has no business being reachable
from the internet. If you ever want it away from home, put it behind Tailscale
rather than opening it up.
