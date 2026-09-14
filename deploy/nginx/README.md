# Hosting the console with plain nginx

For a normal Linux box on the LAN - the Ubuntu VM on the TrueNAS server is the
obvious home. No Docker, no reverse proxy, nothing installed on the TrueNAS
host itself, so nothing here is lost when TrueNAS updates or a config is
restored.

## Setup

```bash
# 1. the files
sudo git clone https://github.com/engineericly/procam-3dprinting-timelapse-trigger.git \
  /srv/procam-console

# 2. nginx
sudo apt update && sudo apt install -y nginx

# 3. the site
sudo cp /srv/procam-console/deploy/nginx/procam-console.conf \
  /etc/nginx/sites-available/procam-console.conf
sudo ln -s /etc/nginx/sites-available/procam-console.conf \
  /etc/nginx/sites-enabled/procam-console.conf

# 4. check the config before reloading, then reload
sudo nginx -t && sudo systemctl reload nginx
```

nginx is already enabled at boot by the Debian/Ubuntu package, so it comes back
after a VM restart with no extra step.

Port `8088` is a guess. The VM already runs other services, so check it is free
first with `sudo ss -lntp | grep 8088` and edit both `listen` lines if not.

## Tell Moonraker to accept the page

On the printer, in `moonraker.conf`:

```
[authorization]
cors_domains:
    http://<vm-ip>:8088
```

Restart Moonraker. Without this the browser blocks the response even though the
request reaches the printer. The origin must match exactly - scheme, host and
port.

## Use it

Bookmark `http://<vm-ip>:8088`. Works from any device on the LAN, phone
included. In the Printer link panel enter the printer's own address
(`192.168.x.x:7125`) and hit **Check link**.

## Updating

```bash
cd /srv/procam-console && sudo git pull
```

No reload needed - nginx serves the file from disk, and the `no-cache` header
means the browser picks up the change on the next load.

## Keep it on the LAN

No certificate, and do not put this behind Nginx Proxy Manager or Cloudflare.
Two reasons: HTTPS would break the printer calls, and the page can jog the
toolhead and trigger the shutter while Moonraker's API executes arbitrary
G-code. If you ever want it from outside the house, reach it over Tailscale
rather than publishing it.
