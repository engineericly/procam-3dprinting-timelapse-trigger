// Copy this file to secrets.h and fill in your own values.
//
//     cp secrets.example.h secrets.h
//
// secrets.h is gitignored and must never be committed. The sketch compiles
// without it, using the placeholder defaults in the .ino, but it will not
// join your network until these are real.

#pragma once

#define ROUTER_SSID  "your-wifi-ssid"
#define ROUTER_PASS  "your-wifi-password"

// The printer running Klipper + Moonraker.
#define MOONRAKER_HOST "192.168.1.50"
#define MOONRAKER_PORT 7125

// The camera, once it has joined the same network.
#define CAM_IP_STR "192.168.1.60"
