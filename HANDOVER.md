# Handover

What the next person needs that the code does not say: where everything
runs, how to get in, and what is tied to a person's accounts. Surveyed on
the robot on 2026-10-23. Secrets are not written here, only where they are.

## The four computers

| Computer | Address on the robot | Tailscale name | Login | What runs there |
|---|---|---|---|---|
| Perception (Jetson Xavier NX, Ubuntu 20.04, ROS2 Foxy) | 192.168.1.103 | `lite3-perception` | `ysc` | this repository in `~/robot`; services `hmi`, `lio_relay`, `realsense_ros2`, `transfer_ros2`, `roam-rebind` |
| Motion (RK3588, vendor) | 192.168.1.120; 192.168.2.1 on the robot's wifi | `lite3-motion` | `ysc` | the vendor controller `jy_exe`, `track` (person detector), `rtsp_stream` (front camera), `robot-nat`, the wifi access point (`~/wifi_mode.sh`) |
| Orin Nano (Ubuntu, ROS 2 Humble) | 192.168.1.5 | Tailscale IP 100.114.74.5 (hostname `ubuntu`) | `lite3` | the lidar and mapping: `livox-driver`, `lidar-web` (panel on :8000), `lidar-scan`, `lio-send`, `cloud-locate`, `eduroam-watchdog`, `orin-nat` |
| Livox MID-360 lidar | 192.168.1.102 | | | |

The robot's wifi is `YSC-JYML-pr3gdi-5G`. A laptop on it gets a
192.168.2.x address and reaches the motion computer at 192.168.2.1.

## Getting in

```bash
ssh ysc@lite3-perception                               # over Tailscale
ssh -J ysc@192.168.2.1 ysc@192.168.1.103               # from the robot's wifi, no Tailscale
# from the perception computer, keys are already set up:
ssh ysc@192.168.1.120                                  # motion computer
ssh lite3@192.168.1.5                                  # Orin
```

Tailscale SSH asks for a browser sign-in to the tailnet the first time a
new laptop connects.

The panel is `http://lite3-perception:8080` (Tailscale) or
`http://192.168.2.1:8080` (the robot's wifi). It runs as `hmi.service`, so it
is up after every boot. Its log is `/tmp/hmi.log`. Stop it
(`sudo systemctl stop hmi`) before you run another program that holds the
robot.

## Tied to a person: change these before the owner leaves

- [ ] **Tailscale.** All the robot's nodes are on a personal tailnet. When
      that account goes, remote access goes. Move the nodes to a lab or
      department tailnet (`sudo tailscale up --force-reauth` on each computer),
      then update this file.
- [ ] **Eduroam.** The Orin is the robot's only way to the internet, and it
      signs in to eduroam with a personal `@uia.no` account (NetworkManager
      connection `eduroam` on the Orin). Without internet, Gemini and the
      Tailscale relay stop working. Replace it with a lab or service account.
- [ ] **Gemini API key.** It is in `~/.gemini_key` on the perception computer
      and belongs to Håvard Karlsen Solheim's personal Google account. Replace
      it with a lab-owned key: write the new key into that file
      (`chmod 600`), then `sudo systemctl restart hmi`.
- **Who to ask:** Håvard Karlsen Solheim, haavardkso@gmail.com, who built
  this repository.

## Code that is not in this repository

- **The Orin's `~/bin`** (52 files) holds the lidar web panel
  (`lidar_web.py`, `map_web.py`), the map slicer (`map_walls.py`), the
  FAST-LIO2 and KISS-ICP scripts and their configs. It is **not under git**.
  Its own notes are `~/bin/README.md` there; the hostname that README
  gives (`jetson-lite3`) is out of date. Three files
  are copies of this repository's: `lio_relay.py`, `cloud_locate.py` and
  `cloudreg.py`. Copy them over again after you change them here.
- The Orin also has `~/robot_pc/`, an older Nav2 setup from before this
  repository took over navigation.
- The vendor code lives on the robot, outside this repository. Our changes
  to it are `env/*.patch` (docs/environment.md).

## Leftovers you can delete once you have checked them

- On the perception computer: `~/robot.broken-20261007` and
  `~/robot_git_corrupt_20261010` (old broken copies of the repository), and
  `~/archive`.
- Many `*.pre-*` and `*.orig` backups in the Orin's `~/bin`.
- Two systemd units on the perception computer fail at every boot:
  `nv-l4t-usb-device-mode` and `trojan_client` (nothing in this repository
  uses either).

## What does not work yet

__________ (fill in: open bugs, half-finished features, what you would do next)
