# Privacy

The robot carries cameras through places where people are, so it processes
personal data under the GDPR, which the Norwegian Personal Data Act
(personopplysningsloven) makes law in Norway. This file records what it
collects, where the data goes and how long it is kept, and how to run the
robot with as little of it as possible. It is written for the operator and
for the university's data protection officer. It is not legal advice:
the blanks below are for the controller and the data protection officer to
fill in.

## Who is responsible

| | |
|---|---|
| Controller (behandlingsansvarlig) | __________ (e.g. University of Agder, department of __________) |
| Contact for the robot | __________ |
| Data protection officer (personvernombud) | __________ |
| Legal basis (GDPR art. 6) | __________ (to be decided with the data protection officer) |
| Notified to Sikt | __________ (yes / no / not needed, and the reference) |
| DPIA done | __________ (date, or why not needed) |

## What the robot collects

| Data | Personal? | Where it goes | Kept |
|---|---|---|---|
| Live video, front camera and RealSense colour | Yes: faces of people in view | Only to logged-in panel pages, while a page shows that camera. Not stored by the panel. | Not kept |
| Front camera frames during a measured run (`bin/run.py --frames`) | Yes | Saved on the Orin during the run, then copied to the laptop's `logs/` | **1 hour** after the run stops, then deleted by `bin/run.py purge` (cron, every 5 minutes, on the laptop) |
| Text an operator types for him to say | Only if someone types personal data | Spoken by Piper on the perception computer; shown in the panel's chat | Panel: the last 80 lines, in memory, gone when the panel restarts |
| Person detection boxes | Low: a box's position, no image | Robot only (the tracker runs on the motion computer's chip) | Not kept |
| Depth cloud, lidar scans, floor maps | Low: shapes, no faces | Robot, Orin, laptop | Maps until deleted; run bags as long as the run folder |
| Panel log, `/tmp/hmi.log`, `/tmp/follow_nav.log` | No (commands and positions) | Perception computer | Until the next reboot (`/tmp`) |

There is no microphone input: the robot only speaks. Nothing is sent to an
AI service or any other third party: the Google Gemini chat, which could send
camera frames to the USA, was removed on 2026-10-23.

## How it is protected

- **Panel login.** The panel asks for a password before it shows video or
  takes a command. The password is in `~/.lite3_panel_password` on the
  perception computer (made at the panel's first start, readable by its user
  only). Give it only to operators. To change it, delete the file and restart
  the panel (`sudo systemctl restart hmi`). That logs every browser out.
- **Camera frames in runs** are off unless `bin/run.py start` gets `--frames`,
  and are deleted an hour after the run stops.
- **Network.** The panel is reachable only on the tailnet and on the robot's
  own wifi (password-protected). Traffic over Tailscale is end-to-end encrypted;
  the panel itself is plain HTTP, so use the Tailscale address or
  `tailscale serve` (HTTPS) rather than an open network.

## Running the robot in public

1. Put the sign on the robot (`docs/privacy-sign.html`, printed, with the
   blanks filled in).
2. Do not record runs with `--frames` where people who have not agreed pass by.
3. Do not show the camera views to people who do not need them.

## People's rights

Anyone filmed can ask the contact above what was recorded about them, and
have it deleted. In practice: live video is not kept. Run frames are in
`logs/<run>/orin/front/` on the operator's laptop, and can be deleted by hand.
Anyone can also complain to Datatilsynet (datatilsynet.no).

## Open tasks

- [ ] Fill in the table under "Who is responsible" with the data protection officer.
- [ ] Ask whether the project needs a Sikt notification and a DPIA. Camera
      monitoring in shared spaces with new technology usually needs a DPIA.
- [ ] On the robot: delete `~/.gemini_key` and `robot/persona_private.py`
      (neither is used any more), and revoke the key in Google AI Studio.
