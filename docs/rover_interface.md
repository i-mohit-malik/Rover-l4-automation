# Rover interface (frozen)

The WAVE ROVER runs its unchanged Waveshare firmware. We have **no access** to it: no code,
no flashing, no wiring. The only contact is Wi-Fi: the rover joins the box access point
`CAS-Box-NN` as a station and answers HTTP requests.

```
GET http://<rover-ip>/js?json=<url-encoded JSON>
```

`firmware/box/src/comms/RoverLink.*` wraps this. Application code never builds HTTP itself; it calls
`rover.setDrive(l, r)` once per loop pass.

## Commands the starter program uses

| JSON | Meaning | Reply |
|---|---|---|
| `{"T":1,"L":l,"R":r}` | wheel speed per side, `-0.5 … 0.5` (0.5 = 100 % PWM). Sent every 100 ms while driving. | `200 OK` |
| `{"T":1,"L":0,"R":0}` | stop; repeated until confirmed three times, then every second while idle | `200 OK` |
| `{"T":130}` | base info; the box reads the battery voltage `"v":…` from the reply | JSON |
| `{"T":405}` | Wi-Fi config; used only to *identify* the rover among the AP stations (`wifi_mode_on_boot` in the reply) | JSON |

The Waveshare WAVE ROVER JSON command set contains more types (OLED text, IMU of the rover,
PWM input, …). They are **not** part of this project unless a decision record in
`docs/decisions/` adds one with a measured reason.

## Motion primitives available to us

Everything is differential drive from `L` and `R`:

| Motion | L | R |
|---|---|---|
| forward | +v | +v |
| backward | −v | −v |
| spin left on the spot | −v | +v |
| spin right on the spot | +v | −v |
| arc left | +v·k | +v (k < 1) |
| arc right | +v | +v·k |
| stop | 0 | 0 |

There is **no feedback** from the rover about distance driven or angle turned (no odometry
over this link). Turning "by 90°" therefore has to be closed on the **box IMU yaw**
(`sensors` → `ImuManager::yawDeg()`), and distances on time × speed or LiDAR/ultrasonic.
Low battery (3× 18650) makes the rover unable to turn: check `rover.voltage()` before a run.

## Safety chain (must stay)

1. Phone page: key release → `l = r = 0`; page loses focus → stop.
2. Box `loop()`: no drive update from the phone for `PAD_TIMEOUT_MS` (400 ms) → stop.
3. `RoverLink` task: set point older than 500 ms → sends 0/0 (deadman), even if `loop()` hangs.
4. Rover firmware: no command for 3 s → stops by itself.

Autonomous code replaces step 1 only. Steps 2–4 remain exactly as in the starter program.
