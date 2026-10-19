# Rover-l4-automation

Firmware and tooling for the **sensor box ("brain")** of the WAVE ROVER in the
Case Study *Cooperative and Autonomous Systems* (CAS), WS 2026/27.

```
phone ──Wi-Fi──► box ESP32 (this repo: sensors, logging, decisions) ──Wi-Fi──► rover (motors, FROZEN)
                 ESP32-CAM (this repo, separate port) ──► box / phone
```

Only code that gets **flashed to the box / CAM** or **run on the laptop** lives here.
Datasheets, the original starter program, slides and reports stay in the project
folder outside this repository (see [docs/project_folder.md](docs/project_folder.md)).
Drive logs go to `04_SAFETY_AND_TESTING/Test_Results/`, the submission zip to
`06_REPORTS_AND_DEMO/Final_Submission/`.

## Layout

| Folder | What it holds | Flashed / run where |
|---|---|---|
| [`firmware/box/`](firmware/box/) | Arduino sketch for the **box ESP32** (ESP32 Dev Module, CH340 USB). Sensors, web HMI, rover link, logging, autonomy. | Box ESP32, COM port of the box |
| [`firmware/cam/`](firmware/cam/) | Arduino sketch for the **ESP32-CAM** (AI-Thinker). Own serial port, own upload. | ESP32-CAM, second port |
| [`tools/`](tools/) | Laptop-side Python/PowerShell: fetch, parse and plot the timeline logs; build the submission zip. | Laptop |
| [`docs/`](docs/) | Architecture, pinout, rover interface, logging design, decision records, where things live outside the repo. | – |

## Quick start

1. Install Arduino IDE 2.x, the `esp32` board package **3.3.x** and the libraries listed
   in [firmware/README.md](firmware/README.md).
2. Open `firmware/box/box.ino`, set `BOX_ID` in [`firmware/box/src/core/Config.h`](firmware/box/src/core/Config.h)
   to the kit number, choose board *ESP32 Dev Module*, upload.
3. Switch on the box first, then the rover. Phone joins `CAS-Box-NN`, opens `http://192.168.(10+N).1`.
4. Drive with the pad. The box writes a timeline log; afterwards fetch and plot it with the
   scripts in `tools/` (see [docs/logging.md](docs/logging.md)).

## Rules that bind this repo

- **The rover firmware is frozen.** Nothing here is ever flashed to the rover. The only
  interface is the JSON command set in [docs/rover_interface.md](docs/rover_interface.md).
- **Hardware is fixed.** Pin assignments in [docs/hardware/pinout.md](docs/hardware/pinout.md)
  are facts, not choices.
- **Stop logic stays.** Key release, page timeout, RoverLink deadman and rover heartbeat
  must never be removed (starter program rule, see the header of `firmware/box/box.ino`).
- **Naming scheme stays.** Wi-Fi name, password, IP and channel derive from `BOX_ID` only.

## Deadlines (CAS guidelines WS 2026/27)

| What | When |
|---|---|
| Paper (PDF, IEEE two-column, max 8 pages), video (MP4), **source code (zip)** on iLearn | 18 Jan 2027, 18:00 CET |
| Competitive technical demonstration | 21 Jan 2027, 14:00–17:30 CET |

Source zip name: `2627_CAS_source_<matr1>_<matr2>_<matr3>.zip`, built by `tools/make_submission.ps1`.

## Workflow

- `main` always compiles and drives safely. Work on short-lived branches
  (`feature/<topic>`, `fix/<topic>`), merge after a test on the rover.
- Tag every version that was driven on the real rover: `drive-YYYY-MM-DD`.
- Every test drive gets a log file and a row in the test log in
  `04_SAFETY_AND_TESTING/Test_Results/` naming the firmware tag that produced it.
- [CHANGELOG.md](CHANGELOG.md) records what changed per version.
