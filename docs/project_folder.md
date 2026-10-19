# Project folder outside the repo

The repository holds only code and the docs needed to use it. Everything else lives in the
project folder `L4_Rover/` (two levels above this repo). Do not copy PDFs, slides, logs or zips
into the repo.

| Output | Location (relative to `L4_Rover/`) | Written by |
|---|---|---|
| Drive logs (CSV), plots, test log | `04_SAFETY_AND_TESTING/Test_Results/` | `tools/fetch_log.py`, `tools/plot_run.py`, by hand |
| Source-code zip for iLearn | `06_REPORTS_AND_DEMO/Final_Submission/` | `tools/make_submission.ps1` |

Source material:

| Material | Location (relative to `L4_Rover/`) | Used for |
|---|---|---|
| CAS guidelines WS 2026/27 | `01_PROJECT_MANAGEMENT/Guidelines/2627_CAS Guidelines.pdf` | deadlines, file naming, grading |
| Starter program v1.0 (pristine) | `03_ENGINEERING/Test_Programs/cas_box/` | baseline of `firmware/box/`, also git tag `starter-v1.0` |
| Sensor test sketch | `03_ENGINEERING/Test_Programs/Test_Code/Test_Code.ino` | pin cross-check, simple diagnostics page |
| ESP32 Dev board datasheet | `03_ENGINEERING/Datasheets/ESP32 Dev Datasheet.pdf` | GPIO capabilities |
| ESP32-CAM datasheet | `03_ENGINEERING/Datasheets/ESP32 Cam Datasheet.pdf` | camera pins, flashing |
| LiDAR (LD20) datasheet | `03_ENGINEERING/Datasheets/LiDAR Datasheet.pdf` | protocol, range |
| IMU (BNO055) datasheet | `03_ENGINEERING/Datasheets/IMU Datasheet.pdf` | fusion modes, calibration |
| GPS (NEO-6M) datasheet | `03_ENGINEERING/Datasheets/GPS Datasheet.pdf` | NMEA, update rate |
| Ultrasonic (HC-SR04) datasheet | `03_ENGINEERING/Datasheets/Ultrasonic Sensors Datasheet.pdf` | timing, range |
| Previous-year presentations | `06_REPORTS_AND_DEMO/Previous_Year_Presentations/` | what the demo looked like |

External:
- Waveshare WAVE ROVER wiki (JSON command reference): search "Waveshare WAVE ROVER JSON command"
  on the Waveshare wiki; the subset we rely on is in `docs/rover_interface.md`.
