# ADR-0001: Repository layout and what belongs in it

Date: 2026-10-19 · Status: accepted

## Context

The CAS deliverable includes a source-code zip. The team has a fixed sensor box (ESP32 Dev
Module), a separate ESP32-CAM flashed through a different port, a frozen rover, and a project
folder (`L4_Rover/`) that already holds datasheets, guidelines, slides and test results.

## Decision

1. **One git repository** at `03_ENGINEERING/Rover-l4-automation/` holds only what is flashed
   (`firmware/`) or executed on the laptop (`tools/`), plus the documentation needed to use it
   (`docs/`). Drive logs, the test log, the submission zip, datasheets and slides live in the
   project folder (`04_SAFETY_AND_TESTING/Test_Results/`, `06_REPORTS_AND_DEMO/Final_Submission/`,
   …), listed in `docs/project_folder.md`.
2. **Box and CAM are two sketches in the same repo**, `firmware/box/` and `firmware/cam/`, not
   two repositories. Reasons: one submission zip, one changelog, one tag per drive that
   describes both devices; the Arduino IDE treats each sketch folder independently anyway, so
   different ports are no problem. A second repo would only add a sync problem.
3. **Datasheets, guidelines, the pristine starter program and presentations stay outside** the
   repo in their existing project folders. `docs/project_folder.md` points to them. PDF, PPTX,
   PNG and ZIP files are git-ignored so nobody commits a copy by accident.
4. **The starter program is the first commit**, tagged `starter-v1.0`. Every change is a diff
   against it, which keeps "what did we change versus what the course gave us" answerable.
5. **Settings live in `Config.h`** (kit number, tunables). Pins stay in the modules that use
   them, because the PCB is fixed and moving numbers around adds a second place to get wrong.
   The pin table in `docs/hardware/pinout.md` is the human-readable index.

## Consequences

- The submission zip is produced by `tools/make_submission.ps1` from the repo tree, so what is
  submitted is exactly what is versioned. It is written to `06_REPORTS_AND_DEMO/Final_Submission/`.
- The tools assume the repo sits at `L4_Rover/03_ENGINEERING/Rover-l4-automation/`;
  `fetch_log.py --out` overrides the log folder for a different checkout location.
- If the CAM ever needs a different toolchain (ESP-IDF, PlatformIO), it gets its own subfolder
  under `firmware/cam/` rather than a new repository.
