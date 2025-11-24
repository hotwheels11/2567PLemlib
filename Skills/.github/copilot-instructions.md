# Copilot Instructions for 2567PLemlib

## Project Overview
- VEX V5 robotics project using PROS, LemLib, robodash, and LVGL.
- Main code in `src/`, headers in `include/`.
- Custom libraries: `lemlib/` (chassis, odometry, PID), `robodash/` (UI, selector).
- External dependencies: libpros, libc, libm, LemLib, robodash, LVGL.

## Architecture & Key Components
- Entry point: `src/main.cpp` (PROS lifecycle functions).
- Autonomous selector: `rd::Selector` (from robodash) is used for routine selection.
- LemLib provides advanced chassis and odometry logic (`include/lemlib/chassis/`).
- robodash provides UI and selector logic (`include/robodash/views/selector.hpp`).

## Build & Developer Workflow
- **Build:** Use `pros make` or VS Code PROS extension.
- **Clean:** `pros clean` or `make clean`.
- **Upload:** `pros upload`.
- **Debug:** `pros terminal` for logs.
- **Hot/cold linking** is enabled (see Makefile, common.mk).

## Project-Specific Patterns
- **Namespace usage:** Use `lemlib::` and `rd::` for library code.
- **Selector pattern:** Autonomous routines are managed by `rd::Selector`.
- **Header includes:** Angle brackets for system/pros headers, quotes for local headers.
- **No CMake:** All builds via Makefile/PROS CLI.

## Common Issues & Solutions
- **Undefined reference errors:**
  - If you see errors like `undefined reference to rd::Selector::run_auton()` or `rd_view_create`, ensure the robodash library is linked. The actual implementation is in the prebuilt `robodash.a` (not in your src/).
  - Do not expect to find robodash or LemLib source files in `src/`; they are linked as prebuilt libraries.
  - If you add new files, update the Makefile if needed.
- **robodash integration:**
  - All robodash symbols (e.g., `rd::Selector`, `rd_view_create`) are provided by the robodash library, not your own code.

## Key Files & Directories
- `src/main.cpp`: Main entry point
- `include/lemlib/`: Chassis, odometry, PID
- `include/robodash/`: Dashboard, selector, UI
- `static/`: Static assets
- `Makefile`, `project.pros`: Build configuration

## When in Doubt
- If you see linker errors for robodash or LemLib, check that the libraries are present in `firmware/` and referenced in the Makefile/project.pros.
- Follow namespace conventions (`lemlib::`, `rd::`).
- Reference this file for project-specific patterns before asking for help.
