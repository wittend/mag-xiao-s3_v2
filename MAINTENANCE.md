# Maintenance — mag-xiao-s3 (PlatformIO / Seeed XIAO ESP32S3)

This document describes how to keep PlatformIO Core, platforms, frameworks, and libraries up to date while maintaining a balanced approach between freshness and stability.

- Project board: Seeed XIAO ESP32S3
- Build system: PlatformIO (CLI and/or VS Code PlatformIO IDE)
- Language/Framework: Arduino on ESP32-S3

## Philosophy: Balanced Updates

We favor safe, routine updates that pull in bug fixes and minor improvements without risking large breaking changes.

- Platform policy: keep `platform = espressif32` unpinned to track the latest stable platform release.
- Library policy: keep caret version pins (e.g., `name@^x.y.z`) so minor/patch updates are allowed, but major breaking changes are held back.
- Testing: run host-native unit tests and a firmware build after updates.

If a breaking change slips in, temporarily pin the platform to the last known good version until it’s resolved.

## Quick Checklist (run from project root)

```bash
# 1) Update PlatformIO Core (stable)
pio upgrade            # ensures Core is latest stable
pio --version          # verify Core version

# 2) Update platforms, toolchains, frameworks, libs
pio update             # updates packages for all environments
pio system prune -f    # optional: clean unused caches/packages

# 3) Verify and build
pio run -e seeed_xiao_esp32s3  # build firmware for the board
pio test -e native             # run host-native unit tests
```

## PlatformIO Core: Installation-specific notes

Use the command that matches how you installed PlatformIO:

- pipx (recommended):
  ```bash
  pipx upgrade platformio
  pio upgrade
  pio --version
  ```
- pip:
  ```bash
  python3 -m pip install -U platformio
  pio upgrade
  pio --version
  ```
- Homebrew (macOS):
  ```bash
  brew update && brew upgrade platformio
  pio upgrade
  pio --version
  ```
- VS Code IDE: update the “PlatformIO IDE” extension from Marketplace, then verify in the built-in terminal with `pio --version`.

Notes:
- `pio upgrade` installs the latest stable Core (avoid `--dev` unless explicitly testing pre-releases).
- `pio system info` prints useful environment diagnostics.

## Project Dependency Updates

- Update everything (platforms, frameworks, toolchains, libs):
  ```bash
  pio update
  ```
- Show details for the ESP32 platform:
  ```bash
  pio platform show espressif32
  ```
- Update only platforms:
  ```bash
  pio platform update
  ```
- Update globally-installed libraries (if you use any):
  ```bash
  pio lib -g update
  ```

## Verify Resolved Versions

- Dump the resolved environment configuration:
  ```bash
  pio run -e seeed_xiao_esp32s3 -t envdump
  ```
- List installed packages and libraries:
  ```bash
  pio pkg list --installed
  pio lib list
  ```

## Current Versioning Policy in platformio.ini (balanced)

- `platform = espressif32` (no explicit version): track latest stable platform.
- Libraries use caret pins to allow minor/patch updates automatically. Example (excerpt):

```ini
[env:seeed_xiao_esp32s3]
platform = espressif32
board = seeed_xiao_esp32s3
framework = arduino
lib_deps =
    wire
    bblanchon/ArduinoJson@^7.4.2
    jgromes/RadioLib@^7.4.0
    ESP32Async/ESPAsyncWebServer@^3.8.1
    ESP32Async/AsyncTCP@^3.4.9
    mikalhart/TinyGPSPlus@^1.0.3
    lorol/LittleFS_esp32@^1.0.6
    fbiego/esp32time@^2.0.6
```

If you encounter instability, you can temporarily pin the platform, for example:

```ini
platform = espressif32@~6.6.0  ; use the last known good minor series
```

## Troubleshooting & Tips

- Upload/USB (XIAO ESP32S3): If upload port isn’t detected after updates, double-tap reset to enter bootloader and retry. Ensure your user is in the `dialout` group on Linux.
- CDC on boot: If you rely on native USB CDC early in boot, keep these flags in `platformio.ini` (already present):
  ```ini
  build_flags =
    -DARDUINO_USB_MODE=1
    -DARDUINO_USB_CDC_ON_BOOT=1
  ```
- Dependency resolution issues: clear caches and retry updates/builds:
  ```bash
  pio system prune -f
  pio update
  pio run -e seeed_xiao_esp32s3
  ```
- Breakages after updates: 
  1) Pin the platform (`platform = espressif32@<version>`), 
  2) Open an issue with release notes/links, 
  3) Plan migration and unpin once fixed.

## Reference Commands Cheatsheet

```bash
# Build for board
pio run -e seeed_xiao_esp32s3

# Upload to board
pio run -e seeed_xiao_esp32s3 -t upload

# Serial monitor (115200)
pio device monitor -b 115200

# Run host-native unit tests
pio test -e native
```
