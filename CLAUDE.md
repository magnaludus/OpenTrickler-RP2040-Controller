# OpenTrickler RP2040 Controller — session-stats / Learn Powder fork

Fork of [eamars/OpenTrickler-RP2040-Controller](https://github.com/eamars/OpenTrickler-RP2040-Controller),
firmware for a powder-trickler/scale build on a **Raspberry Pi Pico 2 W**.

## Repo layout

- `main` — tracks upstream `eamars/OpenTrickler-RP2040-Controller` unmodified.
- `session-stats-bracket-learn` — our feature branch, based on upstream main at
  commit `e23cc45` (PR #112). This is the active development branch.

## What's on the feature branch (session-v1.14 baseline)

Ported from an earlier chat-based dev loop (patch-and-flash against a full
repo clone, no local git). Current firmware version tag: `session-v1.14`.
Flashed over WiFi via Settings > Firmware (single `.uf2`).

- Session stats with CSV export (`src/session_stats.c/h`)
- Normal and Match brackets in 0.02 gr steps (`src/charge_mode.cpp/h`)
- Amber session backlight
- Learn Powder auto-tune (`src/learn_mode.cpp/h`): cup capacity, time goal,
  target success rate; per-throw adaptive tuning; lag measurement and
  compensation; settle-before-judging plus top-up on a short throw
- Mobile web portal layout updates (`src/html/web_portal.html`): gate
  controls hidden unless the servo gate is enabled, Learn on the bottom bar
  and in the menu, over/under popups that self-clear
- WiFi firmware update (`src/ota.c/h`)

New files: `learn_mode.cpp/h`, `session_stats.c/h`, `ota.c/h`,
`session_version.h`. Modified: `charge_mode.cpp/h` (largest diff),
`web_portal.html`, plus small hooks in `app.c/h`, `eeprom.h`, `http_rest.c`,
`rest_endpoints.c`, `menu.c`, `mui_menu.c`, `neopixel_led.h`, `lwipopts.h`,
`CMakeLists.txt`.

## This machine's calibration numbers (from v1.11 Learn CSV)

- Coarse tube: **10.1 gr/s per rps**
- Fine tube: **0.33 gr/s per rps**
- Scale lag: **0.55 s** — consistent across both tubes and all speeds tested

These are the values the Learn-mode fit should reproduce. If a fresh fit's
lag column isn't close to 0.55 s, or the coarse flow constant isn't close to
10 (not ~1), the fit is wrong before looking at anything else.

## Where we left off

Waiting on a Learn run on `session-v1.14` on real hardware, then the
exported **Learn Throws CSV** and the resulting fitted profile numbers.
Expectation: lag ≈ 0.55 s, coarse flow ≈ 10 gr/s/rps, and a 40 gr charge
landing in the 6–8 s range end to end. When the CSV is dropped into this
repo, read it directly to check the fit rather than waiting for a summary.

## Known open items

- Learn-mode behavior on hardware *past* the initial fit (i.e. once tuned,
  running further charges) is only lightly tested.
- The WiFi OTA update's auto-reboot-after-flash path was fixed in v1.10, but
  only the file copy step has been confirmed on hardware — the automatic
  reboot itself hasn't been verified end to end.

## Build

Standard Pico SDK / CMake build; submodules must be checked out first:

```
git submodule update --init --recursive
cmake -S . -B build -DPICO_BOARD=pico2_w
cmake --build build
```

Output firmware is `build/app.uf2`. Flash either via USB (BOOTSEL) or over
WiFi through Settings > Firmware in the web portal.
