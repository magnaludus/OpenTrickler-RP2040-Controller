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

## This machine's calibration numbers

From the v1.11 bench CSV:
- Coarse tube: **10.1 gr/s per rps**, fine tube: **0.33 gr/s per rps**
- Scale lag: **0.55 s** — consistent across both tubes and all speeds tested

From a session-v1.14 Learn run (profile 8208XBR, confirm target 30gr, coarse
speed ceiling 6 rps but hardware-capped to ~0.83 rps):
- Fitted: coarse flow 10.676 gr/s/rps, fine flow 0.318 gr/s/rps, measured lag
  0.53s coarse / 0.74s fine, lag SD 0.19s, dead time 0.83s
- Fitted `coarse_stop_threshold` = **5.258 gr** — matches the ~5.3gr the user
  found by hand-tuning down from an earlier run's ~5.8gr fit. The auto-tune
  fit is landing in the right place; see "Known open items" below for why
  its own confirm phase still looked slow.
- Real 15-throw production session at 26gr target, same profile: **100%
  pass, avg 7.02s/throw, error mean −0.037gr, error SD 0.017gr, always
  under (never over) target.** This is the reference "it's working well"
  baseline — Learn changes should be checked against not regressing this.

## Where we left off

The `coarse_stop_threshold` mystery (5.8→5.3 by hand) is explained, not a
live bug: see "Known open items" for the confirm-phase vs. live-tuning gap.
No open ask right now — next real signal will be another Learn run or
session CSV the user drops in.

## Known open items

- **Confirm phase can't show the live-tuned steady state.** `learn_post_throw()`
  (`src/charge_mode.cpp:167`) only tightens `coarse_stop_threshold` after 5
  *consecutive* clean passes (`clean_streak >= 5`, then a 5% nudge). Learn's
  own confirmation run defaults to `LEARN_CONFIRM_THROWS = 5`
  (`src/learn_mode.h:14`), so it can pass all 5 and still never trigger a
  single tightening step — a 6th throw would be needed to see it. This is
  why a Learn confirm can show a much slower avg time (e.g. 13.4s) than what
  the machine actually settles into over a longer real session (e.g. 7.02s
  over 15 throws) — not a bug in the fit itself, just a short sample window.
  If we want Learn's own numbers to reflect steady-state performance, either
  raise `confirm_throws` well past 5, or teach `fit_profile()`/confirm to
  account for expected live-tuning convergence. Not done — flagging only.
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
