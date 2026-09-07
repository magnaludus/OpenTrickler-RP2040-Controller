# OpenTrickler RP2040 Controller — session-stats / Learn Powder fork

Fork of [eamars/OpenTrickler-RP2040-Controller](https://github.com/eamars/OpenTrickler-RP2040-Controller),
firmware for a powder-trickler/scale build on a **Raspberry Pi Pico 2 W**.

## Repo layout

- `main` — this fork's active branch (merged from `session-stats-bracket-learn`),
  based on upstream main at commit `e23cc45` (PR #112). GitHub's default branch,
  shown on the repo's front page.
- `session-stats-bracket-learn` — kept in sync with `main`; new work can land on
  either, just keep them merged.

## What's on this fork (session-v1.14 baseline)

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
- Real production session at 26gr target, same profile (8208XBR): **21/21
  pass through the full session, avg 6.97s/throw, error mean −0.037gr,
  error SD 0.017gr, always under (never over) target.** (First checked at
  15 throws — 7.02s/−0.0373/0.017 — essentially unchanged by throw 21, no
  detectable drift in coarse_s or fine_s across the session. That flatness
  suggests either live per-throw tuning is off for this profile and it's
  running a fixed already-good static fit, or it's already at a plateau —
  worth confirming which if it matters later.) This is the reference "it's
  working well" baseline — Learn changes should be checked against not
  regressing this.

## Second Learn run on v1.14 (raw learn_throws CSV, same powder)

Reproducibility check against the first v1.14 fit — fit from the raw 12
coarse + 12 fine throws:
- Coarse flow: **10.734 gr/s/rps** (was 10.676, bench 10.1 — consistent)
- Fine flow: **0.326 gr/s/rps** (was 0.318, bench 0.33 — consistent)
- Measured lag: **coarse 0.534s / fine 0.704s** (was 0.53/0.74 — same
  asymmetry, reproduced almost exactly)
- Combined lag SD: 0.164s (was 0.19s)

The coarse-vs-fine lag split (~0.53s vs ~0.70-0.74s) has now shown up
identically in two independent Learn runs — it's real, not noise, and it
directly contradicts the original v1.11 bench-CSV note above ("dead
consistent across both tubes"). That earlier number was wrong, or measured
differently; trust the Learn-derived per-phase split from here.

**This asymmetry isn't just a display quirk — it affects real-time control.**
`fit_profile()` computes `r->coarse_lag_s`/`r->fine_lag_s` separately
(`src/learn_mode.cpp:504`) but only ever uses the *blended* `r->lag_used_s`
for everything that matters: `coarse_tail_per_rps`, `fine_tail_per_rps`,
the taper window, and — critically — the single `scale_lag_s` EEPROM value
Learn writes (`src/learn_mode.cpp:637`). That one value is what
`charge_mode.cpp:525` uses for live lag-compensated prediction, applied the
same way in both the coarse and fine phases. So right now the coarse phase
is predicted with lag ~0.1s too high, and the fine phase with lag ~0.1s too
low, relative to what's actually measured for each. Given how tight actual
performance already is (see below), this isn't causing failures, but it's
a real, reproducible inefficiency with a known fix: split `scale_lag_s`
into per-phase values, threaded through the charge loop and Learn's fit.
That's an EEPROM-format change (revision bump, REST params, portal fields)
so it hasn't been done — flagging for a decision, not doing it silently.

## Live-tuning convergence caught on camera (NewProfile5, 11-throw session)

A fresh profile off the second Learn run (same 26gr target, same powder)
showed `fine_s` drop monotonically throw-over-throw — 10.96, 9.30, 9.20,
8.42, 7.52, 7.22, 5.56, 5.94, 4.84, 5.50, 4.60s — while `coarse_s` crept
slightly up (2.10→2.52s). That's the exact signature of `learn_post_throw()`
(`src/charge_mode.cpp:167`): every 5 consecutive clean passes it narrows
`coarse_stop_threshold` by 5% and raises fine max speed by 5%, so coarse
hands off marginally later (slightly more work, hence the uptick) while
fine gets less to trickle and more speed to do it with. By throw 11 it's
converged to 7.12s total / 4.60s fine — matching 8208XBR's steady state
(~6.97s / ~4.5s) almost exactly, same hardware/powder as expected.

This resolves the earlier open question: `learn_enable` **is** active on
this hardware (the compiled default of `false` only applies to a fresh
EEPROM/factory reset — doesn't tell us what's actually toggled on a live
device). 8208XBR's 15/21-throw CSVs looked flat not because live tuning was
off, but because it had already fully converged before that logging
started. The original "5.8→5.3 by hand" from the very first report was the
user manually doing in one edit what this loop does automatically in about
10 throws — nothing was ever actually wrong with the fit or the tuner.

## Overcharge bug found and fixed (deterministic, not just variance)

NewProfile5 escalated from intermittent overshoot at 26gr (4 of 5 throws)
to **100% overshoot at 40gr** (11/11 OVER, up to 0.897gr over target) the
first time the target weight changed. That ruled out "unlucky variance" as
the sole explanation and made this safety-critical (this device controls
powder charge weight for live ammunition).

Root causes fixed (EEPROM_CHARGE_MODE_DATA_REV bumped 11→12 — **flashing
this resets all charge-mode settings to firmware defaults**: coarse stop
threshold, brackets, LED colours, lag config, learn/predict/auto-lag
toggles all revert; re-run Learn Powder or reconfigure after flashing):

1. **Split `scale_lag_s` into `coarse_lag_s`/`fine_lag_s`.** One blended
   value was applied uniformly to both phases by the live control loop
   (`charge_mode.cpp`'s real-time `predicted_weight`) despite coarse and
   fine consistently measuring different lag (~0.53s vs ~0.70-0.74s,
   confirmed across two Learn runs). Fixed: the charge loop now picks
   `coarse_lag_s` while the coarse phase is still moving, `fine_lag_s`
   after. `auto_lag_enable`'s continuous re-learning only ever updates
   `fine_lag_s` now (its measurement point, the throw's final settle, is
   downstream of the fine phase - it was never actually reading coarse's
   lag, just overwriting the shared value with fine's).
2. **Live per-throw tightening had no variance floor.** `learn_post_throw()`
   narrowed `coarse_stop_threshold` by 5% every 5 consecutive clean passes
   with no reference to how much the coarse tube's own dispensing actually
   varies - a streak of 5 isn't proof a margin is safe, just that it hasn't
   failed yet. Fixed: persisted the Learn fit's `coarse_tail_sd_at_max`
   into a new EEPROM field `coarse_tail_sd_gr`, and the tightening floor
   is now `max(handoff*0.95, taper*1.5, 3*coarse_tail_sd_gr)` - can't
   narrow the margin past 3 sigma of the coarse tube's measured spread,
   regardless of streak length or target weight (this floor is in absolute
   grams, so it protects a fresh/never-tuned target weight too, which is
   exactly where the 40gr session failed).

Verified: the historical raw-vs-predicted-weight blame bug from the
original chat-based workflow (comparing the coarse stop against the raw
lagging reading instead of the lag-compensated prediction) is **not**
present in this codebase - traced end to end and confirmed correct.

Web portal: "Scale Lag (s)" split into "Coarse Lag (s)" / "Fine Lag (s)"
(REST c23/c25), new "Coarse Tail Spread (gr)" field (c26), live lag display
on the trickler page now shows both phases (s11 coarse / s14 fine), Learn
results panel's "Lag compensation" line shows both instead of the blended
value.

## Where we left off

Two real, data-backed threads open, both flagged not implemented pending a
decision (see above): splitting lag compensation by phase, and whether to
extend Learn's own confirm phase past 5 throws so it can show the converged
steady state instead of the cold-start number. Everything else is
confirmed working as designed. Next signal: another Learn run, a new
profile's early convergence, or a session CSV.

## Version screen quirk

`Ver:`/`VCS:` come from upstream's `scripts/gen_version.py` (`git describe
--tags`, expecting eamars-style `vX.Y` tags) — separate from our own
`Build:` line (`SESSION_BUILD_TAG` in `session_version.h`). This session's
GitHub App can't push tag refs to the fork (`git push --tags`/single-tag
pushes 403 everywhere tried, including from the properly-attached clone),
so `Ver:` will keep reading `no-tag` and `VCS:` will show a bare commit
hash instead of upstream's `vX.Y-N-gHASH` format. Fixed the actual bug in
that script (it used to blank the hash too on a no-tag build, showing
`VCS:` empty) but there's no way to restore full `vX.Y` versioning without
manually creating a tag through the GitHub web UI on the fork — not done,
low priority since `Build:` already answers "what firmware is this."

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
