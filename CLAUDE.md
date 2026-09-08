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

## Handoff was stuck too wide: pooled-variance bug, not a tuning limit

After the overcharge fix, the user reported the coarse handoff was still
5-8gr (expected more like 2-2.5gr, maybe 1-1.5gr) and that live tuning
wasn't closing the gap - the new safety floor (`coarse_tail_sd_gr`) had
made it *worse* than before the fix, not better.

Root cause: `lag_stats()` computed **one** standard deviation pooled across
both coarse and fine throws. Coarse and fine consistently measure
different mean lag (~0.53s vs ~0.70s), so pooling them inflates the
apparent spread by the *gap between the means*, not just real noise.
Computed from actual Learn data: coarse's true per-phase SD is **~0.019s**
(right at the numeric floor - genuinely rock-solid), while the old pooled
figure was **~0.16-0.19s**, ~9x larger. Both the original static fit's
handoff formula and the new live-tuning safety floor were built on that
inflated pooled number, so the coarse handoff was landing around 5gr from
day one (not just after the recent floor), and the floor then locked that
inflated value in as a hard minimum.

Fix (not removing the floor - correcting what feeds it): `lag_stats()` now
also returns per-phase SD; `fit_profile()` uses `coarse_lag_err`/
`fine_lag_err` from each phase's own SD instead of one blended `lag_err`,
for the coarse handoff formula, the fine landing-speed error budget, and
both `coarse_tail_sd_at_max`/`fine_tail_sd_at_max` (which feeds the live
floor via `coarse_tail_sd_gr`). Expected effect: the coarse-variance term
in the handoff formula drops close to its own floor, so the **taper x 1.5**
floor (a real, unrelated physical constraint - the fine ramp needs that
much room) becomes the binding constraint instead, landing handoff in the
~3-4gr range rather than ~5-6gr, with live tuning now free to actually
narrow it further from there instead of hitting an inflated wall
immediately.

Needs a fresh Learn Powder run to take effect (old profiles keep whatever
handoff they already have baked in).

That prediction was only half right, and the follow-up review below found
why: the pooled-SD fix did drop the handoff (4.56gr -> 3.63gr on the real
Learn data) but `taper x 1.5` then became the binding floor at ~3.6gr, and
the *taper itself* was the thing that needed fixing.

## Handoff and speed, root-caused properly

Re-derived from the raw Learn throws (`opentrickler_learn_throws_1.csv`,
24 throws) rather than reasoned about in the abstract. Per-phase numbers:
`coarse_k` 10.734 gr/s/rps, `fine_k` 0.3262 gr/s/rps, coarse lag
**0.534s +/- 0.019s** and genuinely flat across all four speed levels
(0.543 / 0.537 / 0.530 / 0.527); fine lag 0.704s +/- 0.201s, but that
spread is a *speed dependence*, not noise (0.742s at 0.6rps, 0.885s at
1.85rps, 0.485s at 3.9rps).

Three findings, in order of how much they cost:

1. **The fit never swept fine speed.** `fmax = f_hi` was hardcoded to the
   ceiling. But the fine phase takes about the *same* time at any fine
   speed: the taper scales with fine flow, so `taper / mean taper rate`
   collapses to ~6 x lag and `(handoff - taper) / flow` to ~1.5 x lag,
   both independent of speed. Running the fine tube flat out therefore
   buys almost nothing in time while forcing a proportionally wider
   handoff. Fixed by sweeping fine max alongside coarse max.

2. **The taper used the pooled lag mean** (`lag_used_s`, 0.619s) for what
   is purely a fine-phase quantity - it should use `fine_lag_s`. Same
   class of bug as the pooled SD, one layer down.

3. **The coarse sweep could pick a speed 3x outside its own data.** The
   ladder deliberately stops at `LEARN_COARSE_FLOW_CAP_GPS` (18 gr/s,
   ~1.68rps on this hardware) but the sweep ran to the motor limit of
   5rps, extrapolating `coarse_k` far past anything measured. Now capped
   at `LEARN_COARSE_EXTRAP_MULT` (1.25x) past the fastest ladder throw.

Also: **live tuning could never tighten the handoff at all.** The fit set
`handoff = taper * 1.5` and the live tuner floored tightening at
`taper * 1.5` - the same number - so `learn_post_throw()`'s narrowing
branch was a no-op from a fresh fit, which is exactly the "it isn't
learning" the user reported. Live floors are now deliberately below what
the fit hands over (`taper * 0.75` and a bare `3 x coarse_tail_sd_gr`,
versus the fit's full taper and 1.5 x that 3 sigma), so repeated clean
throws can spend the fit's safety factor but never the 3 sigma itself.

New handoff rule: `max(0.30, 1.5 x 3 sigma coarse stop scatter + 0.15,
taper)`, where the 1.5 (`LEARN_COARSE_STOP_SAFETY`) applies only when lag
compensation is on - with it off the handoff already has to swallow the
whole tail at 3x and stacking the factor would just make an uncompensated
profile needlessly slow. Because every grid point then carries the same
margin, the sweep picks the **fastest** predicted throw rather than the
slowest coarse that scrapes the goal.

Simulated against the real Learn data (all four charge weights the user
runs, 26/30/40/42.5gr - the fit is target-independent here, numbers shown
for 40gr):

| | handoff | Cmax | Fmax | predicted |
|---|---|---|---|---|
| original | 4.56gr | 0.83 | 4.00 | 9.15s |
| after pooled-SD fix | 3.63gr | 1.25 | 4.00 | 7.45s |
| **after this fix** | **2.30gr** | 2.10 | 3.33 | **6.10s** |

Handoff lands in the 2-2.5gr band the user asked for while getting
*faster*, not slower - min-handoff and min-time turn out to point the same
way once the formula is honest, because every grain handed to the slow
fine tube costs ~0.8s at best. Cushion over the coarse tube's measured
3 sigma scatter is 1.7x, and live tuning may narrow to 1.72gr (25% of
headroom) on repeated clean throws.

Also fixed: if no grid point was usable (every margin-driven handoff
exceeded half the charge) `best_handoff` was left at 0.0, which would let
the coarse tube run the whole way to target. Now falls back to the slowest
bulk and the narrowest legal handoff. Latent in the original too.

Not changed, deliberately: `fine_lag_s` is speed-dependent (0.485s at fine
max, 0.885s mid-ramp) and `auto_lag_enable` re-learns it from the final
settle, i.e. at landing speed. So the fine phase over-predicts early and
the controller's `new_speed = 0` hold kicks in sooner than it needs to.
That costs time, not accuracy, and the much smaller handoff shrinks the
window where it matters. Flagged, not fixed - it needs its own data.

## Objective: hand the bulk as much as it can, on two user dials

Follow-up from the user: the handoff shouldn't be a number the fit happens
to land on, it should be adjustable, and Learn should be getting bulk as
close as it can without overthrowing. (`coarse_stop_threshold` was already
editable - the misread was that 2.30gr looked hardcoded when it was a
computed result. Worth keeping in mind for how results get presented.)

Objective changed from *fastest predicted throw* to **tightest handoff
that still meets the time goal**, fastest on a tie, falling back to
fastest only when nothing meets the goal. Safe to optimise this way
because every grid point already carries the same margin by construction,
so tightening the handoff never spends safety - it spends time.

That makes the two existing/new settings a clean pair of dials:

- **Time Goal** (`l6`, default now **7.0s**, was 8.5s) is the trade. The
  fit spends every second under it buying a closer handoff. At 40gr:
  6.5s -> 1.84gr, 7.0s -> 1.61gr, 8.5s -> 0.99gr, 10s -> 0.83gr.
- **Bulk Safety Factor** (`l9`, new, default **1.5**, clamped 1.0-5.0) is
  the margin - the multiple of the coarse tube's measured 3 sigma stop
  scatter the handoff must cover. At 40gr / 7.0s goal: 1.0x -> 1.15gr,
  1.5x -> 1.61gr, 2.0x -> 1.95gr, 3.0x -> 2.85gr. Raise it on overthrows.

At the new defaults the bulk carries ~96% of the charge at every weight
the user loads (26/30/40/42.5gr), throws land ~6.8-6.9s predicted.

`EEPROM_LEARN_CONFIG_REV` 2 -> 3 for the added field. This resets **Learn
Powder settings only** - learn config lives at its own EEPROM base
address, so profiles, charge mode config and `coarse_stop_threshold` are
untouched. Note the default Time Goal changes with the reset.

Also fixed while in `http_rest_learn_config`: the range clamps ran *after*
the EEPROM save, so an out-of-range value could be written and reloaded on
the next boot. Clamps now run first.

## Predicted throw time was ~2x optimistic: the taper is exponential

User reported Learn predicting 5-6s against a real 10-12s. This stopped
being cosmetic the moment the time goal became the dial that sets the
handoff - the dial was calibrated against a fiction.

Validated against a fresh Learn run the user posted (portal screenshot,
their own hardware): `coarse_k` 10.773, `fine_k` 0.3104, coarse lag 0.52s,
fine lag 0.76s, dead time 0.86s, fine max/min 4.00/0.19 rps, `fine_kp`
1.421, taper 2.815gr, handoff 2.815gr. Portal said **predicted 5.8s
(coarse 1.0, fine 4.8)**; the confirm set ran **10/10 pass, avg 11.8s**.

Root cause: `predict_throw()` modelled the taper as a linear ramp and
divided the window by the *arithmetic mean* of the two ramp endpoints. But
the controller sets `speed = fine_kp x error`, so the approach is
**exponential** - the tube spends most of the window crawling near its
landing speed. Analytically:

    taper traverse = tau x (ln(fmax/fmin) + 1),  tau = taper / flow at fine max

and since `taper = LEARN_FINE_TAPER_TAIL_MULT x fine_lag x flow at fine
max`, tau reduces to `3 x fine_lag` - so **fine-phase time is set by the
fmax/fmin ratio and the tube's lag, not by how fast the tube is run.**

On the user's numbers: linear model 4.33s, exponential 9.18s (2.1x). Add
the 0.86s dead time, which was not modelled at all, and predicted total
goes 5.81s -> **11.52s against an actual 11.8s**. The old figure of 5.81s
reproduces the portal's 5.8s exactly, so the diagnosis is confirmed both
ways.

Second finding from the same formula: the landing speed was pinned at
`fmin_land` (LEARN_FINE_LAND_FLOW_GPS / fine_k = 0.19 rps, the "couple of
kernels a second" heuristic) giving an fmax/fmin ratio of 21 and over 9
seconds of trickling - while `fmin_err`, the bracket arithmetic, said 0.77
rps would still settle inside the bracket. That is a 4x lever on the
dominant term, left unused. The fit now sweeps the landing speed
(`LEARN_LAND_STEPS`, slowest first) from `fmin_land` up to `fmin_err`,
preferring the slowest that meets the goal; `fmin_err` remains a hard
accuracy ceiling that is never exceeded.

Objective tie-break is now: tightest handoff, then *slowest* landing (the
extra speed was not needed), then fastest.

Replayed on the user's own constants at a 26gr charge, the fixed model
gives handoff 1.33gr with fine max 1.67 / fine min 0.77 and an honest
7.50s - against 2.815gr and a real 11.8s today. The fine phase drops from
~9.2s to ~4.3s almost entirely by landing four times faster.

Note the 7.0s default goal is now *not* reachable on this hardware (best
is ~7.5s), so `meets_time_goal` reports false and the fit falls back to
fastest. That is honest rather than broken - raising the goal to 8s makes
the tightest-handoff objective engage again and yields 0.82gr at 7.84s.

## Stale live-tuning baseline (serious), and confirm-throw suppression

Found while answering "should per-throw tuning be suppressed during Learn's
confirm throws?" - it should, but chasing it turned up a worse bug next to
it.

`learn_post_throw()` bounds every adjustment to 50-140% of a baseline
captured on the **first throw after boot** and never refreshed. Nothing
else that writes the profile - `learn_mode_apply_to_profile()`, a portal
edit - told it to re-take that baseline. So the window stayed anchored to
whatever was loaded at power-on, and because `learn_bound()` clamps *into*
the window from both sides, an out-of-date baseline doesn't merely loosen
the guard rails, it actively drags the profile back toward the values that
were just replaced.

Worked through with the user's real numbers - boot on the old profile
(handoff 5.258gr, coarse max 0.83rps), run Learn, fit applies 1.61gr /
2.10rps. Bound window is still 2.63-7.36gr and 0.41-1.16rps, so:

- five **clean** throws want to tighten the handoff to 1.53gr and instead
  get 2.63gr - the reward for a clean streak was a *wider* handoff
- one **over** wants coarse max at 1.93rps (a 8% trim) and gets 1.16rps,
  a 45% cut

So a fresh fit got pulled apart by the first adjustment after it. This is
almost certainly part of what the user was seeing as "it isn't learning",
and it applies to manual portal edits too, not just Learn.

Fix is self-healing rather than a set of hooks: `learn_state_t` now also
stores the values the tuner itself last left behind (`seen_*`). If the
profile or the handoff differs from those at the top of the next throw,
something else moved it, so the baseline is re-taken from the current
values and the clean streak resets (throws that ran clean on a different
profile vouch for nothing). Covers Learn applying a fit, back-off rounds,
portal edits and profile reloads without any cross-module calls.

Separately, per-throw tuning is now suppressed during Learn's confirm set
(`charge_mode_learn_set_suppressed()`, held for the whole confirm loop
including back-off rounds). Confirmation is a *measurement* of the fitted
profile: tuning it mid-set means the pass rate that decides whether to back
off describes a profile that moved while being measured, and on a passing
round the saved profile silently differs from what the results screen
reports. `learn_mode_menu()` has a single return, so the flag cannot leak.

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
