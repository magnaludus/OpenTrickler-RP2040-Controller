# OpenTrickler: Learn Edition

**A powder trickler that tunes itself.**

Point it at your powder, press Learn, and it works out how your two tubes flow, how far
behind your scale reads, and how close to the target the bulk can safely get - then it
keeps getting better with every charge you throw.

Firmware for a Raspberry Pi **Pico 2 W** (Pico W also builds) driving the
[OpenTrickler](https://github.com/eamars/OpenTrickler) hardware. This is a GPL-3.0 fork of
[eamars/OpenTrickler-RP2040-Controller](https://github.com/eamars/OpenTrickler-RP2040-Controller)
with a new brain on top - see [Credits](#credits).

> **This meters gunpowder.** Verify your charges on an independent scale, work up loads
> the usual careful way, and treat every number below as "what happened on one bench with
> one powder", not a promise. Developed and tested on a Pico 2 W.

---

## What makes it different

### Learn Powder - one button, a fitted profile

Learn runs 24 calibration throws (12 on the coarse tube, 12 on the fine) at a spread of
speeds, then fits what matters for *your* powder and *your* scale:

- flow rate of each tube, in grains per second per revolution
- how far behind the scale reads - **measured separately for coarse and fine**, because
  they genuinely differ (about 0.53 s versus 0.70 s on the test bench)
- dead time from motor start to first movement
- the scatter of where the bulk tube stops

From that it searches a grid of bulk speeds, landing speeds and handoffs and picks the
**tightest handoff that still meets your Time Goal**, then proves it with real confirm
throws. Miss your target success rate and it backs off and runs again by itself.

### Learn Style - Normal or Aggressive

One switch for how hard it pushes.

| | Normal | Aggressive |
|---|---|---|
| Time goal | 8.5 s | 7.0 s |
| Target success | 95% | 90% |
| Bulk safety / landing sigma | 1.5 / 2.0 | 1.0 / 1.5 |
| Handoff floor | 0.30 gr | 0.15 gr |
| Live tuner | careful | probes further, tolerates more misses |

Picking a style fills the numbers in on the portal, and every one stays editable. On the
test bench (NewProfile7, 26.5 gr, one powder) the same profile went from **10.3 s a charge
on Normal to 8.1 s on Aggressive**, with a tighter handoff (0.58 gr down to 0.37 gr).
That is a small sample; Aggressive is expected to throw a couple more outside the bracket
per hundred, so it is opt-in and Normal is the default.

### A live tuner that earns its keep

The tuner that runs after Learn was rebuilt from scratch and is **tested against a
simulated tube** before it ever touches hardware.

- **Safety first.** It centres your results on target (a learned aim trim) and holds the
  fine tube's landing speed to the accuracy margin you asked for.
- **Then speed.** It probes handoff, fine Kp, bulk speed and bulk taper one at a time, ten
  throws each, and keeps a step only if throws got measurably faster *and* landed no wider.
  No physics model in the loop to be wrong - the stopwatch decides.
- **Spends only spare margin.** A risky speed-up won't run unless the landing spread has
  headroom to give.
- **Watches for drift.** A new powder lot or a humid day slows throws or clusters misses;
  the tuner notices, backs off, and searches again.
- **Never worse than it found it.** If its best profile isn't clearly faster than the one
  it started from, it puts the original back.
- **Tell me what you're doing.** The portal shows it live: `tuner: Trying handoff +25% 4/10`.

In simulation, after the powder changed mid-session, a static profile threw 16 misses per
100 charges while the tuned one recovered to about 4.5.

### Lag-compensated, phase-aware control

Stopping on what the scale shows is stopping late. This firmware predicts where the weight
is heading using each tube's own measured lag, stops the bulk early on *predicted* weight,
and tapers the coarse tube into its stop so the last grains arrive slowly. A learned
landing trim then shifts the aim point so results centre on the target instead of piling
up against the low edge of the bracket.

### Know exactly what happened

- **Session statistics** - every throw logged (target, actual, error, total and coarse
  time, pass/under/over) with a live average and success rate, exportable as **CSV**.
- **Brackets** - Normal and Match in 0.02 gr steps, with an amber backlight while a
  session is running.
- **Learn throws CSV** - every calibration throw, so you can see the fit's raw data.

### A portal worth keeping open

Mobile-first web interface: live weight and timing, Learn on the bottom bar, gate controls
that only appear when the servo gate is enabled, self-clearing over/under popups, and the
tuner's status line. Full REST API underneath.

### Update over WiFi

Flash a new `.uf2` from **Settings > Firmware** - no pulling the board, no USB cable.
(BOOTSEL still works and recovers either board.)

### Built to be tested

Everything that decides a throw is checked by tests that run on a normal PC:

- a **charge-loop simulator** around a modelled tube and scale (`tests/tuner_sim.c`) that
  drives the real tuner code, including a port of the old tuner for honest comparison
- invariants fuzzed over random tubes and starting profiles, drift recovery, determinism
- regressions for the motor queue, OTA bounds, scale loss and cancel/top-up paths

```
python tests/tuner_regressions.py        # tuner behaviour on the simulated tube
python tests/tuner_eval.py               # static vs old tuner vs new tuner table
python tests/control_regressions.py      # firmware routines with hardware stubbed
```

---

## Install

### Download

Grab the `.uf2` for your board from the
[releases page](https://github.com/magnaludus/OpenTrickler-RP2040-Controller/releases) (or the
latest build from the
[Actions tab](https://github.com/magnaludus/OpenTrickler-RP2040-Controller/actions/workflows/cmake.yml)).

- **Pico 2 W** (RP2350) - developed and tested on this.
- **Pico W** (RP2040) - builds and the PIO fix is in, but it is not confirmed on hardware.

Hold **BOOTSEL**, plug in the USB cable, and copy the `.uf2` onto the drive that appears.
After that, further updates can go over WiFi.

### First run

1. Connect to the device's WiFi and open the portal (see
   [connecting to wireless](manuals/connect_to_wireless.md)).
2. Put your pan on the scale, pick a profile, and open **Learn**.
3. Choose a **Learn Style**, set a confirm charge weight, and run it. About eight minutes.
4. Throw charges. Leave the profile alone for the first thirty or so throws and let the
   tuner finish its first measure-and-probe cycle.

### Good to know

- **Settings you change in the portal are RAM-only until you save them.** Reboot loses them.
- **Set Point Mean Margin must stay above zero.** At 0.000 the scale can never "zero" and
  nothing will throw.
- **Compatibility:** this fork's EEPROM layout has diverged from stock. Flashing over (or
  back to) a stock build resets Charge Mode settings to defaults. Re-run Learn afterwards.

---

## Hardware and scales

### Supported scales

| Supported Scale              | Read | Force Zero (Tare)                 | Notes                             |
| ---------------------------- | ---- | --------------------------------- | --------------------------------- |
| A&D fx-i series (Std Format) | ✔️   | ✔️                                | Baud: **19200**, Format: 8d,1s,np |
| Steinberg SBS                | ✔️   | ❌                                 | Baud: 9600, Format: 8d,1s,np      |
| G&G JJ / JJB series          | ✔️   | ✔️(Unreliable)                    | Baud: 9600, Format: 8d,1s,np      |
| U.S.Solid JFDBS              | ✔️   | ❌                                 | Baud: 9600, Format: 8d,1s,np      |
| JM Science series            | ✔️   | ❌                                 | Baud: 9600, Format: 8d,1s,np      |
| Creedmoor Sports series      | ✔️   | ❌                                 | Baud: 9600, Format: 8d,1s,np      |
| Radwag R series              | ✔️   | ✔️                                | Baud: 9600, Format: 8d,1s,np      |
| Sartorius series             | ✔️   | ❌ (Not supported yet, but doable) | Baud: 9600, Format: **7d**,1s,np  |
| Generic Scale Driver         | ✔️   |                                   | Depends                           |

Learn and the lag-compensated control work best with a scale that accepts a software zero
(A&D fx-i is what this fork is developed against).

### Supported hardware

- Mini 12864 display module (rotary encoder, 3x Neopixel LED)
- Dedicated Neopixel LED (up to 16 chains)
- 2x miniature servo motors (TowerPro SG/MG90S or similar)
- 2x TMC2209 (STEP/DIR with 1-line UART)
- On-board EEPROM (up to 256 kbits)

### Connectivity

WiFi (2.4 GHz, AP or Station mode), web interface, RESTful interface, mDNS lookup.

### Modes

Charge Mode and Cleanup Mode.

---

## Using the mini 12864 display

1. From the main menu, select "Start".

    ![12864_main_menu](resources/main_menu_screen_mirror.png)

2. Provide the target charge weight in grain then press Next to continue.

    ![12864_select_charge_weight](resources/select_weight_screen_mirror.png)

3. Remember to put the pan on the scale.

    ![12864_waring_put_pan_on_scale](resources/put_pan_warning_screen_mirror.png)

4. Wait for the scale to stabilise at 0, or press the rotary button to force a re-zero.

    ![12864_wait_for_zero](resources/wait_for_zero_screen_mirror.png)

5. Wait for the charge to reach the set point.

    ![12864_wait for charge](resources/wait_for_charge_screen_mirror.png)

6. Once the set point is reached, remove the pan. The program restarts from step 4.

    ![12864_wait_for_cup_removal](resources/wait_for_cup_removal.png)

---

## Build from source

Reference: <https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf>

### Prerequisites (Windows)

[Git](https://gitforwindows.org/) and [VSCode](https://code.visualstudio.com/) are required. To
install the build dependencies, use the
[VSCode Raspberry Pi Pico extension](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico)
and create a pico-example project (any project triggers the download of pico-sdk). Select Pico
SDK **v2.1.1** when creating it. Verify the install under `C:\Users\<user name>\.pico-sdk`.

![pico_sdk_path](resources/pico_sdk_path.png)

### Get the source

    git clone https://github.com/magnaludus/OpenTrickler-RP2040-Controller
    cd OpenTrickler-RP2040-Controller
    git submodule update --init --recursive

The submodules can take up to five minutes.

### Configure and build

From PowerShell, load the environment, then configure for your board:

    .\configure_env.ps1
    cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DPICO_BOARD=pico2_w    # Pico 2 W
    cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DPICO_BOARD=pico_w     # Pico W

    cmake --build build --config Debug

The firmware lands at `build/app.uf2`. To open VSCode with the right environment, run
`.\run_vscode.ps1` (it is pre-configured for Pico 2 W; change `.vscode/settings.json` for Pico W).

Every push to `main` builds both boards on GitHub Actions and attaches the `.uf2` files.

---

## Credits

This project stands on the original **OpenTrickler** hardware and firmware by
[eamars](https://github.com/eamars) and contributors - the motor drivers, scale drivers,
display, WiFi stack and the original charge loop all come from there, and the upstream
community is the best place for hardware and build help:
[upstream repo](https://github.com/eamars/OpenTrickler-RP2040-Controller) and its
[discord](https://discord.gg/ZhdThA2vrW).

The Learn Edition additions - Learn Powder, Learn Style, the live tuner, lag-compensated
control, session statistics, brackets, WiFi update and the test bench - were built on top of
that foundation. This fork is not merged upstream and is not officially affiliated with it.

Licensed under the **GNU General Public License v3.0**; see [LICENSE](LICENSE).
