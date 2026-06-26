# HITL / Sim-on-Hardware Instructions

This document covers building and running the **Sim-on-Hardware (SIH)** firmware for
the IF800 (CubeOrangePlus), used for hardware-in-the-loop style testing. The board
runs ArduPilot's native SITL physics model *on the real flight controller*, with
simulated IMU/GPS/baro/compass, while real peripherals (e.g. the Gremsy gimbal and
its MAVLink camera) stay connected and active.

> A separate **SITL** (desktop simulation) section will be added to this doc later.

---

## 1. Sim-on-Hardware (SIH)

### 1.1 What gets built

The SIH firmware is the **production `IF800-CubeOrangePlus` board** firmware, plus a
build-time overlay (`sih-extra-hwdef.dat`) that:

- turns on the on-board simulator (`env SIM_ENABLED 1`),
- caps compass instances and trims unused features for flash,
- forces the simulated vehicle to a quad (`AP_SIM_FRAME_CLASS MultiCopter`,
  `AP_MOTORS_FRAME_QUAD_ENABLED`),
- keeps the gimbal + MAVLink camera backends compiled in.

Files live in `libraries/AP_HAL_ChibiOS/hwdef/IF800-CubeOrangePlus/`:

| File | Role |
|------|------|
| `hwdef.dat` | Production board definition (includes `../CubeOrangePlus/hwdef.dat`). |
| `defaults.parm` | **The embedded default parameters** (see §1.4). |
| `sih-extra-hwdef.dat` | SIH build overlay (`env SIM_ENABLED 1`, frame, feature trims). |

### 1.2 Prerequisites

**ARM cross-compiler** — `gcc-arm-none-eabi-10-2020-q4-major` (the version this
ArduPilot 4.5.x tree expects). Install to your home dir (no sudo):

```bash
cd ~
wget https://firmware.ardupilot.org/Tools/STM32-tools/gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2
tar xjf gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2
export PATH="$HOME/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"   # add to ~/.bashrc to persist
```

**Python ≤ 3.11.** This tree bundles an old `waf` (`modules/waf`) that still does
`import imp`, which was **removed in Python 3.12**. Building under a 3.12+ interpreter
fails with `ModuleNotFoundError: No module named 'imp'`. Use a 3.10/3.11 venv:

```bash
python3.11 -m venv ~/venv-ap311
source ~/venv-ap311/bin/activate
pip install --upgrade pip
pip install "empy==3.3.4" pexpect future pymavlink intelhex dronecan
```

> `empy` **must** be 3.3.x — empy 4.x breaks ArduPilot's code generation.

### 1.3 Build

From the repo root, with the toolchain on `PATH` and the Python ≤3.11 venv active:

```bash
./waf configure --board IF800-CubeOrangePlus \
  --extra-hwdef=libraries/AP_HAL_ChibiOS/hwdef/IF800-CubeOrangePlus/sih-extra-hwdef.dat

./waf copter            # build only
./waf copter --upload   # build and flash the connected board
```

A successful configure prints:

```
Default parameters path from hwdef: .../IF800-CubeOrangePlus/defaults.parm
```

Artifacts land in `build/IF800-CubeOrangePlus/bin/` (`arducopter.apj`,
`arducopter_with_bl.hex`, etc.).

> **Note:** do **not** pass `--default-param=...`. It is silently ignored here (see
> §1.4) and is not needed — the board's `defaults.parm` is picked up automatically.

### 1.4 Where the parameters come from

This is the part that trips people up, so it is spelled out in full.

**The embedded defaults are `defaults.parm` in the board's hwdef directory** —
`libraries/AP_HAL_ChibiOS/hwdef/IF800-CubeOrangePlus/defaults.parm`.

During `./waf configure`, `chibios_hwdef.py:write_default_parameters()` looks for a
`defaults.parm` next to the board's `hwdef.dat` and, because this board does **not**
set `FORCE_APJ_DEFAULT_PARAMETERS`, embeds it into the firmware via **ROMFS** (as the
file `defaults.parm`). At boot `AP_Param` loads it through
`load_embedded_param_defaults()`.

The processed copy that actually gets embedded is written to
`build/IF800-CubeOrangePlus/processed_defaults.parm` — inspect it to confirm what
shipped.

**Why `--default-param=<file>` does nothing here.** `chibios_hwdef.py` resolves the
CLI path as `os.path.join(dirname(hwdef), args.params)`. Passing a full relative path
(e.g. `libraries/.../sih-defaults.parm`) produces a non-existent joined path, so the
check fails and the code falls back to the board's `defaults.parm`. The CLI file is
ignored without warning. For that reason all SIH defaults are kept **directly in
`defaults.parm`** rather than in a separate `--default-param` file.

> This board variant is built **SIH-only**, so `defaults.parm` carries the simulator
> configuration (e.g. `GPS_TYPE 100`, `AHRS_EKF_TYPE 10`, `SIM_*`). Do not flash a
> build of this board from this branch onto a real aircraft.

**Parameter precedence (low → high):** compiled-in defaults → embedded `defaults.parm`
→ values saved in the board's flash/FRAM → live `param set` from a GCS. A value
previously written to the board's storage **overrides** the embedded default. After
flashing, refresh params and confirm the key values, or `param set` them once if the
board still holds stale values from a prior session.

### 1.5 What the SIH defaults configure

Beyond the standard simulated-sensor setup, `defaults.parm` configures the gimbal and
camera signal chain:

| Area | Params | Purpose |
|------|--------|---------|
| Sim sensors | `AHRS_EKF_TYPE 10`, `GPS_TYPE 100`, `SIM_MAG1_DEVID`, INS cal offsets, `SIM_RATE_HZ 400`, `SCHED_LOOP_RATE 400` | Run the native SITL model on the board. |
| Sim GPS port | `SERIAL3_PROTOCOL 5` | The SITL GPS backend (type 100) only starts if a serial port is set to GPS protocol. |
| Gimbal/camera link | `SERIAL4_PROTOCOL 2`, `SERIAL4_BAUD 115`, `SERIAL4_OPTIONS 0` | UART8 is MAVLink2 so the Gremsy gimbal (`MAV_TYPE_GIMBAL`) and MAVLink camera (`MAV_TYPE_CAMERA`) heartbeats are routed and the devices are discovered. |
| Gimbal | `MNT1_TYPE 6` (Gremsy), `MNT1_DEFLT_MODE 3` (RC_TARGETING), `MNT1_*` ranges | Gremsy MAVLink gimbal. |
| Camera | `CAM2_TYPE 6` (MAVLink CamV2), `CAM2_MNT_INST 0` | CamV2 is the backend that requests + relays `CAMERA_INFORMATION`. |
| Gimbal RC control | `RC15_OPTION 214` (Mount1 yaw), `RC9_OPTION 163` (yaw lock/follow) | RC stick/wheel control in RC_TARGETING mode. GCS/mission `DO_MOUNT_CONTROL` works without these. |

> The Gremsy gimbal and the MAVLink camera are physically wired to **UART8 / SERIAL4**.
> If that wiring changes, update the `SERIAL4_*` (or relevant `SERIALx_*`) lines so the
> device's port is MAVLink2 — otherwise the gimbal will not move and
> `CAMERA_INFORMATION` reports `UNSUPPORTED`.

### 1.6 Verifying on the board

After flashing, connect a GCS and check:

- A component with `MAV_TYPE_GIMBAL` and one with `MAV_TYPE_CAMERA` appear on the link.
- The gimbal responds to a commanded angle (GCS/mission) and to RC yaw input.
- `CAMERA_INFORMATION` returns vendor/model instead of `UNSUPPORTED`.
- `SERIAL4_PROTOCOL` reads `2` and `GPS_TYPE` reads `100` (if a stale stored value
  shadows the default, `param set` it once).

---

## 2. SITL (desktop simulation)

_To be added._
