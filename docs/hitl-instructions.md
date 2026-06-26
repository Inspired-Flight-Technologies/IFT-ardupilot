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

#### Reset parameters on first SIH boot (required)

The IF800 is a production board repurposed for SIH, so its flash/FRAM still holds
the **real** sensor calibration from production firmware. Parameter space is shared
between the real and simulated aircraft, and the sensor suites differ
([ArduPilot SIH docs](https://ardupilot.org/dev/docs/sim-on-hardware.html)). The
simulated IMUs/compasses register fewer instances than production, so the sim never
overwrites the stale extra device IDs, and embedded `defaults.parm` **cannot** clear a
value already saved to storage. Symptoms of skipping this step:

- The board demands an **accelerometer calibration** even though `INS_ACC*` cal offsets
  are in `defaults.parm`. A stale `INS_ACC3_ID` (the disabled aux IMU) left in storage
  trips the "missing accel" branch of `accel_calibrated_ok_all()`.
- The board demands a **compass calibration** ("Compass not calibrated"). `COMPASS_OFS`
  reads back as zero (the embedded non-zero offsets are shadowed by saved zeros) and/or
  a stale `COMPASS_DEV_ID3`/`COMPASS_PRIO3_ID` from production lingers.

Neither can be fixed by `param set` of individual IDs — do a full storage wipe so the
embedded SIH defaults apply and the simulated sensors re-register cleanly:

1. (Optional) Save the board's current parameters first if you will reflash production
   firmware later.
2. Set `FORMAT_VERSION 0` and **reboot**. On the next boot the param header no longer
   matches, storage is erased (`AP_Param::erase_all()`), and the embedded SIH defaults
   are re-applied. (Equivalent: Mission Planner *Reset to Default*, or
   `MAV_CMD_PREFLIGHT_STORAGE` param1=2.)
3. Reboot once more and confirm `COMPASS_OFS_X` reads `5` (not `0`), `COMPASS_DEV_ID`
   and `INS_ACC_ID` are populated by the sim backend, and no accel/compass cal is
   demanded.

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

This section covers running **desktop SITL** via `sim_vehicle.py` while a **real**
Gremsy gimbal + MAVLink camera are connected to the host over USB/serial. Unlike SIH
(§1), nothing runs on the flight controller — the autopilot is a process on your PC,
and a physical UART is bridged into one of its simulated serial ports.

### 2.1 The key gotcha: SITL serial ports are not real UARTs

In desktop SITL the **byte transport** for each serial port is hard-wired in code and
is independent of the `SERIALx_*` params. See
`libraries/AP_HAL_SITL/SITL_State.h` (`_serial_path[]`):

| Port | Default transport | Notes |
|------|-------------------|-------|
| SERIAL0 | `tcp:0:wait` | console (TCP 5760) |
| SERIAL1 | `tcp:2` | MAVLink (TCP 5762) |
| SERIAL2 | `tcp:3` | MAVLink (TCP 5763) |
| SERIAL3 | `GPS1` | **simulated** GPS |
| SERIAL4 | `GPS2` | **simulated** 2nd GPS |
| SERIAL5–8 | `tcp:5`…`tcp:8` | |

So by default **SERIAL4 is wired to a fake `GPS2` simulator**, not to a physical UART.
Setting `SERIAL4_PROTOCOL 2` alone does nothing — your gimbal's MAVLink bytes never
reach the autopilot, the gimbal won't move, and `CAMERA_INFORMATION` reports
`UNSUPPORTED`. You must point SERIAL4 at the real device on the command line.

> This is the desktop-SITL equivalent of the SIH wiring note in §1.5: the device has to
> be on a MAVLink2 port **and** that port has to actually carry the device's bytes.

### 2.2 Bridging a real serial device into SITL

`sim_vehicle.py` forwards `-A` args to the SITL binary, which understands a
`uart:<path>:<baud>` device string (see `libraries/AP_HAL_SITL/UARTDriver.cpp`,
`_parse_args` / `uart:` case):

```bash
-A "--serial4=uart:/dev/ttyUSB0:115200"
```

Confirm the device path (the gimbal usually enumerates as `/dev/ttyUSB0`; check
`dmesg | tail` after plugging it in). For a name that is stable across replug/reboot,
use `/dev/serial/by-id/<id>` from `ls -l /dev/serial/by-id/` instead.

Make sure your user can open it (add to the `dialout` group once, then re-login):

```bash
sudo usermod -aG dialout "$USER"
```

The `:115200` in the string sets the host UART baud directly; if SITL prints
`Failed to open (...)` at startup it is a wrong path or a permissions problem, and if
it prints `Opened /dev/...` the bridge is up.

### 2.3 Parameters: the board defaults do NOT apply here

The board's `defaults.parm` / SIH config from §1 are embedded in **board firmware** and
are **not** loaded when you run desktop SITL. You must supply the gimbal/camera params
explicitly. Keep them in a small file (repo root `gimbal-sitl.parm`):

```
# --- serial link to the real Gremsy gimbal/camera on SERIAL4 ---
SERIAL4_PROTOCOL 2
SERIAL4_BAUD 115
SERIAL4_OPTIONS 0

# --- Gremsy MAVLink mount ---
MNT1_TYPE 6
MNT1_DEFLT_MODE 3
MNT1_OPTIONS 1
MNT1_RC_RATE 90
MNT1_PITCH_MIN -90
MNT1_PITCH_MAX 90
MNT1_ROLL_MIN -45
MNT1_ROLL_MAX 45
MNT1_YAW_MIN -319
MNT1_YAW_MAX 319

# --- MAVLink camera tied to mount instance 0 ---
CAM2_TYPE 6
CAM2_MNT_INST 0

# --- RC stick control of the gimbal (MNT1_DEFLT_MODE 3 = RC_TARGETING) ---
# RC9  = 163 MOUNT_LOCK  (yaw lock vs follow toggle)
# RC15 = 214 MOUNT1_YAW  (yaw input -- this is the stick that moves yaw)
RC9_OPTION 163
RC15_OPTION 214
```

> Do **not** add the sim-sensor params (`GPS_TYPE 100`, `AHRS_EKF_TYPE 10`, `SERIAL3_*`).
> Desktop SITL already simulates its own sensors and runs the sim GPS on SERIAL3.

### 2.4 Run it

```bash
cd <repo root>
Tools/autotest/sim_vehicle.py -v ArduCopter \
  -A "--serial4=uart:/dev/ttyUSB0:115200" \
  --add-param-file=gimbal-sitl.parm \
  --console --map
```

- `--add-param-file=` applies the params right after defaults load.
- Add `-w` for a clean param/EEPROM wipe, `-N` to skip the rebuild if already built.

To load params into an **already-running** SITL instead, use the MAVProxy console:

```
param load gimbal-sitl.parm
reboot
```

> `MNT1_TYPE` only instantiates the mount backend at boot, so a `reboot` is required
> after changing it (MAVProxy reconnects automatically).

### 2.5 Driving the gimbal in SITL

`MNT1_DEFLT_MODE 3` (RC_TARGETING) means the gimbal moves from RC input. **Commanded
control and RC control are separate code paths** — `MAV_CMD_DO_MOUNT_CONTROL` goes
straight to the mount target, while RC control reads `get_radio_in()` of the channel
mapped to `MOUNT1_YAW` and only acts when that raw PWM is `> 0`
(`AP_Mount_Backend::get_rc_input`). So commanded control can work while RC does
nothing — that just means no RC is being injected.

With no physical transmitter in SITL, override the channels from MAVProxy
(**ch15 = yaw input**, ch9 = lock/follow). Values must be **off-center** — 1500 is the
trim/deadzone and produces no movement:

```
rc 15 1900      # MOUNT1_YAW input -- gimbal yaw
rc 9  1900      # MOUNT_LOCK -- follow vs lock (frame only)
```

Verify the override actually reaches the FC (the key diagnostic if RC seems dead):

```
status RC_CHANNELS      # chan15_raw should now read 1900, not 0 / 65535
```

With `MNT1_RC_RATE 90` (rate mode, as on the production board) ch15 controls yaw
*rate*: `1900` slews continuously, `1500` holds. With `MNT1_RC_RATE 0` (angle mode) ch15
maps stick *position* to absolute yaw angle, so `1500` snaps back to center.

GCS/mission `MAV_CMD_DO_MOUNT_CONTROL` works regardless of RC mapping, e.g.:

```
mount 1 pitch -45 0 0
```

### 2.6 Verifying

- SITL printed `Opened /dev/...` (not `Failed to open`) at startup.
- A `MAV_TYPE_GIMBAL` and a `MAV_TYPE_CAMERA` component appear on the link.
- The gimbal responds to a commanded angle and to ch15 yaw override.
- `CAMERA_INFORMATION` returns vendor/model instead of `UNSUPPORTED`; camera trigger
  (e.g. `module load camera` / GCS shutter) takes a picture.
