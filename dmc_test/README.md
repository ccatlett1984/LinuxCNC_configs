# DMC Mini 2 — Spindle Control (H100 VFD over RS485)

LinuxCNC 2.9.10 on a Raspberry Pi 5 with a Mesa 7C80. The spindle VFD is an
**H100-2S2-1B**. It is controlled over **RS485 Modbus RTU**, and the 7C80's isolated
spindle-enable output provides a **hardwired run permissive**.

Spindle: 24,000 RPM at 400 Hz, minimum safe speed 6,000 RPM (100 Hz).

---

## 1. Physical wiring

### 1a. RS485 (speed / run / direction commands)

Adapter: Waveshare USB-RS485 (FTDI FT232R), plugged into the Pi.

| USB-RS485 adapter | H100 control terminal |
|---|---|
| A / 485+ | RS+ |
| B / 485− | RS− |
| GND | GND / COM |

- A and B run on one twisted pair. If there is no communication, swap A and B first.
- The cable shield is grounded at the VFD end only.
- Keep this cable away from the VFD-to-spindle motor leads.

### 1b. Hardwired run permissive / E-stop (7C80 → VFD X3)

The 7C80 SPINDLE EN output is an isolated opto-coupled switch rated 50 mA at 0–100 VDC.
It does **not** supply power. The VFD supplies the input current, and the 7C80 opens or
closes the path.

| 7C80 TB5 pin | Signal | Wire color | H100 terminal |
|---|---|---|---|
| 6 | SPINDLE EN+ | **Green** | **X3** |
| 5 | SPINDLE EN− | **White** | **COM** |

- The VFD inputs are in **NPN** mode (jumper J1 at the factory setting, left unchanged).
- The DIR pair (TB5 pins 7/8) is **not connected**.
- Voltage across X3–COM, measured:

| Condition | X3–COM | Meaning |
|---|---|---|
| LinuxCNC closed | 10.65 V | output open, VFD will not run |
| LinuxCNC open, machine **off** | 10.65 V | output open, VFD will not run |
| LinuxCNC open, machine **on** | 0.5 V | output closed, OK to run |

**Do not power the VFD's terminal wiring on until the VFD parameters in section 2 are set.**
In particular, F067 = 1 inverts every input, so any unused input left with a function
would become active.

---

## 2. H100 VFD parameter changes

Parameter numbering follows the H100 Series Operation Manual.

| Parameter | Function | Set to | Notes |
|---|---|---|---|
| F001 | Control mode (run command source) | **2** | communication interface |
| F002 | Frequency source | **2** | communication (register 0201h) |
| F005 | Max / reference frequency | **400 Hz** | read back over Modbus |
| F011 | Lower frequency limit | **100 Hz** | 6,000 RPM floor, also applies to the keypad |
| F044 | X1 function | **0** | invalid (was 2, forward) |
| F045 | X2 function | **0** | invalid (was 3, reverse) |
| F046 | X3 function | **13** | emergency cut-off (was 14, reset) |
| F047 | X4 function | **0** | invalid (was 22, high speed) |
| F048 | X5 function | **0** | invalid (was 23, medium speed) |
| F049 | X6 function | **0** | invalid (was 24, low speed) |
| F067 | Digital input logic | **1** | inverted, so **open X3 = emergency stop** |
| F163 | Communication address | **1** | |
| F164 | Baud rate | **1** | 9600 |
| F165 | Data format | **3** | 8N1 RTU |

Order used when setting the input parameters: set F044, F045, F047, F048 and F049 to 0
**before** setting F046 = 13 and F067 = 1.

F022 sets the emergency-stop deceleration time used when X3 opens.

---

## 3. LinuxCNC configuration (`configs/dmc_test`)

| File | What it does |
|---|---|
| `h100_vfd.py` | Userspace HAL component that talks Modbus RTU to the H100 (standard library only) |
| `test.hal` | Loads `h100_vfd.py` and wires the spindle nets and the 7C80 permissive output. The old 7C80 analog/PWM spindle section is kept, commented out. |
| `test.ini` | `[H100_VFD]` holds the port, baud, address and RPM/Hz settings. `[SPINDLE_0]` clamps commanded speed to 6,000–24,000 RPM. |
| `../../nc_files/atspeed_test.ngc` | Test program: M3 S12000, feed move (waits for at-speed), M5 |

Serial port (stable name, survives unplug and replug):
```
/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BH001YJZ-if00-port0
```
If the adapter is ever replaced, update `PORT` in `[H100_VFD]` using `ls -l /dev/serial/by-id/`.

### Why a custom component instead of mb2hal
mb2hal rewrites every coil on every cycle, including writing 0x0000 to the run-forward and
run-reverse coils. The H100 only defines FF00 writes for these coils, so `h100_vfd.py`
sends each command once, when it changes, plus a 1-second refresh.

### H100 Modbus map used
| Action | Function | Address | Value |
|---|---|---|---|
| Run forward | 05 write coil | 0x0049 | FF00 |
| Run reverse | 05 write coil | 0x004A | FF00 |
| Stop | 05 write coil | 0x004B | FF00 |
| Set frequency | 06 write register | 0x0201 | Hz × 10 |
| Read output frequency | 04 read input registers | 0x0000 (2 regs) | Hz × 10 |
| Read F005 / F011 | 03 read holding | 0x0005 / 0x000B | Hz × 10 |

### Hardwired permissive logic (`test.hal`)
```
spindle-permit = machine-is-enabled AND h100.comm-healthy
spindle-permit -> hm2_7c80.0.gpio.020.out and gpio.021.out (invert_output true)
```
`h100.comm-healthy` drops if the VFD has not answered for 1 second. Both GPIO 20 and 21 are
driven because the 7C80 docs don't say which one is the EN pair (DIR isn't wired).

### Main HAL pins (`h100.*`)
`forward`, `reverse`, `speed-cmd-rpm` (in) ·
`speed-fb-rpm`, `speed-fb-rps`, `at-speed`, `running`, `comm-ok`, `comm-healthy`,
`error-count`, `freq-cmd-hz` (out)

---

## 4. What stops the spindle (all tested 2026-10-25)

| Failure | How it stops | Tested |
|---|---|---|
| M5 / program end | Modbus stop | ✔ |
| E-stop / machine off | Modbus stop + permissive opens X3 | ✔ |
| LinuxCNC / Pi realtime freeze | 7C80 watchdog opens X3 (`halcmd stop`) | ✔ |
| USB adapter / RS485 link lost | comm-healthy drops after 1 s → X3 opens | ✔ |
| LinuxCNC exits | driver sends stop + X3 opens | ✔ |

After the comm link recovers, the spindle stays stopped until it is commanded again (M3/M4).

Also verified: M3 runs clockwise; commanded 12,000 RPM with feedback reading 12,000 RPM;
feed moves wait for at-speed before starting.

---

## 5. Troubleshooting

**Comms test** (LinuxCNC closed; read-only, never starts the spindle):
```
python3 ~/linuxcnc/configs/dmc_test/h100_vfd.py --port /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BH001YJZ-if00-port0 --test
```
Expected: F005 400.0 Hz, F011 100.0 Hz.

- **Timeouts:** swap A and B, check F163/F164/F165, and check that the user is in the
  `dialout` group (`groups`).
- **Spindle won't run but comms are fine:** check that X3–COM reads about 0.5 V with the
  machine on. If it reads 10.65 V, the permissive is open. Check `halcmd show pin and2.0`
  and `halcmd show pin h100.comm-healthy`.
- **Driver messages:** start LinuxCNC from a terminal to see `h100:` log lines.
