#!/usr/bin/env python3
"""
h100_vfd.py - LinuxCNC userspace HAL component for H100-series VFDs over
RS485 Modbus RTU.

Why not mb2hal: mb2hal rewrites every coil on every cycle, including 0x0000
to the run-forward / run-reverse coils. The H100 run/stop commands are
"write FF00 to coil 0x49 / 0x4A / 0x4B", so this component sends each
command only when the requested state changes (plus a slow refresh).

H100 Modbus map (from the FluidNC H100 driver, tested there):
  run forward   fn 05  coil 0x0049  value FF00
  run reverse   fn 05  coil 0x004A  value FF00
  stop          fn 05  coil 0x004B  value FF00
  set frequency fn 06  reg  0x0201  value = Hz * 10
  read freq     fn 04  reg  0x0000  count 2  -> [running Hz*10, set Hz*10]
  read F005     fn 03  reg  0x0005  (max frequency, Hz*10)
  read F011     fn 03  reg  0x000B  (min frequency, Hz*10)

HAL pins (component name set with --name, default "h100"):
  in  bit   forward          spindle.0.forward
  in  bit   reverse          spindle.0.reverse
  in  float speed-cmd-rpm    spindle.0.speed-out (sign ignored)
  out float speed-fb-rpm     measured spindle speed (from VFD output Hz)
  out float speed-fb-rps     same, in rev/s -> spindle.0.speed-in
  out bit   at-speed         -> spindle.0.at-speed
  out bit   running          VFD output frequency > 0
  out bit   comm-ok          last transaction succeeded
  out bit   comm-healthy     a transaction succeeded within --comm-timeout
                             (use as a run permissive; one glitch won't drop it)
  out u32   error-count      failed transactions since start
  out float freq-cmd-hz      frequency being commanded

Stand-alone comms test (LinuxCNC NOT running, never starts the spindle):
  python3 h100_vfd.py --port /dev/ttyUSB0 --test
"""

import argparse
import os
import select
import signal
import sys
import termios
import time

COIL_FWD = 0x0049
COIL_REV = 0x004A
COIL_STOP = 0x004B
REG_SET_FREQ = 0x0201
REG_RUN_FREQ = 0x0000
REG_F005 = 0x0005
REG_F011 = 0x000B

STOP, FWD, REV = 0, 1, 2
STATE_NAME = {STOP: "stop", FWD: "forward", REV: "reverse"}
STATE_COIL = {STOP: COIL_STOP, FWD: COIL_FWD, REV: COIL_REV}

BAUDS = {4800: termios.B4800, 9600: termios.B9600,
         19200: termios.B19200, 38400: termios.B38400}


class ModbusError(Exception):
    pass


# anything that can go wrong talking to the VFD (incl. USB adapter unplugged)
COMM_ERRORS = (ModbusError, OSError, termios.error)


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return bytes((crc & 0xFF, crc >> 8))


class ModbusRTU:
    """Minimal Modbus RTU master using only the standard library."""

    def __init__(self, port, baud, slave, timeout=0.25):
        self.port = port
        self.baud = baud
        self.slave = slave
        self.timeout = timeout
        self.fd = None
        # 3.5 character inter-frame gap (11 bits/char), at least 2 ms
        self.gap = max(0.002, 3.5 * 11.0 / baud)

    def open(self):
        self.close()
        fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(fd)
        speed = BAUDS[self.baud]
        attrs[0] = 0                                   # iflag
        attrs[1] = 0                                   # oflag
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # 8N1
        attrs[3] = 0                                   # lflag (raw)
        attrs[4] = speed
        attrs[5] = speed
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
        self.fd = fd

    def close(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None

    def _transact(self, pdu, resp_len):
        if self.fd is None:
            self.open()
        frame = bytes([self.slave]) + pdu
        frame += crc16(frame)
        termios.tcflush(self.fd, termios.TCIFLUSH)
        os.write(self.fd, frame)
        termios.tcdrain(self.fd)

        buf = b""
        deadline = time.monotonic() + self.timeout
        while len(buf) < resp_len:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            r, _, _ = select.select([self.fd], [], [], remaining)
            if r:
                chunk = os.read(self.fd, 256)
                if chunk:
                    buf += chunk
                    # exception response is 5 bytes
                    if len(buf) >= 5 and buf[1] & 0x80:
                        break
        time.sleep(self.gap)

        if len(buf) >= 5 and buf[1] & 0x80:
            if crc16(buf[:3]) != buf[3:5]:
                raise ModbusError("bad CRC on exception reply")
            raise ModbusError("VFD exception code %d" % buf[2])
        if len(buf) < resp_len:
            raise ModbusError("timeout (%d of %d bytes)" % (len(buf), resp_len))
        buf = buf[:resp_len]
        if crc16(buf[:-2]) != buf[-2:]:
            raise ModbusError("bad CRC")
        if buf[0] != self.slave or buf[1] != pdu[0]:
            raise ModbusError("unexpected reply %s" % buf.hex())
        return buf

    def write_coil(self, addr, on=True):
        val = 0xFF00 if on else 0x0000
        pdu = bytes([0x05, addr >> 8, addr & 0xFF, val >> 8, val & 0xFF])
        self._transact(pdu, 8)

    def write_register(self, addr, value):
        value = max(0, min(0xFFFF, int(value)))
        pdu = bytes([0x06, addr >> 8, addr & 0xFF, value >> 8, value & 0xFF])
        self._transact(pdu, 8)

    def _read(self, fn, addr, count):
        pdu = bytes([fn, addr >> 8, addr & 0xFF, count >> 8, count & 0xFF])
        buf = self._transact(pdu, 5 + 2 * count)
        if buf[2] != 2 * count:
            raise ModbusError("bad byte count %d" % buf[2])
        return [(buf[3 + 2 * i] << 8) | buf[4 + 2 * i] for i in range(count)]

    def read_input_registers(self, addr, count):
        return self._read(0x04, addr, count)

    def read_holding_registers(self, addr, count):
        return self._read(0x03, addr, count)


def parse_args():
    p = argparse.ArgumentParser(description="H100 VFD Modbus HAL component")
    p.add_argument("--name", default="h100", help="HAL component name")
    p.add_argument("--port", default="/dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=9600, choices=sorted(BAUDS))
    p.add_argument("--slave", type=int, default=1, help="VFD address (F163)")
    p.add_argument("--max-rpm", type=float, default=24000.0)
    p.add_argument("--max-hz", type=float, default=400.0)
    p.add_argument("--min-rpm", type=float, default=6000.0)
    p.add_argument("--at-speed-tol", type=float, default=0.03,
                   help="at-speed tolerance as a fraction of commanded speed")
    p.add_argument("--period", type=float, default=0.1,
                   help="seconds between polling cycles")
    p.add_argument("--refresh", type=float, default=1.0,
                   help="seconds between re-sending the current command")
    p.add_argument("--comm-timeout", type=float, default=1.0,
                   help="seconds without a good reply before comm-healthy drops")
    p.add_argument("--test", action="store_true",
                   help="read-only comms test without HAL, then exit")
    return p.parse_args()


def run_test(args, bus):
    print("Talking to VFD at %s, %d baud, address %d ..."
          % (args.port, args.baud, args.slave))
    ok = True
    for label, fn in (
        ("F005 max frequency", lambda: bus.read_holding_registers(REG_F005, 1)[0] / 10.0),
        ("F011 min frequency", lambda: bus.read_holding_registers(REG_F011, 1)[0] / 10.0),
        ("output frequency", lambda: bus.read_input_registers(REG_RUN_FREQ, 2)[0] / 10.0),
    ):
        try:
            print("  %-20s %8.1f Hz" % (label, fn()))
        except COMM_ERRORS as e:
            ok = False
            print("  %-20s FAILED: %s" % (label, e))
    print("OK - VFD is responding." if ok else
          "Problems talking to the VFD - check wiring (try swapping A/B), "
          "F163/F164/F165 and that nothing else has the port open.")
    return 0 if ok else 1


def main():
    args = parse_args()
    bus = ModbusRTU(args.port, args.baud, args.slave)

    if args.test:
        try:
            return run_test(args, bus)
        finally:
            bus.close()

    import hal  # only needed when running under LinuxCNC

    h = hal.component(args.name)
    h.newpin("forward", hal.HAL_BIT, hal.HAL_IN)
    h.newpin("reverse", hal.HAL_BIT, hal.HAL_IN)
    h.newpin("speed-cmd-rpm", hal.HAL_FLOAT, hal.HAL_IN)
    h.newpin("speed-fb-rpm", hal.HAL_FLOAT, hal.HAL_OUT)
    h.newpin("speed-fb-rps", hal.HAL_FLOAT, hal.HAL_OUT)
    h.newpin("at-speed", hal.HAL_BIT, hal.HAL_OUT)
    h.newpin("running", hal.HAL_BIT, hal.HAL_OUT)
    h.newpin("comm-ok", hal.HAL_BIT, hal.HAL_OUT)
    h.newpin("comm-healthy", hal.HAL_BIT, hal.HAL_OUT)
    h.newpin("error-count", hal.HAL_U32, hal.HAL_OUT)
    h.newpin("freq-cmd-hz", hal.HAL_FLOAT, hal.HAL_OUT)
    h.ready()

    quit_flag = [False]

    def on_signal(signum, frame):
        quit_flag[0] = True

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)

    rpm_per_hz = args.max_rpm / args.max_hz
    sent_state = None
    sent_freq = None
    last_refresh = 0.0
    errors = 0
    last_err_msg = None
    last_good = None

    def log(msg):
        sys.stderr.write("%s: %s\n" % (args.name, msg))
        sys.stderr.flush()

    try:
        while not quit_flag[0]:
            cycle_start = time.monotonic()

            rpm_cmd = abs(h["speed-cmd-rpm"])
            if h["forward"] and rpm_cmd > 0:
                want = FWD
            elif h["reverse"] and rpm_cmd > 0:
                want = REV
            else:
                want = STOP

            if want != STOP:
                rpm_cmd = max(args.min_rpm, min(args.max_rpm, rpm_cmd))
                freq_reg = int(round(rpm_cmd / rpm_per_hz * 10.0))
            else:
                rpm_cmd = 0.0
                freq_reg = sent_freq if sent_freq is not None else 0
            h["freq-cmd-hz"] = freq_reg / 10.0 if want != STOP else 0.0

            now = time.monotonic()
            refresh = (now - last_refresh) >= args.refresh

            try:
                # speed first, so the drive starts at the right speed
                if want != STOP and (freq_reg != sent_freq or refresh):
                    bus.write_register(REG_SET_FREQ, freq_reg)
                    sent_freq = freq_reg
                if want != sent_state or refresh:
                    bus.write_coil(STATE_COIL[want], True)
                    if want != sent_state:
                        log("spindle %s" % STATE_NAME[want])
                    sent_state = want
                if refresh:
                    last_refresh = now

                run_hz = bus.read_input_registers(REG_RUN_FREQ, 2)[0] / 10.0
                fb_rpm = run_hz * rpm_per_hz
                h["speed-fb-rpm"] = fb_rpm
                h["speed-fb-rps"] = fb_rpm / 60.0
                h["running"] = run_hz > 0.0
                if want == STOP:
                    h["at-speed"] = False
                else:
                    h["at-speed"] = abs(fb_rpm - rpm_cmd) <= args.at_speed_tol * rpm_cmd
                if not h["comm-ok"]:
                    log("communication OK")
                h["comm-ok"] = True
                last_good = time.monotonic()
                last_err_msg = None
            except COMM_ERRORS as e:
                errors += 1
                h["error-count"] = errors
                h["comm-ok"] = False
                h["at-speed"] = False
                # force everything to be re-sent once comms recover
                sent_state = None
                sent_freq = None
                if str(e) != last_err_msg:
                    log("communication error: %s" % e)
                    last_err_msg = str(e)
                if not isinstance(e, ModbusError):
                    bus.close()
                    time.sleep(1.0)

            healthy = (last_good is not None and
                       time.monotonic() - last_good <= args.comm_timeout)
            if h["comm-healthy"] and not healthy:
                log("no reply from VFD for %.1f s - comm-healthy dropped"
                    % args.comm_timeout)
            h["comm-healthy"] = healthy

            elapsed = time.monotonic() - cycle_start
            if elapsed < args.period:
                time.sleep(args.period - elapsed)
    finally:
        h["comm-healthy"] = False
        # always try to stop the spindle on exit
        for _ in range(3):
            try:
                bus.write_coil(COIL_STOP, True)
                log("spindle stopped on exit")
                break
            except COMM_ERRORS:
                time.sleep(0.1)
        bus.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
