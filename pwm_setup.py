#!/usr/bin/env python3
"""
Raspberry Pi 5 PWM setup

For the Raspberry Pi 5, the Pi foundation moved GPIO control away from the Broadcom2712 in favor of utilizing the new RP1 I/O
controller. As such, the previous utilization of pwm_device provided from Linux on the Raspberry Pi 4 cannot be utilized. 
As a stop gap, this script allows you to set up PWM through the terminal.

The following default parameters are used:
- frequency = 100 Hz
- duty cycle = 40%

You can edit these below, but remember to re-run the program.

Reminder to update /boot/firmware/config.txt with the following and reboot:
    dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
"""

import os
import sys
import time

# the PWM chip name
PWM_CHIP = "/sys/class/pwm/pwmchip0"

# === EDIT AS DESIRED ===
PERIOD_NS = 10_000_000  # 100Hz
DUTY_NS = 4_000_000     # 40% duty cycle
# =======================

CHANNELS = (0, 1)       # PWM0 and PWM1

# write to file
def write_file(path, value):
    with open(path, "w") as f:
        f.write(str(value))

# helper to wait for Linux to create the PWM directory
def wait_for_path(path, timeout=1.0):
    start = time.monotonic()

    while time.monotonic() - start < timeout:
        if os.path.exists(path):
            return True

        time.sleep(0.01)

    return False

# export the PWM channels for userspace access
def export_channel(channel):
    pwm_dir = f"{PWM_CHIP}/pwm{channel}"

    if os.path.isdir(pwm_dir):
        return
    
    try:
        write_file(f"{PWM_CHIP}/export", channel)
    except OSError as exc:
        # EBUSY usually means the PWM is already exported
        if exc.errno != 16:
            raise

    if not wait_for_path(pwm_dir):
        raise RuntimeError(
            f"Timed out waiting for {pwm_dir}"
        )


def configure_channel(channel):
    pwm_dir = f"{PWM_CHIP}/pwm{channel}"

    # export channel for writing
    export_channel(channel)

    # disable PWM before changing period/duty
    write_file(f"{pwm_dir}/enable", 0)

    # configure with user specified period
    write_file(f"{pwm_dir}/period", PERIOD_NS)

    # configure with user specified duty cycle
    write_file(f"{pwm_dir}/duty_cycle", DUTY_NS)

    # enable PWM
    write_file(f"{pwm_dir}/enable", 1)


def main():
    # make sure to run the program as root
    if os.geteuid() != 0:
        print(
            "Run this program as root:\n"
            "    sudo python3 pwm_setup.py",
            file=sys.stderr,
        )
        return 1

    # look for the PWM Chip and exit if not loaded
    if not os.path.isdir(PWM_CHIP):
        print(
            f"{PWM_CHIP} does not exist.\n\n"
            "Make sure /boot/firmware/config.txt contains:\n"
            "    dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4\n"
            "and reboot before running this program.",
            file=sys.stderr,
        )
        return 1

    # config step, configure PWM 0, 1
    try:
        for channel in CHANNELS:
            configure_channel(channel)

    except (OSError, RuntimeError) as exc:
        print(f"PWM setup failed: {exc}", file=sys.stderr)
        return 1

    print("PWM setup complete:")
    print("  PWM0 / GPIO12: 100 Hz, 40% duty cycle")
    print("  PWM1 / GPIO13: 100 Hz, 40% duty cycle")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())