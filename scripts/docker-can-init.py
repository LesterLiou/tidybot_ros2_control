#!/usr/bin/env python3
"""Initialize a host CANivore through Phoenix without creating a motor device."""

import os
from pathlib import Path
import subprocess
import sys
import time

from phoenix6 import CANBus


def main() -> int:
    interface = os.environ.get('TIDYBOT_CAN_INTERFACE', 'can0')
    net_path = Path('/sys/class/net') / interface
    module_link = net_path / 'device/driver/module'

    if not net_path.exists():
        print(f'ERROR: {interface} does not exist; check the host canivore-usb driver.',
              file=sys.stderr)
        return 1

    driver = Path(os.path.realpath(module_link)).name if module_link.exists() else 'unknown'
    if driver != 'canivore_usb':
        print(
            f'ERROR: {interface} uses {driver}, not canivore_usb. Configure this adapter '
            'on the host according to its documentation.',
            file=sys.stderr,
        )
        return 1

    for helper in ('/usr/bin/canivore_setup', '/usr/bin/canivore_shutdown'):
        if not Path(helper).is_file():
            print(f'ERROR: required host helper is not mounted: {helper}', file=sys.stderr)
            return 1

    print(f'Initializing CTRE CANivore interface {interface} with Phoenix...')
    bus = CANBus(interface)
    status = bus.get_status()

    for _ in range(30):
        state_file = net_path / 'operstate'
        state = state_file.read_text(encoding='utf-8').strip()
        if state in ('up', 'unknown'):
            subprocess.run(['ip', '-details', 'link', 'show', interface], check=False)
            print(
                'CANivore network initialized. No motor object, control request, or '
                'actuator enable was created.'
            )
            return 0
        time.sleep(0.1)
        status = bus.get_status()

    subprocess.run(['ip', '-details', 'link', 'show', interface], check=False)
    print(
        f'ERROR: Phoenix could not initialize {interface}; status={status.status}.',
        file=sys.stderr,
    )
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
