"""Keep the managed container available after the motor launch shuts down."""
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    status = Path('/tmp/tidybot-launch-status')
    status.write_text('STARTING\n')
    stopping = False

    def stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    with subprocess.Popen(sys.argv[1:]) as launch:
        status.write_text('LAUNCH_RUNNING\n')
        while not stopping:
            try:
                code = launch.wait(timeout=0.25)
            except subprocess.TimeoutExpired:
                continue
            message = (f'FAULT: ROS launch exited ({code}). See node logs. '
                       'After resolving the fault, restart tidybot explicitly.')
            status.write_text(message + '\n')
            print(message, flush=True)
            while not stopping:
                time.sleep(0.25)
            break
        if launch.poll() is None:
            launch.send_signal(signal.SIGINT)


if __name__ == '__main__':
    main()
