#!/usr/bin/env python3
"""Remote e-stop, run on the laptop: space = STOP, r = reset, q = quit."""

import argparse
import json
import pathlib
import select
import socket
import sys
import termios
import time
import tty
import uuid

PERIOD_S = 0.1
LINK_TIMEOUT_S = 0.5


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--host', default='10.42.0.1', help='the car; default: its own hotspot')
    ap.add_argument('--port', type=int, default=5005)
    # A file, not an argument: keeps the secret out of shell history and ps.
    ap.add_argument('--token-file', default='~/.config/rc_car/estop_token')
    args = ap.parse_args()
    try:
        token = pathlib.Path(args.token_file).expanduser().read_text().strip()
    except OSError as e:
        sys.exit(f'no e-stop token: {e} - copy it from the car, see README, Remote e-stop')

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    session = uuid.uuid4().hex
    stop, reset_id = True, 0
    car, last_reply = {}, 0.0

    def send():
        pkt = {'token': token, 'session': session, 'stop': stop, 'reset_id': reset_id}
        try:
            sock.sendto(json.dumps(pkt).encode(), (args.host, args.port))
        except OSError:
            pass  # no route; the car times out on its own

    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    print(__doc__)
    try:
        while True:
            if select.select([sys.stdin], [], [], PERIOD_S)[0]:
                key = sys.stdin.read(1)
                if key == 'q':
                    break
                if key == ' ':
                    stop = True
                elif key == 'r':
                    stop, reset_id = False, reset_id + 1
            send()

            try:
                while True:
                    car = json.loads(sock.recv(1024))
                    last_reply = time.monotonic()
            except (OSError, ValueError):
                pass

            if time.monotonic() - last_reply > LINK_TIMEOUT_S:
                status = 'LINK LOST | car stopping on its own (or the token differs)'
            elif car['stopped']:
                status = f"LINK OK   | STOPPED ({car['reason']})"
            else:
                status = 'LINK OK   | RUNNING'
            print(f'\r\033[K{status}', end='', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        stop = True
        for _ in range(5):
            send()
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        print()


if __name__ == '__main__':
    main()
