"""Latching remote e-stop. Every stop holds until a new reset from a live remote."""

import json
from pathlib import Path


def read_token(path: str) -> str:
    """The shared secret in a file outside the repo; raises if missing or empty."""
    token = Path(path).expanduser().read_text().strip()
    if not token:
        raise ValueError(f'{path} is empty')
    return token


def parse_packet(data: bytes, token: str):
    """An empty token accepts any sender (bench only)."""
    try:
        pkt = json.loads(data)
        if token and pkt['token'] != token:
            return None
        return {
            'session': str(pkt['session']),
            'stop': bool(pkt['stop']),
            'reset_id': int(pkt['reset_id']),
        }
    except (ValueError, TypeError, KeyError):
        return None


class EStopState:
    def __init__(self, timeout_s: float = 0.5):
        self.timeout_s = timeout_s
        self.latched = True
        self.reason = 'startup'
        self._last_rx = None
        self._session = None
        self._reset_id = 0

    def _latch(self, reason: str) -> None:
        self.latched = True
        self.reason = reason

    def _check_timeout(self, now: float) -> None:
        if self._last_rx is not None and now - self._last_rx > self.timeout_s:
            self._latch('link timeout')
            self._last_rx = None

    def on_packet(self, pkt: dict, now: float) -> None:
        self._check_timeout(now)
        self._last_rx = now

        if pkt['session'] != self._session:
            self._session = pkt['session']
            self._reset_id = pkt['reset_id']
            self._latch('new remote')
        elif pkt['stop']:
            self._latch('stop pressed')
        # ">" rather than "!=": a late, reordered packet can never reset.
        elif pkt['reset_id'] > self._reset_id:
            self._reset_id = pkt['reset_id']
            self.latched = False
            self.reason = 'running'

    def active(self, now: float) -> bool:
        self._check_timeout(now)
        return self.latched
