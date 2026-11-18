import json

import pytest

from rc_safety.estop_state import EStopState, parse_packet, read_token


def pkt(stop=False, reset_id=0, session='a'):
    return {'session': session, 'stop': stop, 'reset_id': reset_id}


def running():
    s = EStopState(timeout_s=0.5)
    s.on_packet(pkt(), 0.0)
    s.on_packet(pkt(reset_id=1), 0.1)
    assert not s.active(0.1)
    return s


def test_starts_latched():
    assert EStopState().active(0.0)


def test_first_packet_does_not_arm():
    s = EStopState()
    s.on_packet(pkt(reset_id=7), 0.0)
    assert s.active(0.0)


def test_stop_latches_until_next_reset():
    s = running()
    s.on_packet(pkt(stop=True, reset_id=1), 0.2)
    s.on_packet(pkt(reset_id=1), 0.3)
    assert s.active(0.3)
    s.on_packet(pkt(reset_id=2), 0.4)
    assert not s.active(0.4)


def test_timeout_latches_and_recovered_link_stays_stopped():
    s = running()
    assert s.active(0.7)
    s.on_packet(pkt(reset_id=1), 1.0)
    assert s.active(1.0)


def test_packet_after_gap_latches_without_active_call():
    s = running()
    s.on_packet(pkt(reset_id=1), 5.0)
    assert s.active(5.0)


def test_late_reordered_packet_cannot_reset():
    s = running()
    s.on_packet(pkt(reset_id=3), 0.2)
    s.on_packet(pkt(stop=True, reset_id=3), 0.3)
    s.on_packet(pkt(reset_id=2), 0.4)
    assert s.active(0.4)


def test_restarted_remote_starts_stopped():
    s = running()
    s.on_packet(pkt(session='b'), 0.2)
    assert s.active(0.2)
    s.on_packet(pkt(reset_id=1, session='b'), 0.3)
    assert not s.active(0.3)


def raw(**kw):
    return json.dumps({'token': 't', 'session': 'a', 'stop': False, 'reset_id': 0, **kw}).encode()


def test_parse():
    assert parse_packet(raw(), 't') == pkt()
    assert parse_packet(raw(token='x'), 't') is None
    assert parse_packet(b'\xff', 't') is None
    assert parse_packet(b'[1]', 't') is None
    assert parse_packet(b'{"token": "t"}', 't') is None


def test_empty_token_accepts_any_sender():
    assert parse_packet(raw(token='x'), '') == pkt()


def test_read_token(tmp_path):
    f = tmp_path / 'estop_token'
    f.write_text('s3cret\n')
    assert read_token(str(f)) == 's3cret'
    f.write_text('\n')
    with pytest.raises(ValueError):
        read_token(str(f))
    with pytest.raises(OSError):
        read_token(str(tmp_path / 'missing'))
