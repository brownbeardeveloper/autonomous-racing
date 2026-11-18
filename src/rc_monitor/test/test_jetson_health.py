from rc_monitor import jetson_health as jh

# Captured on this Jetson (JetPack 6, R36.5).
IW_DEV = """phy#0
\tInterface wlP1p1s0
\t\tifindex 2
\t\taddr f8:3d:c6:1f:a1:9b
\t\ttype managed
\t\ttxpower 13.00 dBm
"""
IW_LINK = """Connected to 08:bf:b8:0e:18:74 (on wlP1p1s0)
\tsignal: -52 dBm
\trx bitrate: 780.0 MBit/s VHT-MCS 9 80MHz VHT-NSS 2
"""
IW_STATIONS = """Station aa:bb:cc:dd:ee:01 (on wlP1p1s0)
\tsignal:  \t-48 [-50, -51] dBm
\tsignal avg:\t-47 [-49, -50] dBm
Station aa:bb:cc:dd:ee:02 (on wlP1p1s0)
\tsignal:  \t-74 [-75, -76] dBm
\tsignal avg:\t-73 dBm
"""


def write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text + '\n')


def test_level_rises_with_value():
    assert jh.level(84.9, 85, 95) == jh.OK
    assert jh.level(85, 85, 95) == jh.WARN
    assert jh.level(95, 85, 95) == jh.ERROR


def test_signal_level_falls_with_dbm():
    assert jh.signal_level(-52, -70, -80) == jh.OK
    assert jh.signal_level(-70, -70, -80) == jh.WARN
    assert jh.signal_level(-85, -70, -80) == jh.ERROR


def test_board_model_drops_trailing_nul(tmp_path):
    (tmp_path / 'model').write_bytes(b'NVIDIA Jetson Orin Nano\x00')
    assert jh.board_model(tmp_path / 'model') == 'NVIDIA Jetson Orin Nano'


def test_gpu_load_is_per_mille(tmp_path):
    write(tmp_path / 'load', '873')
    write(tmp_path / 'devfreq' / '17000000.gpu' / 'cur_freq', '918000000')
    assert jh.gpu_load_pct(tmp_path) == 87.3
    assert jh.gpu_freq_mhz(tmp_path) == 918


def test_missing_gpu_reads_none(tmp_path):
    assert jh.gpu_load_pct(tmp_path) is None
    assert jh.gpu_freq_mhz(tmp_path) is None


def test_fan_pwm_to_percent(tmp_path):
    write(tmp_path / 'hwmon3' / 'pwm1', '255')
    assert jh.fan_pct(f'{tmp_path}/hwmon*') == 100


def ina3221(root, alarm='0'):
    hwmon = root / '1-0040' / 'hwmon' / 'hwmon1'
    for i, (label, mv, ma) in enumerate(
        [('VDD_IN', '4968', '1488'), ('VDD_CPU_GPU_CV', '4960', '472')], start=1
    ):
        write(hwmon / f'in{i}_label', label)
        write(hwmon / f'in{i}_input', mv)
        write(hwmon / f'curr{i}_input', ma)
        write(hwmon / f'curr{i}_crit_alarm', alarm if label == 'VDD_IN' else '0')
        write(hwmon / f'curr{i}_max_alarm', '0')
    write(hwmon / 'in7_label', 'sum of shunt voltages')
    write(hwmon / 'in7_input', '120')
    return f'{root}/*/hwmon/hwmon*'


def test_power_rails_in_watts(tmp_path):
    rails = jh.power_rails(ina3221(tmp_path))
    assert set(rails) == {'VDD_IN', 'VDD_CPU_GPU_CV'}  # the shunt sum has no current
    watts, over_current = rails['VDD_IN']
    assert abs(watts - 7.392) < 1e-3
    assert not over_current


def test_power_rail_over_current_alarm(tmp_path):
    assert jh.power_rails(ina3221(tmp_path, alarm='1'))['VDD_IN'][1]


def test_iw_dev_interface_types():
    assert jh.parse_iw_dev(IW_DEV) == {'wlP1p1s0': 'managed'}
    assert jh.parse_iw_dev(IW_DEV.replace('managed', 'AP')) == {'wlP1p1s0': 'AP'}
    assert jh.parse_iw_dev('') == {}


def test_signals_skip_averages():
    assert jh.parse_signals(IW_LINK) == [-52]
    assert jh.parse_signals(IW_STATIONS) == [-48, -74]
    assert jh.parse_signals('Not connected.\n') == []
