"""Jetson readings the kernel exposes outside psutil, and how bad a reading is.

Every path is an argument so the tests can point them at a fake sysfs tree.
"""

from glob import glob
from pathlib import Path
import re

OK, WARN, ERROR = 0, 1, 2

GPU_DIR = '/sys/devices/platform/17000000.gpu'
INA3221_GLOB = '/sys/bus/i2c/drivers/ina3221/*/hwmon/hwmon*'
FAN_GLOB = '/sys/devices/platform/pwm-fan/hwmon/hwmon*'


def level(value, warn, error):
    """Severity of a reading that gets worse as it rises: load, temperature, fill."""
    return ERROR if value >= error else WARN if value >= warn else OK


def signal_level(dbm, warn, error):
    """Severity of a radio signal, which gets worse as it falls."""
    return ERROR if dbm <= error else WARN if dbm <= warn else OK


def _read(path):
    try:
        return Path(path).read_text().strip()
    except OSError:
        return None


def _first(pattern):
    paths = sorted(glob(pattern))
    return _read(paths[0]) if paths else None


def board_model(path='/proc/device-tree/model'):
    raw = _read(path)
    return None if raw is None else raw.rstrip('\x00')  # device-tree strings end in NUL


def gpu_load_pct(gpu_dir=GPU_DIR):
    raw = _read(f'{gpu_dir}/load')
    return None if raw is None else int(raw) / 10  # per mille


def gpu_freq_mhz(gpu_dir=GPU_DIR):
    raw = _first(f'{gpu_dir}/devfreq/*/cur_freq')
    return None if raw is None else int(raw) / 1e6


def fan_pct(fan_glob=FAN_GLOB):
    raw = _first(f'{fan_glob}/pwm1')
    return None if raw is None else int(raw) * 100 / 255


def power_rails(ina3221_glob=INA3221_GLOB):
    """{rail: (watts, over_current)} from the INA3221 power monitor.

    over_current is the chip's own alarm. When it trips on VDD_IN the board
    throttles its clocks, which on a battery usually means the supply is sagging.
    """
    rails = {}
    for hwmon in sorted(glob(ina3221_glob)):
        for label_file in sorted(glob(f'{hwmon}/in*_label')):
            i = re.search(r'in(\d+)_label$', label_file).group(1)
            mv, ma = _read(f'{hwmon}/in{i}_input'), _read(f'{hwmon}/curr{i}_input')
            if mv is None or ma is None:
                continue  # "sum of shunt voltages" has no current of its own
            alarm = any(_read(f'{hwmon}/curr{i}_{kind}_alarm') == '1' for kind in ('crit', 'max'))
            rails[_read(label_file)] = (int(mv) * int(ma) / 1e6, alarm)
    return rails


def parse_iw_dev(text):
    """{interface: type} from `iw dev`. The hotspot's interface has type AP."""
    return dict(re.findall(r'Interface (\S+)[\s\S]*?\btype (\S+)', text))


def parse_signals(text):
    """Every `signal: <n> dBm` in `iw dev <if> link` or `iw dev <if> station dump`."""
    return [int(s) for s in re.findall(r'^\s*signal:\s*(-?\d+)', text, re.MULTILINE)]
