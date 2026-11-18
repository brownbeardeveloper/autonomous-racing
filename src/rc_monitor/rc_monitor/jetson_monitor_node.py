#!/usr/bin/env python3
"""Jetson health on /diagnostics, once a second.

CPU, GPU, memory, temperature, power, disk, WiFi and the sensor topic rates.
It only reports: nothing here brakes or drives.
"""

import subprocess

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import FrequencyStatusParam, HeaderlessTopicDiagnostic, Updater
from nav_msgs.msg import Odometry
import psutil
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from rc_monitor import jetson_health as jh

LEVELS = (DiagnosticStatus.OK, DiagnosticStatus.WARN, DiagnosticStatus.ERROR)
STALE = DiagnosticStatus.STALE


def run(*cmd):
    """A command's stdout, or '' if it is missing, fails or hangs."""
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=0.5).stdout
    except (OSError, subprocess.SubprocessError):
        return ''


class JetsonMonitor(Node):
    def __init__(self):
        super().__init__('jetson_monitor_node')
        self.declare_parameters('', [
            ('cpu.warn', 85.0), ('cpu.error', 95.0),
            ('gpu.warn', 85.0), ('gpu.error', 95.0),
            ('memory.warn', 85.0), ('memory.error', 95.0),
            ('temperature.warn', 85.0), ('temperature.error', 95.0),
            ('disk.warn', 90.0), ('disk.error', 97.0),
            ('wifi.warn', -70.0), ('wifi.error', -80.0),
            ('scan_topic', 'scan'), ('scan_min_hz', 36.0),
            ('odom_topic', 'odom'), ('odom_min_hz', 18.0),
        ])

        self.updater = Updater(self)  # period: the diagnostic_updater.period parameter, 1 s
        self.updater.setHardwareID(jh.board_model() or 'jetson')
        self.updater.add('CPU', self.cpu)
        self.updater.add('GPU', self.gpu)
        self.updater.add('Memory', self.memory)
        self.updater.add('Temperature', self.temperature)
        self.updater.add('Power', self.power)
        self.updater.add('Disk', self.disk)
        self.updater.add('WiFi', self.wifi)

        for name, msg_type in (('scan', LaserScan), ('odom', Odometry)):
            topic = self.resolve_topic_name(self.param(f'{name}_topic'))
            rate = FrequencyStatusParam({'min': self.param(f'{name}_min_hz')}, tolerance=0.0)
            diag = HeaderlessTopicDiagnostic(topic, self.updater, rate)
            self.create_subscription(
                msg_type, topic, lambda _, diag=diag: diag.tick(), qos_profile_sensor_data
            )

        self.power_mode = run('nvpmodel', '-q').partition('\n')[0].removeprefix('NV Power Mode: ')
        psutil.cpu_percent(percpu=True)  # the first call only starts the measurement

    def param(self, name):
        return self.get_parameter(name).value

    def grade(self, value, check):
        return LEVELS[jh.level(value, self.param(f'{check}.warn'), self.param(f'{check}.error'))]

    def cpu(self, stat):
        cores = psutil.cpu_percent(percpu=True)
        mean = sum(cores) / len(cores)
        stat.summary(self.grade(mean, 'cpu'), f'{mean:.0f}% (busiest core {max(cores):.0f}%)')
        stat.add('Load average 1 min', f'{psutil.getloadavg()[0]:.2f}')
        stat.add('Frequency (MHz)', f'{psutil.cpu_freq().current:.0f}')
        for i, pct in enumerate(cores):
            stat.add(f'Core {i} (%)', f'{pct:.0f}')
        return stat

    def gpu(self, stat):
        load = jh.gpu_load_pct()
        if load is None:
            stat.summary(STALE, f'no GPU load at {jh.GPU_DIR}')
            return stat
        stat.summary(self.grade(load, 'gpu'), f'{load:.0f}%')
        freq = jh.gpu_freq_mhz()
        if freq is not None:
            stat.add('Frequency (MHz)', f'{freq:.0f}')
        return stat

    def memory(self, stat):
        ram, swap = psutil.virtual_memory(), psutil.swap_memory()
        # RAM is shared with the GPU, so CUDA allocations show up here too.
        stat.summary(
            self.grade(ram.percent, 'memory'),
            f'{ram.percent:.0f}% ({ram.available / 1e9:.1f} GB free)',
        )
        stat.add('Swap used (%)', f'{swap.percent:.0f}')
        return stat

    def temperature(self, stat):
        zones = {
            name: sensors[0].current for name, sensors in psutil.sensors_temperatures().items()
        }
        if not zones:
            stat.summary(STALE, 'no thermal zones')
            return stat
        hottest = max(zones, key=zones.get)
        stat.summary(
            self.grade(zones[hottest], 'temperature'), f'{zones[hottest]:.0f} C ({hottest})'
        )
        for name, celsius in zones.items():
            stat.add(f'{name} (C)', f'{celsius:.1f}')
        fan = jh.fan_pct()
        if fan is not None:
            stat.add('Fan (%)', f'{fan:.0f}')
        return stat

    def power(self, stat):
        rails = jh.power_rails()
        if not rails:
            stat.summary(STALE, 'no INA3221 power monitor')
            return stat
        board = f'{rails["VDD_IN"][0]:.1f} W' if 'VDD_IN' in rails else 'see rails'
        tripped = [rail for rail, (_, alarm) in rails.items() if alarm]
        if tripped:
            stat.summary(
                DiagnosticStatus.WARN,
                f'{board}, over-current on {", ".join(tripped)}: clocks throttled',
            )
        else:
            stat.summary(DiagnosticStatus.OK, board)
        for rail, (watts, _) in rails.items():
            stat.add(f'{rail} (W)', f'{watts:.2f}')
        stat.add('Power mode', self.power_mode or 'unknown')
        return stat

    def disk(self, stat):
        usage = psutil.disk_usage('/')
        stat.summary(
            self.grade(usage.percent, 'disk'),
            f'{usage.percent:.0f}% ({usage.free / 1e9:.0f} GB free)',
        )
        return stat

    def wifi(self, stat):
        interfaces = jh.parse_iw_dev(run('iw', 'dev'))
        if not interfaces:
            stat.summary(DiagnosticStatus.WARN, 'no WiFi interface')
            return stat
        iface, kind = next(iter(interfaces.items()))
        stat.add('Interface', iface)
        if kind == 'AP':
            signals = jh.parse_signals(run('iw', 'dev', iface, 'station', 'dump'))
            if not signals:
                stat.summary(DiagnosticStatus.WARN, 'hotspot up, no clients')
                return stat
            dbm, what = min(signals), f'hotspot, weakest of {len(signals)} clients'
        else:
            signals = jh.parse_signals(run('iw', 'dev', iface, 'link'))
            if not signals:
                stat.summary(DiagnosticStatus.WARN, 'not connected')
                return stat
            dbm, what = signals[0], 'link'
        level = jh.signal_level(dbm, self.param('wifi.warn'), self.param('wifi.error'))
        stat.summary(LEVELS[level], f'{dbm} dBm ({what})')
        return stat


def main(args=None):
    rclpy.init(args=args)
    node = JetsonMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
