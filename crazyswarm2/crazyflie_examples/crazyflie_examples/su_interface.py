import math
import os
from pathlib import Path
import tempfile
import rclpy
from rclpy.node import Node
from crazyflie_interfaces.msg import LogDataGeneric
from std_msgs.msg import Float64MultiArray, String

from ament_index_python.packages import get_package_share_directory
from crazyflie_py import Crazyswarm


CALIB_IDLE = 0
CALIB_IMU_TRIM = 1
CALIB_COM_COLLECT = 2
CALIB_DONE = 3
CALIB_ERROR = 4
CALIBRATION_TIMEOUT_SEC = 70.0
DISARM_RETRY_PERIOD_SEC = 0.1
DISARM_RETRY_COUNT = 20


class SuInterface(Node):
    def __init__(self):
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.cf = self.swarm.allcfs.crazyflies[0]

        super().__init__('su_interface')

        self.subscription = self.create_subscription(
            String,
            'keyboard_input',
            self.keyboard_callback,
            10
        )
        self.debug_subscription = self.create_subscription(
            LogDataGeneric,
            '/cf2/cf_calibration',
            self.calibration_status_callback,
            10,
        )
        self.calibration_publisher = self.create_publisher(
            Float64MultiArray,
            'cf2/hover_calibration',
            10,
        )
        self.calibration_result_publisher = self.create_publisher(
            String,
            'calibration_status',
            10,
        )
        self.disarm_retry_timer = None
        self.pending_disarm_repeats = 0
        self.su_params_path = Path(get_package_share_directory('crazyflie')) / 'config' / 'su_params.yaml'
        self.calibration_active = False
        self.calibration_start_time = None
        self.calibration_run_id = None
        self.last_firmware_run_id = 0
        self.completed_run_ids = set()
        self.calibration_watchdog_timer = self.create_timer(0.5, self.calibration_watchdog)
        self.get_logger().info('su_interface node ready.')

    def keyboard_callback(self, msg):
        if not msg.data:
            return
        input_char = msg.data[0]
        if input_char == 'o':
            self._cancel_disarm_retries()
            self.cf.arm(True)
            self.get_logger().info('ARM command sent.')
        elif input_char == 'p':
            self.request_disarm()
        elif input_char == 'f':
            self.trigger_hover_calibration()

    def trigger_hover_calibration(self):
        if self.calibration_active:
            self.get_logger().warning('Calibration already running; duplicate f ignored.')
            return
        msg = Float64MultiArray()
        msg.data = [1.0]
        self.calibration_publisher.publish(msg)
        self.calibration_active = True
        self.calibration_start_time = self.get_clock().now()
        self.calibration_run_id = None
        self.get_logger().info('Firmware calibration start trigger sent once.')

    def calibration_status_callback(self, msg):
        if len(msg.values) != 6:
            return
        state = int(round(msg.values[0]))
        run_id = int(round(msg.values[1]))
        self.last_firmware_run_id = run_id
        if not self.calibration_active:
            return
        if state in (CALIB_IMU_TRIM, CALIB_COM_COLLECT):
            if self.calibration_run_id is None:
                self.calibration_run_id = run_id
            return
        if self.calibration_run_id is None or run_id != self.calibration_run_id:
            return
        if state == CALIB_ERROR:
            self.calibration_active = False
            self.get_logger().error('Firmware calibration failed.')
            return
        if state != CALIB_DONE or run_id in self.completed_run_ids:
            return
        values = [float(value) for value in msg.values[2:6]]
        if not all(math.isfinite(value) for value in values):
            self.calibration_active = False
            self.get_logger().error('Calibration result contains NaN or Inf; YAML not updated.')
            return
        try:
            self._persist_calibration(values[0], values[1], values[2], values[3])
        except Exception as exc:
            self.calibration_active = False
            self.get_logger().error(f'Calibration YAML update failed: {exc}')
            return
        self.completed_run_ids.add(run_id)
        self.calibration_active = False
        result_msg = String()
        result_msg.data = '[calibration done]'
        self.calibration_result_publisher.publish(result_msg)
        print('[calibration done]', flush=True)

    def calibration_watchdog(self):
        if not self.calibration_active or self.calibration_start_time is None:
            return
        elapsed = (self.get_clock().now() - self.calibration_start_time).nanoseconds * 1e-9
        if elapsed > CALIBRATION_TIMEOUT_SEC:
            self.calibration_active = False
            self.get_logger().error('Firmware calibration timed out; YAML not updated.')

    def _persist_calibration(self, acc_roll, acc_pitch, com_x, com_y):
        path = self.su_params_path.resolve()
        lines = path.read_text(encoding='utf-8').splitlines(keepends=True)
        replacements = {
            ('su_wrench', 'comOffX'): com_x,
            ('su_wrench', 'comOffY'): com_y,
            ('imu_sensors', 'accTrimRoll'): acc_roll,
            ('imu_sensors', 'accTrimPitch'): acc_pitch,
        }
        found = set()
        section = None
        for index, line in enumerate(lines):
            stripped = line.lstrip(' ')
            indent = len(line) - len(stripped)
            if indent == 6 and stripped.rstrip().endswith(':'):
                section = stripped.strip()[:-1]
                continue
            if indent <= 6 and stripped.strip() and not stripped.lstrip().startswith('#'):
                section = None
            if section not in ('su_wrench', 'imu_sensors') or indent != 8 or ':' not in stripped:
                continue
            key = stripped.split(':', 1)[0].strip()
            lookup = (section, key)
            if lookup not in replacements:
                continue
            newline = '\n' if line.endswith('\n') else ''
            body = line[:-1] if newline else line
            comment = ''
            if '#' in body:
                comment = '  #' + body.split('#', 1)[1]
            lines[index] = ' ' * 8 + f'{key}: {replacements[lookup]:.6f}' + comment + newline
            found.add(lookup)
        if found != set(replacements):
            missing = sorted(set(replacements) - found)
            raise RuntimeError(f'missing YAML calibration keys: {missing}')
        fd, temporary_name = tempfile.mkstemp(prefix=path.name + '.', suffix='.tmp', dir=path.parent)
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as output:
                output.writelines(lines)
                output.flush()
                os.fsync(output.fileno())
            os.replace(temporary_name, path)
        except Exception:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass
            raise

    def request_disarm(self):
        # Disarm has priority over calibration retries. Do not keep sending
        # hover-calibration app-channel packets after the operator presses p.
        # Send the safety-critical arming-off request first. The setpoint-stop
        # notification is useful, but disarming must not wait behind it.
        self.cf.arm(False)
        self.cf.notifySetpointsStop(remainValidMillisecs=0)
        self.pending_disarm_repeats = DISARM_RETRY_COUNT - 1
        self.get_logger().warning(
            'DISARM command sent; repeating arming-off at %.1f Hz for %.1f s.'
            % (
                1.0 / DISARM_RETRY_PERIOD_SEC,
                DISARM_RETRY_PERIOD_SEC * DISARM_RETRY_COUNT,
            )
        )

        if self.disarm_retry_timer is not None:
            self.disarm_retry_timer.cancel()

        self.disarm_retry_timer = self.create_timer(
            DISARM_RETRY_PERIOD_SEC,
            self._retry_disarm,
        )

    def _cancel_disarm_retries(self):
        self.pending_disarm_repeats = 0
        if self.disarm_retry_timer is not None:
            self.disarm_retry_timer.cancel()
            self.disarm_retry_timer = None

    def _retry_disarm(self):
        if self.pending_disarm_repeats <= 0:
            self._cancel_disarm_retries()
            self.get_logger().info('DISARM retry sequence completed.')
            return

        self.cf.arm(False)
        self.pending_disarm_repeats -= 1

    def shutdown(self):
        self.cf.land(targetHeight=0.04, duration=2.5)
        self.timeHelper.sleep(3.0)



def main(args=None):
    node = SuInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
