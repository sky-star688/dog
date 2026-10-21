import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    import serial
except ImportError:
    serial = None


class ArmSerialBridge(Node):
    """Bridge semantic ROS commands to the MyArm STM32 ASCII serial protocol."""

    def __init__(self):
        super().__init__("arm_serial_bridge")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("read_timeout_sec", 0.05)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("home_command", "$DJR!")
        self.declare_parameter("stop_command", "$DST!")
        self.declare_parameter("pick_command", "$DGS:1!")
        self.declare_parameter("place_command", "$DGS:2!")
        self.declare_parameter("command_topic", "/arm/command")
        self.declare_parameter("status_topic", "/arm/status")

        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.serial_port = None
        self.lock = threading.Lock()
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.create_subscription(
            String,
            str(self.get_parameter("command_topic").value),
            self.on_command,
            10,
        )
        self.create_service(Trigger, "/arm/home", self.on_home)
        self.create_service(Trigger, "/arm/stop", self.on_stop)
        self.create_timer(0.05, self.read_serial)
        self.open_serial()

    def publish_status(self, text):
        message = String()
        message.data = text
        self.status_pub.publish(message)

    def open_serial(self):
        if self.dry_run:
            self.get_logger().warn("Arm bridge is in dry_run mode; no serial bytes will be sent")
            return
        if serial is None:
            self.get_logger().error("pyserial is missing; install python3-serial")
            return
        try:
            self.serial_port = serial.Serial(
                port=str(self.get_parameter("port").value),
                baudrate=int(self.get_parameter("baudrate").value),
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=float(self.get_parameter("read_timeout_sec").value),
            )
            self.get_logger().info("Arm serial port opened: %s" % self.serial_port.port)
        except Exception as exc:
            self.get_logger().error("Cannot open arm serial port: %s" % exc)

    def resolve_command(self, semantic_command):
        command = semantic_command.strip()
        upper = command.upper()
        mapping = {
            "HOME": str(self.get_parameter("home_command").value),
            "STOP": str(self.get_parameter("stop_command").value),
            "PICK": str(self.get_parameter("pick_command").value),
            "PLACE": str(self.get_parameter("place_command").value),
        }
        if upper in mapping:
            return mapping[upper]
        if upper.startswith("GROUP:"):
            group = int(command.split(":", 1)[1])
            if group < 0 or group > 9999:
                raise ValueError("action group must be in [0, 9999]")
            return "$DGS:%d!" % group
        if upper.startswith("KINEMATICS:"):
            return "$KMS:%s!" % command.split(":", 1)[1].rstrip("!")
        if upper.startswith("RAW:"):
            raw = command.split(":", 1)[1]
            if not raw or raw[-1] not in "!}":
                raise ValueError("RAW command must end with ! or }")
            return raw
        raise ValueError("supported commands: HOME, STOP, PICK, PLACE, GROUP:n, KINEMATICS:x,y,z,ms, RAW:...")

    def send(self, command):
        if self.dry_run:
            self.get_logger().warn("DRY RUN arm command: %s" % command)
            self.publish_status("DRY_RUN:%s" % command)
            return True, "dry-run command accepted"
        if self.serial_port is None or not self.serial_port.is_open:
            return False, "serial port is not open"
        try:
            with self.lock:
                self.serial_port.write(command.encode("ascii"))
                self.serial_port.flush()
            self.publish_status("TX:%s" % command)
            return True, "sent"
        except Exception as exc:
            return False, str(exc)

    def on_command(self, message):
        try:
            command = self.resolve_command(message.data)
            ok, detail = self.send(command)
            if not ok:
                self.get_logger().error(detail)
        except Exception as exc:
            self.get_logger().error("Invalid arm command: %s" % exc)
            self.publish_status("ERROR:%s" % exc)

    def on_home(self, request, response):
        del request
        response.success, response.message = self.send(
            str(self.get_parameter("home_command").value)
        )
        return response

    def on_stop(self, request, response):
        del request
        response.success, response.message = self.send(
            str(self.get_parameter("stop_command").value)
        )
        return response

    def read_serial(self):
        if self.serial_port is None or not self.serial_port.is_open:
            return
        try:
            waiting = self.serial_port.in_waiting
            if waiting:
                data = self.serial_port.read(waiting).decode("ascii", errors="replace")
                self.publish_status("RX:%s" % data)
        except Exception as exc:
            self.get_logger().error("Serial read failed: %s" % exc)
            time.sleep(0.1)

    def destroy_node(self):
        if self.serial_port is not None and self.serial_port.is_open:
            self.send(str(self.get_parameter("stop_command").value))
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
