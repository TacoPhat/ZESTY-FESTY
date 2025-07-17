import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class ArmClient(Node):
    def __init__(self):
        super().__init__('arm_client')
        self.arm_client = self.create_client(SetBool, 'arming')
        self.armed = False

        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service')
        self.create_timer(1.0, self.arm_rov)

    def arm_rov(self):
        if self.armed:
            return
        self.get_logger().info('Sending arming command to ROV')
        request = SetBool.Request()
        request.data = True

        future = self.arm_client.call_async(request)
        future.add_done_callback(self.arm_response_callback)
        self.armed = True

    def disarm_rov(self):
        self.get_logger().info('Sending disarm command to ROV')
        request = SetBool.Request()
        request.data = False

        future = self.arm_client.call_async(request)
        future.add_done_callback(self.disarm_response_callback)

    def arm_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"ROV is armed: {response.message}")
            else:
                self.get_logger().warn(f"Arming failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def disarm_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"ROV is disarmed: {response.message}")
            else:
                self.get_logger().warn(f"Disarming failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ArmClient()
    try:
        # runs for 60 seconds
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Tttempting to disarm ROV")
        node.disarm_rov()
        rclpy.spin_once(node, timeout_sec=2.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()

