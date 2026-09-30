import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
from px4_msgs.msg import VehicleLocalPosition
import csv, math

class Recorder(Node):
    def __init__(self):
        super().__init__('replay_recorder')
        self.file = open('replay_data.csv', 'w')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['t_ns', 'type', 'x', 'y', 'z', 'yaw'])
        self.last_pose = None
        
        qos = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Detection3DArray, '/pl/perception/tags', self.det_cb, 10)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.pos_cb, qos)

    def pos_cb(self, msg):
        if not math.isfinite(msg.heading): return
        self.last_pose = (msg.x, msg.y, msg.z, msg.heading)
        t = self.get_clock().now().nanoseconds
        self.writer.writerow([t, 'pose', msg.x, msg.y, msg.z, msg.heading])

    def det_cb(self, msg):
        if not msg.detections or not msg.detections[0].results: return
        p = msg.detections[0].results[0].pose.pose.position
        t = self.get_clock().now().nanoseconds
        self.writer.writerow([t, 'det', p.x, p.y, p.z, 0])

def main():
    rclpy.init()
    node = Recorder()
    print("Recording... Ctrl+C to stop.")
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.file.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
