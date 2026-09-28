import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
import cv2
import numpy as np
from ultralytics import YOLO

class YoloDetector(Node):
    def __init__(self):
        super().__init__("yolo_detector")
        
        # Load the trained model
        self.get_logger().info("Loading YOLO model...")
        self.model = YOLO("/home/madakie/precision_landing/runs/detect/runs/pad_v1/weights/best.pt")
        self.get_logger().info("Model loaded.")
        
        self.declare_parameter("marker_size", 0.4)
        self.marker_size = self.get_parameter("marker_size").value
        
        self.K = None
        self.ema_size = None
        self.alpha = 0.5  # Smoothing factor
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        
        self.sub_info = self.create_subscription(
            CameraInfo, "/pl/camera/camera_info", self.info_cb, 10)
        self.sub_img = self.create_subscription(
            Image, "/pl/camera/image_raw", self.img_cb, 10)
        self.pub = self.create_publisher(Detection3DArray, "/pl/perception/tags", 10)
        
    def info_cb(self, msg):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.fx = K[0, 0]
        self.fy = K[1, 1]
        self.cx = K[0, 2]
        self.cy = K[1, 2]
        self.K = K
        
    def _to_bgr(self, msg):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding == "bgr8":
            return buf.reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            return cv2.cvtColor(buf.reshape(msg.height, msg.width, 3), cv2.COLOR_RGB2BGR)
        if msg.encoding == "mono8":
            return cv2.cvtColor(buf.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
        self.get_logger().warn(f"unsupported encoding: {msg.encoding}", once=True)
        return None

    def img_cb(self, msg):
        if self.fx is None:
            return
        img = self._to_bgr(msg)
        if img is None:
            return

        # Run YOLO inference
        results = self.model(img, verbose=False)
        boxes = results[0].boxes
        
        out = Detection3DArray()
        out.header = msg.header

        if len(boxes) > 0:
            # Take the highest confidence detection
            best_idx = boxes.conf.argmax().item()
            box = boxes[best_idx]
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            
            # Bounding box metrics
            u_center = (x1 + x2) / 2.0
            v_center = (y1 + y2) / 2.0
            w_bbox = x2 - x1
            h_bbox = y2 - y1
            
            # Pinhole Depth Estimation with EMA smoothing to kill YOLO bbox jitter
            raw_proj_size = (w_bbox + h_bbox) / 2.0
            if self.ema_size is None:
                self.ema_size = raw_proj_size
                self.ema_u = u_center
                self.ema_v = v_center
            else:
                self.ema_size = self.alpha * raw_proj_size + (1 - self.alpha) * self.ema_size
                self.ema_u = self.alpha * u_center + (1 - self.alpha) * self.ema_u
                self.ema_v = self.alpha * v_center + (1 - self.alpha) * self.ema_v
            
            # Z = (focal_length * real_width) / projected_width
            z_cam = (self.fx * self.marker_size) / self.ema_size
            
            # X and Y offsets in camera frame (meters)
            x_cam = (self.ema_u - self.cx) * z_cam / self.fx
            y_cam = (self.ema_v - self.cy) * z_cam / self.fy
            
            det = Detection3D()
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(int(box.cls[0].item()))
            hyp.hypothesis.score = float(box.conf[0].item())
            
            # The controller expects (ox, oy, oz) matching ArUco's tvec output
            hyp.pose.pose.position.x = float(x_cam)
            hyp.pose.pose.position.y = float(y_cam)
            hyp.pose.pose.position.z = float(z_cam)
            
            # We don't estimate orientation from a 2D bbox, assume flat pad
            hyp.pose.pose.orientation.w = 1.0 
            
            det.results.append(hyp)
            
            det.bbox.center.position.x = float(u_center)
            det.bbox.center.position.y = float(v_center)
            det.bbox.size.x = float(w_bbox)
            det.bbox.size.y = float(h_bbox)
            out.detections.append(det)

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
