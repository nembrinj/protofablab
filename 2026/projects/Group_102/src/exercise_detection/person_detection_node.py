import time
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, Float32MultiArray


class PersonDetectionNode(Node):
    def __init__(self):
        super().__init__("person_detection_node")

        # Parameters
        self.declare_parameter("image_topic", "/image_raw/compressed")
        self.declare_parameter("show_preview", True)
        self.declare_parameter("model_path", "")
        self.declare_parameter("min_landmark_visibility", 0.5)
        self.declare_parameter("frame_margin", 0.03)
        self.declare_parameter("horizontal_tolerance", 0.08)

        self.image_topic = self.get_parameter("image_topic").value
        self.show_preview = self.get_parameter("show_preview").value

        self.min_visibility = float(self.get_parameter("min_landmark_visibility").value)
        self.frame_margin = float(self.get_parameter("frame_margin").value)
        self.horizontal_tolerance = float(
            self.get_parameter("horizontal_tolerance").value
        )

        # Model path
        model_path = self.get_parameter("model_path").value

        if not model_path:
            model_path = (
                Path(__file__).resolve().parent.parent
                / "models"
                / "pose_landmarker_lite.task"
            )

        model_path = Path(model_path).expanduser()

        if not model_path.is_file():
            raise FileNotFoundError(f"MediaPipe model not found: {model_path}")

        # MediaPipe Pose Landmarker
        BaseOptions = mp.tasks.BaseOptions
        PoseLandmarker = mp.tasks.vision.PoseLandmarker
        PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
        RunningMode = mp.tasks.vision.RunningMode

        options = PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=str(model_path)),
            running_mode=RunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=0.5,
            min_pose_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )

        self.landmarker = PoseLandmarker.create_from_options(options)

        # Publishers: only the 5 required topics
        self.detected_pub = self.create_publisher(Bool, "/person/detected", 10)
        self.center_x_pub = self.create_publisher(Float32, "/person/center_x", 10)
        self.bounding_box_pub = self.create_publisher(
            Float32MultiArray, "/person/bounding_box", 10
        )
        self.upper_body_pub = self.create_publisher(
            Bool, "/person/upper_body_visible", 10
        )
        self.t_pose_pub = self.create_publisher(Bool, "/person/t_pose_visible", 10)

        # Camera subscription
        self.image_sub = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.last_timestamp_ms = 0

        self.get_logger().info(
            f"Person detection started. Listening to {self.image_topic}"
        )
        self.get_logger().info(f"Using model: {model_path}")

    def is_visible(self, landmark):
        visibility = getattr(landmark, "visibility", None)

        return visibility is not None and visibility >= self.min_visibility

    def is_inside_frame(self, landmark):
        margin = self.frame_margin

        return (
            margin <= landmark.x <= 1.0 - margin
            and margin <= landmark.y <= 1.0 - margin
        )

    def is_between(self, value, endpoint_a, endpoint_b):
        return min(endpoint_a, endpoint_b) <= value <= max(endpoint_a, endpoint_b)

    def publish_visibility(self, upper_body=False, t_pose=False):
        self.upper_body_pub.publish(Bool(data=upper_body))
        self.t_pose_pub.publish(Bool(data=t_pose))

    def analyze_upper_body(self, landmarks):

        # MediaPipe landmark indices
        left_shoulder = landmarks[11]
        right_shoulder = landmarks[12]

        left_elbow = landmarks[13]
        right_elbow = landmarks[14]

        left_wrist = landmarks[15]
        right_wrist = landmarks[16]

        left_hip = landmarks[23]
        right_hip = landmarks[24]

        # Check torso visibility
        torso_landmarks = [
            left_shoulder,
            right_shoulder,
            left_hip,
            right_hip,
        ]

        upper_body_visible = all(
            self.is_visible(p) and self.is_inside_frame(p) for p in torso_landmarks
        )

        # Check arm visibility
        arm_landmarks = [
            left_shoulder,
            right_shoulder,
            left_elbow,
            right_elbow,
            left_wrist,
            right_wrist,
        ]

        arms_visible = all(
            self.is_visible(p) and self.is_inside_frame(p) for p in arm_landmarks
        )

        # Check approximate horizontal T-pose
        shoulder_center_y = (left_shoulder.y + right_shoulder.y) / 2.0

        elbows_horizontal = (
            abs(left_elbow.y - shoulder_center_y) <= self.horizontal_tolerance
            and abs(right_elbow.y - shoulder_center_y) <= self.horizontal_tolerance
        )

        wrists_horizontal = (
            abs(left_wrist.y - shoulder_center_y) <= self.horizontal_tolerance
            and abs(right_wrist.y - shoulder_center_y) <= self.horizontal_tolerance
        )

        # Check that wrists extend beyond both shoulders
        shoulder_min_x = min(left_shoulder.x, right_shoulder.x)
        shoulder_max_x = max(left_shoulder.x, right_shoulder.x)

        wrist_min_x = min(left_wrist.x, right_wrist.x)
        wrist_max_x = max(left_wrist.x, right_wrist.x)

        wrists_extended = wrist_min_x < shoulder_min_x and wrist_max_x > shoulder_max_x

        # Each elbow should be between its shoulder and wrist
        elbows_extended = self.is_between(
            left_elbow.x,
            left_shoulder.x,
            left_wrist.x,
        ) and self.is_between(
            right_elbow.x,
            right_shoulder.x,
            right_wrist.x,
        )

        t_pose_visible = (
            upper_body_visible
            and arms_visible
            and elbows_horizontal
            and wrists_horizontal
            and wrists_extended
            and elbows_extended
        )

        self.publish_visibility(
            upper_body=upper_body_visible,
            t_pose=t_pose_visible,
        )

        return upper_body_visible, t_pose_visible

    def image_callback(self, msg):

        # Decode compressed ROS image
        image_array = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)

        if frame is None:
            self.get_logger().warning("Could not decode camera frame")
            return

        height, width, _ = frame.shape

        # Convert BGR to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        mp_image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=rgb_frame,
        )

        # MediaPipe VIDEO mode requires increasing timestamps
        timestamp_ms = int(time.monotonic() * 1000)
        timestamp_ms = max(
            timestamp_ms,
            self.last_timestamp_ms + 1,
        )
        self.last_timestamp_ms = timestamp_ms

        result = self.landmarker.detect_for_video(
            mp_image,
            timestamp_ms,
        )

        person_detected = bool(result.pose_landmarks)

        self.detected_pub.publish(Bool(data=person_detected))

        if not person_detected:
            self.publish_visibility()
            status = "PERSON NOT DETECTED"

            if self.show_preview:
                cv2.putText(
                    frame,
                    status,
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2,
                )

        else:
            landmarks = result.pose_landmarks[0]

            # Analyze torso and T-pose
            upper_body_visible, t_pose_visible = self.analyze_upper_body(landmarks)

            # Calculate bounding box from visible landmarks
            visible_points = [
                p
                for p in landmarks
                if (self.is_visible(p) and 0.0 <= p.x <= 1.0 and 0.0 <= p.y <= 1.0)
            ]

            if visible_points:
                x_values = [p.x for p in visible_points]
                y_values = [p.y for p in visible_points]

                x_min = min(x_values)
                y_min = min(y_values)
                x_max = max(x_values)
                y_max = max(y_values)

                # Use shoulder center for horizontal alignment.
                # This avoids the arms shifting the person's center.
                center_x = (landmarks[11].x + landmarks[12].x) / 2.0

                self.center_x_pub.publish(Float32(data=float(center_x)))

                box_msg = Float32MultiArray()
                box_msg.data = [
                    float(x_min),
                    float(y_min),
                    float(x_max),
                    float(y_max),
                ]
                self.bounding_box_pub.publish(box_msg)

                if self.show_preview:
                    self.draw_detection(
                        frame,
                        landmarks,
                        x_min,
                        y_min,
                        x_max,
                        y_max,
                        center_x,
                        upper_body_visible,
                        t_pose_visible,
                    )

        if self.show_preview:
            cv2.imshow("Person Detection - T Pose", frame)
            cv2.waitKey(1)

    def draw_detection(
        self,
        frame,
        landmarks,
        x_min,
        y_min,
        x_max,
        y_max,
        center_x,
        upper_body_visible,
        t_pose_visible,
    ):
        height, width, _ = frame.shape

        x1 = int(x_min * width)
        y1 = int(y_min * height)
        x2 = int(x_max * width)
        y2 = int(y_max * height)

        center_pixel_x = int(center_x * width)

        # Draw bounding box
        cv2.rectangle(
            frame,
            (x1, y1),
            (x2, y2),
            (0, 255, 0),
            2,
        )

        # Draw torso center
        cv2.circle(
            frame,
            (center_pixel_x, int(height / 2)),
            5,
            (0, 0, 255),
            -1,
        )

        # Draw shoulders, elbows, wrists and hips
        landmark_indices = [11, 12, 13, 14, 15, 16, 23, 24]

        for index in landmark_indices:
            landmark = landmarks[index]

            if self.is_visible(landmark):
                px = int(landmark.x * width)
                py = int(landmark.y * height)

                if 0 <= px < width and 0 <= py < height:
                    cv2.circle(
                        frame,
                        (px, py),
                        6,
                        (255, 200, 0),
                        -1,
                    )

        if t_pose_visible:
            status = "T-POSE: READY"
            color = (0, 255, 0)
        elif upper_body_visible:
            status = "UPPER BODY VISIBLE - ADJUST ARMS"
            color = (0, 165, 255)
        else:
            status = "UPPER BODY NOT FULLY VISIBLE"
            color = (0, 0, 255)

        cv2.putText(
            frame,
            status,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
        )

    def destroy_node(self):
        self.landmarker.close()

        if self.show_preview:
            cv2.destroyAllWindows()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = PersonDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
