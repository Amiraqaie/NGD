import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class UncalibratedCameraSimulator(Node):
    def __init__(self):
        super().__init__('uncalibrated_camera_simulator')

        # Create a callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Publishers for modified camera info
        self.left_camera_pub = self.create_publisher(CameraInfo, '/stereo/left/uncalibrated_camera_info', 10)
        self.right_camera_pub = self.create_publisher(CameraInfo, '/stereo/right/uncalibrated_camera_info', 10)

        # Publishers for modified image info
        self.left_image_pub = self.create_publisher(Image, '/stereo/left/uncalibrated_image', 10)
        self.right_image_pub = self.create_publisher(Image, '/stereo/right/uncalibrated_image', 10)

        # Subscribers to original camera info topics
        self.create_subscription(CameraInfo, '/stereo/left/camera_info', self.left_camera_callback, 10, callback_group=self.callback_group)
        self.create_subscription(CameraInfo, '/stereo/right/camera_info', self.right_camera_callback, 10, callback_group=self.callback_group)

        # Subscribers to original image topics
        self.create_subscription(Image, '/stereo/left/image_raw', self.left_image_callback, 10, callback_group=self.callback_group)
        self.create_subscription(Image, '/stereo/right/image_raw', self.right_image_callback, 10, callback_group=self.callback_group)

        self.get_logger().info("Uncalibrated camera simulator node started.")

    def modify_camera_info(self, camera_info, frame_id, baseline=0):
        """
        Modify the CameraInfo message to simulate an uncalibrated camera.
        """
        # Set intrinsic matrix to an identity matrix (or some uncalibrated values)
        camera_info.k = [457.84999843514953, 0.0, 320.5,
                         0.0, 457.84999843514953, 240.5,
                         0.0, 0.0, 1.0]

        # Set distortion coefficients to zero
        camera_info.distortion_model = "rational_polynomial"
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Set rectification matrix to an identity matrix
        camera_info.r = [1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0]

        # Set projection matrix to an identity matrix
        camera_info.p = [457.84999843514953, 0.0, 320.5, 0.0,
                         0.0, 457.84999843514953, 240.5, 0.0,
                         0.0, 0.0, 1.0, 0.0]
        
        baseline = 457.84999843514953 * baseline
        camera_info.p = [457.84999843514953, 0.0, 320.5, -baseline,
                         0.0, 457.84999843514953, 240.5, 0.0,
                         0.0, 0.0, 1.0, 0.0]

        # Set frame_id
        camera_info.header.frame_id = frame_id

        return camera_info

    def modify_image_frame_id(self, image, frame_id):
        """
        Modify the Image message to simulate a new frame_id.
        """
        image.header.frame_id = frame_id
        image.is_bigendian = 1
        return image

    def left_camera_callback(self, msg):
        """
        Callback for /left/camera/camera_info.
        """
        modified_msg = self.modify_camera_info(msg, "camera_left_optical_frame")
        self.left_camera_pub.publish(modified_msg)

    def right_camera_callback(self, msg):
        """
        Callback for /right/camera/camera_info.
        """
        modified_msg = self.modify_camera_info(msg, "camera_right_optical_frame", baseline=0.1)
        self.right_camera_pub.publish(modified_msg)

    def left_image_callback(self, msg):
        """
        Callback for /left/image_raw.
        """
        modified_msg = self.modify_image_frame_id(msg, "camera_left_optical_frame")
        self.left_image_pub.publish(modified_msg)

    def right_image_callback(self, msg):
        """
        Callback for /right/image_raw.
        """
        modified_msg = self.modify_image_frame_id(msg, "camera_right_optical_frame")
        self.right_image_pub.publish(modified_msg)


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    uncalibrated_camera_simulator = UncalibratedCameraSimulator()

    # Create a MultiThreadedExecutor to handle multiple threads
    executor = MultiThreadedExecutor(4)
    executor.add_node(uncalibrated_camera_simulator)

    try:
        # Run the executor, which processes the callbacks in multiple threads
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup and shutdown
        uncalibrated_camera_simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
