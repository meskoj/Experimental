import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Float64MultiArray
import time
import cv2
from sensor_msgs.msg import Image


class ArucoNode(Node):

    def __init__(self, initial_state):
        super().__init__('aruco_node')
        self.state = initial_state

        self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.controller_pub = self.create_publisher(
            Float64MultiArray, '/joint_cam_controller/commands', 10)
        self.markers_pub = self.create_publisher(Image, 'aruco_markers', 10)

        self.bridge = CvBridge()
        self.markers_list = []

        if self.state == "find":
            # Start spinning
            msg = Float64MultiArray()
            msg.data = [1.0]
            self.controller_pub.publish(msg)
            self.get_logger().info(f"Publishing: {msg.data}")

        # Set the aruco dictionary and parameters
        self.aruco_dictionary = cv2.aruco.Dictionary_get(
            cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()

    def image_callback(self, img):

        cv_image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        corners, marker_ids, _ = cv2.aruco.detectMarkers(
            cv_image, self.aruco_dictionary, parameters=self.aruco_parameters)

        if self.state == "find":
            if marker_ids is not None:
                # For each detected marker add it to the list of markers if it is not already there
                for i in range(len(marker_ids)):
                    if marker_ids[i] not in self.markers_list:
                        self.markers_list.append(marker_ids[i])
                        self.get_logger().info(str(marker_ids[i]))
            if len(self.markers_list) == 5:
                # Reorder markers and change state
                self.markers_list.sort()
                self.get_logger().info(
                    f'Found all markers. Markers list: {", ".join(map(str, self.markers_list))}')
                self.state = "search"
                self.searched_marker = None

        elif self.state == "search":
            if self.searched_marker is None:
                if (len(self.markers_list) == 0):
                    self.state = "finish"
                else:
                    self.searched_marker = self.markers_list.pop(0)
            else:
                if marker_ids is not None:
                    for i in range(len(marker_ids)):
                        if self.searched_marker == marker_ids[i]:
                            # Calculate the center of the detected marker and the center of the image
                            marker_center = corners[i][0].mean(axis=0)
                            image_center = (
                                cv_image.shape[1] / 2, cv_image.shape[0] / 2)

                            # Check if the marker is centered and publish the image
                            if abs(marker_center[0] - image_center[0]) < 10 and abs(marker_center[1] - image_center[1]) < 10:
                                outputImage = cv_image.copy()
                                outputImage = cv2.aruco.drawDetectedMarkers(
                                    cv_image, corners, marker_ids)
                                window_name = f"Image window {self.searched_marker}"
                                self.markers_pub.publish(
                                    self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
                                self.get_logger().info(f"Published marker")
                                cv2.imshow(window_name, outputImage)
                                cv2.waitKey(10000)
                                self.searched_marker = None

        elif self.state == "finish":
            self.get_logger().info("Task completed")
            msg = Float64MultiArray()
            msg.data = [0.0]
            self.controller_pub.publish(msg)
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    time.sleep(10)
    rclpy.init(args=args)

    node = ArucoNode("find")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
