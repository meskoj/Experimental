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
        # Initialize the state machine with the initial state
        self.state = initial_state

         # Create the subscription to the camera image and the publisher for the controller and the markers
        self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.controller_pub = self.create_publisher(
            Float64MultiArray, '/joint_cam_controller/commands', 10)
        self.markers_pub = self.create_publisher(Image, '/aruco_markers', 10)

        self.bridge = CvBridge()
        self.markers_list = []

        if self.state == "find":
            # Start rotating the camera
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
                        self.get_logger().info('Added to list: "%s"' %str(marker_ids[i]))
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

                                # Calculate the radius of the circle
                                marker_radius = int(max(cv2.norm(corners[i][0][0] - corners[i][0][2]),cv2.norm(corners[i][0][1] - corners[i][0][3])) / 2)
                                cv2.circle(outputImage, (int(marker_center[0]), int(marker_center[1])), marker_radius, (0, 255, 0), 2)
                                cv2.putText(outputImage, f"id={int(marker_ids[i])}", (int(marker_center[0]), int(marker_center[1]) - marker_radius - 10),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                                self.markers_pub.publish(
                                    self.bridge.cv2_to_imgmsg(outputImage, "bgr8"))
                                self.get_logger().info(f"Published marker")
                                self.searched_marker = None

        elif self.state == "finish":
            self.get_logger().info("Task completed")
            msg = Float64MultiArray()
            msg.data = [0.0]
            self.controller_pub.publish(msg)
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    # Wait 10 seconds for the system to be ready
    time.sleep(10)
    rclpy.init(args=args)

    node = ArucoNode("find")
    rclpy.spin(node)

if __name__ == '__main__':
    main()
