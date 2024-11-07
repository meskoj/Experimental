import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import time
import cv2
from sensor_msgs.msg import Image
from ros2_aruco_interfaces.msg import ArucoMarkers

found_all = False
searched_marker = None
class ArucoNode(Node):

    def __init__(self):
        super().__init__('aruco_node')
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10) #self.sub = self.create_subscription(CompressedImage, '/camera/image_raw', self.image_callback, 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, 'aruco_markers', 10)
        
        self.bridge = CvBridge()
        self.markers_list = []

        # Start spinning
        msg = Twist()
        msg.angular.z = 1.5
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
        
        self.aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        
    def image_callback(self,img):
        global found_all, searched_marker
        cv_image = self.bridge.imgmsg_to_cv2(img,desired_encoding='mono8')
        corners, marker_ids, rejected = cv2.aruco.detectMarkers(cv_image,self.aruco_dictionary,parameters=self.aruco_parameters)

        #cv2.aruco.drawDetectedMarkers(cv_image,corners,marker_ids)
        #cv2.imshow('Detected Markers', cv_image)
        #cv2.waitKey(100)
        if found_all == False:
            if marker_ids is not None:      
                for i in range(len(marker_ids)):
                    marker_id = marker_ids[i]
                    if marker_id not in self.markers_list:
                        self.markers_list.append(marker_id)
                        self.get_logger().info(str(marker_id))
                        #self.get_logger().info(f'Markers list: {", ".join(map(str, self.markers_list))}')
            if len(self.markers_list) == 5:
                    self.markers_list.sort()
                    self.get_logger().info(f'Found all markers. Markers list: {", ".join(map(str, self.markers_list))}')
                    found_all = True
        else:
            if searched_marker is None:
                if(len(self.markers_list)== 0):
                     self.get_logger().info("Finish")
                else:
                    searched_marker = self.markers_list.pop(0)
            else:
                if marker_ids is not None:  
                    for i in range(len(marker_ids)):
                        marker_id = marker_ids[i]
                        if searched_marker == marker_id:
                            marker_corners = corners[i][0]
                            center_x = int(marker_corners[0][0] + marker_corners[1][0] + marker_corners[2][0] + marker_corners[3][0]) / 4
                            center_y = int(marker_corners[0][1] + marker_corners[1][1] + marker_corners[2][1] + marker_corners[3][1]) / 4
                            radius = int(abs(marker_corners[2][0] - marker_corners[0][0])/2)
                            center = (int(center_x), int(center_y))

                            self.get_logger().info(str(corners[i]))
                            self.get_logger().info(str(center))
                            outputImage = cv_image.copy()
                            outputImage = cv2.circle(cv_image, center, radius, (255,0,0), 2)#cv2.aruco.drawDetectedMarkers(cv_image, corners, marker_ids)
                            window_name = f"Image window {marker_id}_{i}"
                            cv2.imshow(window_name, outputImage)    
                            cv2.waitKey(1)
                            searched_marker = None

def main(args=None):
    time.sleep(10)
    rclpy.init(args=args)

    node = ArucoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()