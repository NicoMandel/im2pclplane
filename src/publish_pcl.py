"""
    Python implementation of the "Publish PCL" node - just for testing and debugging
"""

import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image, PointCloud
from geometry_msgs.msg import Point32
from image_geometry import PinholeCameraModel
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import quaternion_matrix

class PCLConverter:

    def __init__(self) -> None:
        image_topic = "/hbv_1615"
        pcl_topic = rospy.get_param("pcl_topic", default="pcl_plane")
        self.world_topic = rospy.get_param("world_frame", default="map")
        
        info_topic = image_topic + "/camera_info"
        cam_info = rospy.wait_for_message(info_topic, CameraInfo)
        self.cam_model = PinholeCameraModel()
        self.cam_model.fromCameraInfo(cam_info)
        self.pclpub = rospy.Publisher(pcl_topic, PointCloud, queue_size=1)

        # tf
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)

        # Plane definition
        pt1 = rospy.get_param("plane/point1", default=[0., 0., 0.])
        pt2 = rospy.get_param("plane/point2", default=[1., 0., 0.])
        pt3 = rospy.get_param("plane/point3", default=[0., 1., 0.])
        self.world_plane = np.asarray([pt1, pt2, pt3]).T

        self.rays = self.convertPixelsToRays()
        # image stuff
        self.bridge = CvBridge()

        # register the subscriber last
        rospy.Subscriber(image_topic + "/image_color", Image, self.imgCallback, queue_size=1)

    def imgCallback(self, msg):
        stamp = msg.header.stamp

        try:
            image_msg = self.bridge.imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            rospy.logwarn(e)
            return

        # looking up the pose at the time
        try:
            transform = self.tfBuffer.lookup_transform(self.cam_model.tfFrame(), self.world_topic, stamp, rospy.Duration(1.0/30.0))
        except tf2_ros.LookupException as e:
            rospy.loginfo(e)
            return

        ## actual magic happening here
        # TODO: write the conversions here
        T = self.constructTransform(transform)
        newpoints = self.convertPoints(self.world_plane, T)
        newplane = self.calcPlane(newpoints)
        
        contpoints = []
        rayOrigin = np.array([0.0, 0.0, 0.0])
        planeNormal = np.array([newplane[0], newplane[1], newplane[2]])
        for i in range(0,len(self.rays), 10):
            contactPoint = self.linePlaneIntersection(self.rays[i], rayOrigin, planeNormal, newpoints[:3,0])
            contpt = Point32()
            contpt.x = contactPoint[0]
            contpt.y = contactPoint[1]
            contpt.z = contactPoint[2]
            contpoints.append(contpt)
                

        # Turning into a pointcloud message
        pclmsg = PointCloud()
        pclmsg.header = msg.header
        pclmsg.points = contpoints
        self.pclpub.publish(pclmsg)
        

    def convertPixelsToRays(self):
        """
            Function to convert all pixels to rays
            ! Careful. The implementation of projectPixelTo3dRay seems to differ between Python / C++
        """

        rays = []
        for u in range(self.cam_model.width+1):
            for v in range(self.cam_model.height+1):
                rays.append(np.asarray(self.cam_model.projectPixelTo3dRay((u, v))))
        return rays 

    def constructTransform(self, transform):
        """
            Function to construct a transform T from a transform object message
        """
        rot = transform.transform.rotation
        q = np.asarray([rot.x, rot.y, rot.z, rot.w])
        R = quaternion_matrix(q)
        trans = transform.transform.translation
        t = np.asarray([trans.x, trans.y, trans.z])
        R[:3, -1] = t
        return R

    def convertPoints(self, points, T):
        """
            Method to convert the points of a plane into another space 
        """
        points = np.vstack((points, np.ones(points.shape[1])))
        return T @ points

    def calcPlane(self, points):
        """
            Calculating a plane equation from points
        """
        vec1 = points[:,1] - points[:,0]
        vec2 = points[:,2] - points[:,0]

        norm = np.cross(vec1[:3], vec2[:3])

        a = norm[0]
        b = norm[1]
        c = norm[2]
        # norm @ vec1[:3]            # Alternative
        d = -(a * vec1[0] + b* vec1[2] + c* vec1[2])

        fac = np.sqrt(np.dot(norm, norm))
        norm = norm / fac
        return np.append(norm, d/fac)

    def linePlaneIntersection(self, ray, rayOrigin, normal, coord):

        d = np.dot(normal, coord)
        x = (d - np.dot(normal, rayOrigin)) / np.dot(normal, ray)
        contact = rayOrigin + x* ray
        return contact


if __name__=="__main__":
    rospy.init_node("pclconvert", log_level=rospy.DEBUG)
    pclconvert = PCLConverter()
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr_once(e)