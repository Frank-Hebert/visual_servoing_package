#!/usr/bin/env python3
import numpy as np
import rospy
import os
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, LanePose, WheelsCmdStamped, BoolStamped, FSMState, StopLineReading

from lane_controller.controller import PurePursuitLaneController
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from visual_servo.estimation import PoseEstimator
from visual_servo.control import Trajectory
from visual_servo.config import (BUMPER_TO_CENTER_DIST, CAMERA_MODE, CIRCLE_MIN_AREA,
                    CIRCLE_MIN_DISTANCE, CIRCLE_PATTERN_HEIGHT,
                    CIRCLE_PATTERN_WIDTH, TARGET_DIST)


class LaneControllerNode(DTROS):
    """Computes control action.
    The node compute the commands in form of linear and angular velocitie.
    The configuration parameters can be changed dynamically while the node is running via ``rosparam set`` commands.
    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use
    Configuration:

    Publisher:
        ~car_cmd (:obj:`Twist2DStamped`): The computed control action
    Subscribers:
        ~lane_pose (:obj:`LanePose`): The lane pose estimate from the lane filter
    """

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(LaneControllerNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.CONTROL
        )

        # Add the node parameters to the parameters dictionary
        self.params = dict()
        self.pp_controller = PurePursuitLaneController(self.params)

        # Construct publishers
        self.pub_car_cmd = rospy.Publisher("~car_cmd",
                                           Twist2DStamped,
                                           queue_size=1,
                                           dt_topic_type=TopicType.CONTROL)

        # Construct subscribers
        # self.sub_lane_reading = rospy.Subscriber("~lane_pose",
        #                                          LanePose,
        #                                          self.cbLanePoses,
        #                                          queue_size=1)

        # self.sub_image = rospy.Subscriber(f"/{os.environ['VEHICLE_NAME']}/camera_node/image/compressed", CompressedImage, self.cb_image, queue_size=1)

        self.sub_image = rospy.Subscriber(f"/{os.environ['VEHICLE_NAME']}/camera_node/image/compressed", CompressedImage, self.cb_image, queue_size=1)

        self.sub_info = rospy.Subscriber(f"/{os.environ['VEHICLE_NAME']}/camera_node/camera_info", CameraInfo, self.cb_process_camera_info, queue_size=1)

        self.log("Initialized!")

        self.last_v = 0.2
        self.last_w = 0
        self.bridge = CvBridge()

        self.last_stamp = rospy.Time.now()

        self.pcm = PinholeCameraModel()
        self.pose_estimator = PoseEstimator(min_area=CIRCLE_MIN_AREA,
                                            min_dist_between_blobs=CIRCLE_MIN_DISTANCE,
                                            height=CIRCLE_PATTERN_HEIGHT,
                                            width=CIRCLE_PATTERN_WIDTH,
                                            target_distance=TARGET_DIST,
                                            camera_mode=CAMERA_MODE,
                                            )
        self.trajectory = Trajectory()

    def cb_process_camera_info(self, msg):
        """
        Callback that stores the intrinsic calibration into a PinholeCameraModel object.

        Args:

            msg (:obj:`sensor_msgs.msg.CameraInfo`): Intrinsic properties of the camera.
        """

        self.pcm.fromCameraInfo(msg)
        self.pose_estimator.initialize_camera_matrix(self.pcm.intrinsicMatrix(), self.pcm.distortionCoeffs())

    def cb_image(self, image_msg):
        """Callback receiving pose messages

        Args:
            input_pose_msg (:obj:`LanePose`): Message containing information about the current lane pose.
        """

        now = rospy.Time.now()

        dt = (now - self.last_stamp).to_sec()

        self.last_stamp = now

        if self.pose_estimator.initialized:
            image_cv = self.bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")

            target_detected, estimated_pose = self.pose_estimator.get_pose(image_cv)
            # print(estimated_pose)




            if target_detected:
                self.trajectory.update(estimated_pose)




            elif self.trajectory.is_initialized():
                self.trajectory.predict(dt)

            if self.trajectory.is_initialized():
                v, w = self.trajectory.get_commands()
                car_control_msg = Twist2DStamped()
                car_control_msg.header = image_msg.header
                car_control_msg.v = v
                car_control_msg.omega = w

                self.publishCmd(car_control_msg)


    # def cbLanePoses(self, input_pose_msg):
    #     """Callback receiving pose messages
    #
    #     Args:
    #         input_pose_msg (:obj:`LanePose`): Message containing information about the current lane pose.
    #     """
    #     self.pose_msg = input_pose_msg
    #
    #     car_control_msg = Twist2DStamped()
    #     car_control_msg.header = self.pose_msg.header
    #
    #     # TODO This needs to get changed
    #
    #     v, w = self.pp_controller.pure_pursuit(self.pose_msg.d, self.pose_msg.phi, self.last_v, self.last_w,
    #                                            self.vehicle_detection_msg)
    #
    #     self.last_v = v
    #     self.last_w = w
    #
    #     car_control_msg.v = v
    #     car_control_msg.omega = w
    #
    #     self.publishCmd(car_control_msg)

    def publishCmd(self, car_cmd_msg):
        """Publishes a car command message.

        Args:
            car_cmd_msg (:obj:`Twist2DStamped`): Message containing the requested control action.
        """
        self.pub_car_cmd.publish(car_cmd_msg)

    def cbParametersChanged(self):
        """Updates parameters in the controller object."""

        self.controller.update_parameters(self.params)


if __name__ == "__main__":
    # Initialize the node
    lane_controller_node = LaneControllerNode(node_name='lane_controller_node')
    # Keep it spinning
    rospy.spin()
