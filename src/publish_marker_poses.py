#!/usr/bin/env python3

import rospy
import yaml
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Quaternion, TransformStamped, Pose
import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import PoseStamped
from tf.transformations import *


class PreGraspPublisher:
    tfBuffer = None
    pose_marker_marker_frame = None
    subscriber = None
    publisher = None
    broadcaster = None

    def __init__(self) -> None:
        # Initialize node
        rospy.init_node("publish_marker_poses", anonymous=True)

        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)
        self.subscriber = rospy.Subscriber(
            "/ar_pose_marker", AlvarMarkers, self.publish_marker_pregrasp
        )

        self.broadcaster = tf2_ros.TransformBroadcaster()

        self.pose_marker_marker_frame = Pose()
        self.pose_marker_marker_frame.position.x = 0.00
        self.pose_marker_marker_frame.position.y = -0.05
        self.pose_marker_marker_frame.position.z = 0.30

        quat = quaternion_from_euler(0, math.pi, 0)
        self.pose_marker_marker_frame.orientation.x = quat[0]
        self.pose_marker_marker_frame.orientation.y = quat[1]
        self.pose_marker_marker_frame.orientation.z = quat[2]
        self.pose_marker_marker_frame.orientation.w = quat[3]

    def compute_pregrasp(self, marker):


        pose_stamped_marker_marker_frame = PoseStamped()
        pose_stamped_marker_marker_frame.pose = self.pose_marker_marker_frame
        pose_stamped_marker_marker_frame.header.frame_id = "ar_marker_" + str(marker.id)
        pose_stamped_marker_marker_frame.header.stamp = rospy.Time.now()

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            pose_marker_base_frame = self.tfBuffer.transform(
                pose_stamped_marker_marker_frame, "base_link", rospy.Duration(1)
            )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            raise

        rospy.loginfo(f"pose: {pose_stamped_marker_marker_frame}")
        # broadcast transform
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = "ar_marker_" + str(marker.id)
        transform_stamped.child_frame_id = "pregrasp_" + str(marker.id)
        transform_stamped.transform.translation.x = pose_stamped_marker_marker_frame.pose.position.x
        transform_stamped.transform.translation.y = pose_stamped_marker_marker_frame.pose.position.y
        transform_stamped.transform.translation.z = pose_stamped_marker_marker_frame.pose.position.z
        # q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
        # transform_stamped.transform.rotation.x = q[0]
        # transform_stamped.transform.rotation.y = q[1]
        # transform_stamped.transform.rotation.z = q[2]
        # transform_stamped.transform.rotation.w = q[3]
        transform_stamped.transform.rotation = pose_stamped_marker_marker_frame.pose.orientation
        # rospy.loginfo(f"original: {pose_stamped_marker_marker_frame.pose.position.z}")
        # rospy.loginfo(f"original: {transform_stamped.transform.translation.z}")
        # rospy.loginfo(f"transform: {transform_stamped}")

        return transform_stamped

    def publish_marker_pregrasp(self, data):
        # for marker in msg
        rospy.loginfo(rospy.get_caller_id())
        for marker in data.markers:
            # compute pre-grasp
            pregrasp_transform_stamped = self.compute_pregrasp(marker)
            # publish list of pre-grasp
            self.broadcaster.sendTransform(pregrasp_transform_stamped)


if __name__ == "__main__":

    PreGraspPublisher()

    rate = rospy.Rate(10)

    # Run publisher
    while not rospy.is_shutdown():
        rate.sleep()
