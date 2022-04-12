#!/usr/bin/env python3

from email import header
import rospy
import yaml
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Quaternion, TransformStamped, Pose
import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import PoseStamped
from tf.transformations import *
from ratatouille_pose_transforms.transforms import PoseTransforms


class PreGraspPublisher:
    tfBuffer = None
    pose_marker_marker_frame = None
    marker_pose_origin = None
    subscriber = None
    broadcaster = None
    pose_transformer = None

    def __init__(self) -> None:
        # Initialize node
        rospy.init_node("publish_marker_poses", anonymous=True)

        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)
        self.subscriber = rospy.Subscriber(
            "/ar_pose_marker", AlvarMarkers, self.publish_marker_pregrasp, queue_size=1
        )

        # self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.pose_transformer = PoseTransforms()

        # create a pose with container grasping offset
        self.pose_marker_marker_frame = Pose()
        self.pose_marker_marker_frame.position.x = 0.00
        self.pose_marker_marker_frame.position.y = -0.05
        self.pose_marker_marker_frame.position.z = 0.30
        quat = quaternion_from_euler(0, math.pi, 0)
        self.pose_marker_marker_frame.orientation.x = quat[0]
        self.pose_marker_marker_frame.orientation.y = quat[1]
        self.pose_marker_marker_frame.orientation.z = quat[2]
        self.pose_marker_marker_frame.orientation.w = quat[3]

        # create a pose at origin
        self.marker_pose_origin = Pose()
        self.marker_pose_origin.position.x = 0.00
        self.marker_pose_origin.position.y = 0.00
        self.marker_pose_origin.position.z = 0.00
        quat = quaternion_from_euler(0, 0, 0)
        self.marker_pose_origin.orientation.x = quat[0]
        self.marker_pose_origin.orientation.y = quat[1]
        self.marker_pose_origin.orientation.z = quat[2]
        self.marker_pose_origin.orientation.w = quat[3]

    def compute_pregrasp(self, marker):

        # get marker and extract pose, and correct pitch and roll
        pose_marker_corrected_global = self.pose_transformer.transform_pose_to_frame(
            pose_source=self.marker_pose_origin,
            header_frame_id="ar_marker_" + str(marker.id),
            base_frame_id="base_link",
        )
        if pose_marker_corrected_global is None:
            return None

        # Correct yaw and roll
        # print(pose_marker_base_frame)
        euler_corrected = euler_from_quaternion(
            (
                pose_marker_corrected_global.pose.orientation.x,
                pose_marker_corrected_global.pose.orientation.y,
                pose_marker_corrected_global.pose.orientation.z,
                pose_marker_corrected_global.pose.orientation.w,
            )
        )
        # rospy.loginfo(euler_corrected)
        # set roll and pitch to zero
        quat_corrected = quaternion_from_euler(math.pi / 2, 0, euler_corrected[2])

        pose_marker_corrected_global.pose.orientation.x = quat_corrected[0]
        pose_marker_corrected_global.pose.orientation.y = quat_corrected[1]
        pose_marker_corrected_global.pose.orientation.z = quat_corrected[2]
        pose_marker_corrected_global.pose.orientation.w = quat_corrected[3]

        # broadcast transform
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = rospy.Time.now()
        # transform_stamped.header.frame_id = "ar_marker_" + str(marker.id)
        transform_stamped.header.frame_id = "base_link"
        transform_stamped.child_frame_id = "ar_marker_" + str(marker.id) + "_corrected"
        transform_stamped.transform.translation.x = (
            pose_marker_corrected_global.pose.position.x
        )
        transform_stamped.transform.translation.y = (
            pose_marker_corrected_global.pose.position.y
        )
        transform_stamped.transform.translation.z = (
            pose_marker_corrected_global.pose.position.z
        )
        transform_stamped.transform.rotation = (
            pose_marker_corrected_global.pose.orientation
        )
        self.broadcaster.sendTransform(transform_stamped)

        # now we have corrected pose in global frame- we need to convert this pose to marker frame
        pose_marker_corrected_marker = self.pose_transformer.transform_pose_to_frame(
            pose_source=self.pose_marker_marker_frame,
            header_frame_id="ar_marker_" + str(marker.id) + "_corrected",
            base_frame_id="ar_marker_" + str(marker.id) + "_corrected",
        )
        if pose_marker_corrected_marker is None:
            return None

        # End correct yaw and roll

        # end - get marker and extract pose, and correct pitch and roll
        # rospy.loginfo(pose_marker_corrected_marker)

        """
        return transform_stamped
        """
        # broadcast transform
        transform_stamped2 = TransformStamped()
        transform_stamped2.header.stamp = rospy.Time.now()
        # transform_stamped2.header.frame_id = "ar_marker_" + str(marker.id)
        transform_stamped2.header.frame_id = (
            "ar_marker_" + str(marker.id) + "_corrected"
        )
        transform_stamped2.child_frame_id = "pregrasp_" + str(marker.id)
        transform_stamped2.transform.translation.x = (
            pose_marker_corrected_marker.pose.position.x
        )
        transform_stamped2.transform.translation.y = (
            pose_marker_corrected_marker.pose.position.y
        )
        transform_stamped2.transform.translation.z = (
            pose_marker_corrected_marker.pose.position.z
        )
        transform_stamped2.transform.rotation = (
            pose_marker_corrected_marker.pose.orientation
        )

        return transform_stamped2

    def publish_marker_pregrasp(self, data):
        # for marker in msg
        for marker in data.markers:
            # compute pre-grasp
            pregrasp_transform_stamped = self.compute_pregrasp(marker)
            # publish list of pre-grasp
            if pregrasp_transform_stamped is not None:
                self.broadcaster.sendTransform(pregrasp_transform_stamped)


if __name__ == "__main__":

    PreGraspPublisher()

    rate = rospy.Rate(10)

    # Run publisher
    while not rospy.is_shutdown():
        rate.sleep()
