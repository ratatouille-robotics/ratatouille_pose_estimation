import rospy
from geometry_msgs.msg import Pose, PoseStamped
import tf2_ros


class PoseTransforms:
    tfBuffer = None

    def __init__(self) -> None:
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)

    def transform_pose_to_frame(
        self, pose_source: Pose, header_frame_id: str, base_frame_id: str
    ):

        _pose_stamped = PoseStamped()
        _pose_stamped.pose = pose_source
        _pose_stamped.header.frame_id = header_frame_id
        _pose_stamped.header.stamp = rospy.Time.now()

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            pose_marker_corrected_global = self.tfBuffer.transform(
                _pose_stamped, base_frame_id, rospy.Duration(1)
            )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None
        return pose_marker_corrected_global
