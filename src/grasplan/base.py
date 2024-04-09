import tf2_ros


class GraspPlanBase:
    def __init__(self) -> None:
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
