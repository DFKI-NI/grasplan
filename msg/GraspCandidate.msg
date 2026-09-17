geometry_msgs/Pose pose
# Final ranking score; also present in scores under the name "final".
float32 quality
# AnyGrasp jaw opening in metres; used as the MoveIt grasp posture.
float32 width
# Named score components (see GraspScore). May be empty for legacy producers.
grasplan/GraspScore[] scores
