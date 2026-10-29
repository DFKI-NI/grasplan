# grasplan

## Open-set objects

Open-set objects are unknown by design: grasplan must cope with any new object from what the robot perceives (point
cloud, AnyGrasp, gripper feedback). Never put per-object knowledge of open-set objects (dimensions, shapes, diameters,
class-specific tables) into grasplan's code or configs; such measurements belong only in tests and ground-truth
evaluation.
