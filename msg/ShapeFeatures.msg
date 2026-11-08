# Generic shape features of one object, measured from its point cloud (grasplan/shape_features.py). Metres and
# ratios; NaN = not measured (the points could not support it); the tri-state fields are UNKNOWN / NO / YES.
# No field names an object: this is geometry only.
int8 UNKNOWN=-1
int8 NO=0
int8 YES=1

bool valid                # false: not computed (an older publisher, too few points); all other fields are then unset
bool outline_reliable     # the horizontal outline (round / box-like, diameter, footprint) can be trusted
bool height_reliable      # the height can be trusted (not filled in from completed depth)
uint32 points             # points the features were measured from
string shape              # cylinder, sphere, lying_cylinder, box or unknown
float32 height            # above the support plane
float32 diameter          # round shapes
float32 footprint_min
float32 footprint_max
float32[] radii           # per height slice (25, 50, 75 %), NaN where not round
float32 circle_residual   # rms residual of the circle fit / radius
float32 arc_span_deg      # visible arc of the circle fit
float32 rect_residual     # rms distance to the fitted rectangle (m)
int8 round_outline
int8 tall
int8 flat
int8 elongated
int8 graspable_across
