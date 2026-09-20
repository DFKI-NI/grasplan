# One open-set 2-D detection (Grounding DINO box refined by SAM2).
std_msgs/Header header        # header of the image the detection was made on
string detection_id           # "<view_id>:<index>", stable while the view stays in the node's cache
uint32 view_id                # the captured frame this detection belongs to
string label                  # detector label (the prompt token(s) that matched)
string[] matched_phrases      # other phrasings whose near-identical boxes were merged into this one before top-N
float32 score                 # detector confidence, not calibrated
float32[4] bbox_xyxy          # pixel box: x_min, y_min, x_max, y_max
sensor_msgs/Image mask        # mono8 instance mask (255 = object), may be empty
bool has_position             # position below is valid
geometry_msgs/Point position  # coarse map-frame position from the depth inside the box (no SAM2 needed)
bool accepted                 # passed the threshold/margin rule (and the verifier, if it ran)
bool verified                 # true when a verifier (e.g. VLM) confirmed the label (verifier_status == MATCH)
uint8 verifier_status         # grasplan/Verdict status constant; failures are never negatives
string verifier_label         # what the verifier thinks the object is, if it ran
float32 verifier_confidence   # verifier confidence in [0, 1], 0 when it did not run
string verifier_error         # failure detail when verifier_status >= TIMEOUT
