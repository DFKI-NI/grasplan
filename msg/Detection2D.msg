# One open-set 2-D detection (Grounding DINO box refined by SAM2).
std_msgs/Header header        # header of the image the detection was made on
string label                  # detector label (the prompt token(s) that matched)
float32 score                 # detector confidence, not calibrated
float32[4] bbox_xyxy          # pixel box: x_min, y_min, x_max, y_max
sensor_msgs/Image mask        # mono8 instance mask (255 = object), may be empty
bool accepted                 # passed the threshold/margin rule (and the verifier, if it ran)
bool verified                 # true when a verifier (e.g. VLM) confirmed the label
string verifier_label         # what the verifier thinks the object is, if it ran
float32 verifier_confidence   # verifier confidence in [0, 1], 0 when it did not run
