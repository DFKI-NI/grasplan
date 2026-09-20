# Outcome of a verifier (typically a VLM) for one candidate object.
# Only MATCH and NO_MATCH are semantic verdicts; every other non-zero status is
# a verifier failure and must be treated as "unknown", never as a negative.
uint8 NONE=0          # the verifier did not run
uint8 MATCH=1
uint8 NO_MATCH=2
uint8 TIMEOUT=3       # the model did not answer within the verifier's timeout
uint8 HTTP_ERROR=4    # proxy / model endpoint returned an HTTP error
uint8 NO_CONTENT=5    # the model answered without content (e.g. reasoning budget exhausted)
uint8 BAD_JSON=6      # the answer held no usable JSON verdict
uint8 UNAVAILABLE=7   # the verifier service itself was not reachable
uint8 ERROR=8         # any other failure (see error)

uint8 status
float32 confidence            # [0, 1], 0 unless status is MATCH or NO_MATCH
string label                  # what the verifier thinks it sees (MATCH / NO_MATCH)
string rationale
string error                  # failure detail for status >= TIMEOUT
float32 elapsed_s
int32 best_view               # multi-view: 1-based number of the view (as captioned) that showed the object best, 0 unknown
string best_detection_id      # multi-view: the detection of that view, filled by the detector node; may be empty
string geometry_detection_id  # detection whose SAM2 box was used for the 3-D result (best depth support), may be empty
uint32 geometry_points        # valid depth points of that box
uint8 retries                 # verifier retries that were made for this verdict
