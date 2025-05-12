# Defines a 3D detection result.
#
# This extends a basic 3D classification by including the pose of the
# detected object.

std_msgs/Header header

# Class probabilities. Does not have to include hypotheses for all possible
#   object ids, the scores for any ids not listed are assumed to be 0.
ObjectHypothesisWithPose[] results

# 3D bounding box surrounding the object.
BoundingBox3D bbox

# ID used for consistency across multiple detection messages. Detections
# of the same object in different detection messages should have the same id.
# This field may be empty.
string id

### New / old design decision
### 
# Source data that generated this classification are not a part of the message.
# If you need to access them, use an exact or approximate time synchronizer in
# your code, as this message's header should match the header of the source
# data.


### Putting the field back in for our use case
# The 3D data that generated these results (i.e. region proposal cropped out of
#   the image). This information is not required for all detectors, so it may
#   be empty.
sensor_msgs/PointCloud2 source_cloud
