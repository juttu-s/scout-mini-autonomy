# One tracked object, in meters. Position and velocity are expressed in the header frame of the parent
# TrackedObjectArray: ~output_frame (e.g. base_link), or camera_color_optical_frame (x right, y down,
# z forward) by default. Velocity is the object's own motion measured in ~velocity_frame (e.g. odom, so the
# robot's motion is removed); with the default camera frame it includes the camera's motion.
# size is width, height, depth extent along the camera axes.
uint32 id                       # persistent id; with re-identification it survives leaving the view.
                                # 0 = new track still being compared with lost identities (< ~0.5 s)
int64 class_id                  # COCO class index; the track's score-weighted majority over its life
string label                    # class name
float32 score                   # detector confidence in the current frame
vision_msgs/BoundingBox2D bbox  # pixel box in the color image
bool has_position               # false when there was not enough valid depth this frame
geometry_msgs/Point position    # median depth in the box center, deprojected
geometry_msgs/Vector3 velocity  # m/s, line fit over the last ~0.5 s; zero until enough samples
geometry_msgs/Vector3 size      # approximate width, height, depth extent
float32 age                     # seconds since the track was first seen
