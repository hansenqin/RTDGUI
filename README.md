
# Usage Instructions

First modify the ip address of the ROS MASTER in RTD_GUI.m line 18. Launch RTDGUI.m and the GUI now should be ready to visualize data. Use the checkbox window to control the desired data to be visualized. Currently, supported topics along with their types are:

|Topic                     |Types                                    |
| :----------------------- |-----------------:                       |
|/vesc/odom                |nav_msgs/Odometry                        |
|/obstacles                |obstacle_detector/Obstacles              |
|/sPath                    |nav_msgs/Path                            |
|/initialpose              |geometry_msgs/PoseWithCovarianceStamped  |
|/move_base_simple/goal    |geometry_msgs/PoseStamped                |
|/zonotope_visualization   |jsk_recognition_msgs/PolygonArray        |
|/tf                       | tf2_msgs/TFMessage                      |

# Controls
w key: move camera up 
s key: move camera down 

