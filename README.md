# bigredrobot_proj2

For workspace velocity line following and spline drawing:
    in a baxter shell (source baxter.sh):
        roslaunch bigredrobot_proj2 line_follow.launch
 
For c-space path planning using RRT-Connect and c-space velocity control path following:
    in a baxter shell:
        roslaunch bigredrobot_proj2 collision_viz.launch
    in a new terminal with baxter.sh sourced:
        roslaunch bigredrobot_proj2 path_follow.launch
