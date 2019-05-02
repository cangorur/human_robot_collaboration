# To compile
'''
g++ -o track_markers track_markers.cpp -L/opt/ros/kinetic/lib/x86_64-linux-gnu `pkg-config --libs --cflags opencv-3.3.1-dev`
'''
In most of the installaitons opencv package has been found with the name "opencv" instead of "opencv-3.3.1-dev". 

# To run:
'''
./track_markers -d=4 -l=0.05 --refine=2 --ci=0
'''
change ci for different cameras. For ubuntu embedded it is 0 , for lab PC W530 it is 1 

# To run as a ROS node 
rosrun hrc_ros HeadTracking_agent -d=4 -l=0.05 --refine=2 --ci=1 --imgshow=0 --printdebug=0

# Degubbing 
The node can be debugged by setting --printdebug=1 
* head_gesture status will be printed 
* If you run **rosrun sound_play soundplay_node.py** a different sound will be played when notlooking around and lookingaround is detected. You can now turn around your head and see at which angles the looking around will be detected. 

