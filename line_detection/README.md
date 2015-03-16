README for line_detection package

Summary
This package contains a bunch of computer vision nodes that use OpenCV and rospy. Each node is a subclass of a superclass called LaneDetection, which initializes publishers and subscribers, as well as dynamic_reconfigure callbacks, and converts ROS to CV2 and vice versa.

Multiple nodes can be chained in launch files (their primary purpose is to test different combinations of filters to help find the best algorithm for detecting white lines on grass).

Most of the nodes can be run using rosrun with the default parameters. However, fitline, skeletonize, brightest-pixel, and gabor nodes currently need a launch file to run (the use_mono parameter has to be set to true), and they also need input images to be in mono format.

Check the generic.launch and stereo-generic.launch files for the default parameters.