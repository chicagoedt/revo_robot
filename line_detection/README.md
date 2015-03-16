# README for line_detection package

## Summary
This package contains a bunch of computer vision nodes that use OpenCV and rospy. Each node is a subclass of a superclass called LaneDetection, which initializes publishers and subscribers, as well as dynamic_reconfigure callbacks, and converts ROS to CV2 and vice versa.

## How to run
Most of the nodes can be run using rosrun with the default parameters. However, fitline, skeletonize, brightest-pixel, and gabor nodes currently need a launch file to run (the use_mono parameter has to be set to true), and they also need input images to be in mono format. You can use the provided launch files for those special nodes.
For example, to run dilate node: `rosrun line_detection dilate.py`

## How to create a new node
Just use generic.py as a template. Replace the word "generic" with whatever name your node is (watch the underscores or uppercase letters!). Add some code to the middle of `image_callback` to make your node do stuff (`roi` is your input image, and `final_image` is your output image).

## Node Chaining
Multiple nodes can be chained in launch files (their primary purpose is to test different combinations of filters to help find the best algorithm for detecting white lines on grass).

## Launch File Parameters
Check the generic.launch and stereo-generic.launch files for the default parameters.

## Pitfalls (watch out! D:)
If you've used and built an older version of `line_detection`, you might have trouble with the `dynamic_reconfigure` node crashing because of some `cfg` not found error. Unfortunately, it seems this problem persists until you remove your entire workspace and start fresh (perhaps even have to reinstall `ROS` completely [?]). It has to do with some weird cfg filename changes persisting even after attempting clean builds.
