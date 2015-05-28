# README for line_detection package

## Summary
This package contains a bunch of computer vision nodes that use OpenCV and rospy. Each node is a subclass of a superclass called LaneDetection, which initializes publishers and subscribers, as well as dynamic_reconfigure callbacks, and converts ROS to CV2 and vice versa. All the nodes subscribe to a CompressedImage topic and publish to a CompressedImage topic. Some nodes require either grayscale or RGB images (backproject needs RGB. fitline, brightest-pixel, and skeletonize need grayscale).

## How to run
For example, to run dilate node: `rosrun line_detection dilate.py`

## How to create a new node
Just use generic.py as a template. Replace the word "generic" with whatever name your node is (watch the underscores or uppercase letters!). Add some code to the middle of `image_callback` to make your node do stuff (`roi` is your input image, and `final_image` is your output image). Use the `convert_to_mono` function if necessary.

## Node Chaining
Multiple nodes can be chained in launch files (their primary purpose is to test different combinations of filters to help find the best algorithm for detecting white lines on grass).

## Launch File Parameters
Check the generic.launch and stereo-generic.launch files for the default parameters.

## Pitfalls (watch out! D:)
If you've used and built very old versions (before LaneDetection class) of `line_detection` then tried to use the newer versions, you might have trouble with the `dynamic_reconfigure` node crashing because of some `cfg` not found error. Unfortunately, it seems this problem persists until you remove your entire workspace and start fresh (perhaps even have to reinstall `ROS` completely [?]). It has to do with some weird cfg filename changes persisting even after attempting clean builds. Just stay away from the old versions.

## Dependencies
 - `pixel\_to\_coordinate\_calculator` node:
     - `image_geometry` has to be installed and compiled from source. Check [this issue](http://answers.ros.org/question/209953/image_geometry-pinholecameramodel-python-not-importing-properly-when-installed-using-debian-package/) for more details.
<!--      - [scipy](http://scipy.org/install.html) has to be installed from debian packages. -->