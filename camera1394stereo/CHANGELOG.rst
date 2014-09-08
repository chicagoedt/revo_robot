^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package camera1394stereo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.0.1 (2013-07-26)
------------------

* added libdc1394-22 dependency
* Fix `#1 <https://github.com/srv/camera1394stereo/issues/1>`_: pachage was always compiling due to a broken dependency

1.0.0 (2013-04-08)
------------------
* working catkinization of package. Prepared for release
* added package.xml
* catkinization of package
* Merge branch 'develop'
* Merge branch 'develop'
* Merge branch 'develop' of srv.uib.es:uib-ros/camera1394stereo into develop
* Fixed documentation comments concerning published topics.
* Minor fixes in output messages.
* Fixed name bug in nodelet class.
* Fixed camera_info_manager:: namespace warning for Electric.
* Publishing namespace changed to stereo_camera in nodelet.cpp
* Added launch files to test nodelet version
* Fixed white space typo
* Fixed color coding parameter name in launch files
* Merge branch 'develop' of srv.uib.es:uib-ros/camera1394stereo into develop
* Fixed bug wrong service name: single_nh_ now inherits namespace from main camera_nh_
* Merge branch 'develop' of srv.uib.es:uib-ros/camera1394stereo into develop
* adapted launch file to be more general
* Added rosdep libdc1394-utils
* Added rosdep.yaml for libdc1394 dependencies
* Merge branch 'develop'
* Removed from .gitignore Makefile in package root
* Upgrade to version for diamondback
* Initial commit (stable version for cturtle)
