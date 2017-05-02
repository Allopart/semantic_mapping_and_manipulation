^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_state_publisher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.13.0 (2016-04-12)
-------------------
* fix bad rebase
* Contributors: Jackie Kay, Paul Bovbel

1.12.1 (2016-02-22)
-------------------
* Merge pull request `#42 <https://github.com/ros/robot_state_publisher/issues/42>`_ from ros/fix_tests_jade
  Fix tests for Jade
* Correct failing tests
* Re-enabling rostests
* Merge pull request `#39 <https://github.com/ros/robot_state_publisher/issues/39>`_ from scpeters/issue_38
* Fix API break in publishFixedTransforms
  A bool argument was added to
  RobotStatePublisher::publishFixedTransforms
  which broke API.
  I've added a default value of false, to match
  the default specified in the JointStateListener
  constructor.
* Contributors: Jackie Kay, Jonathan Bohren, Steven Peters

1.12.0 (2015-10-21)
-------------------
* Merge pull request `#37 <https://github.com/ros/robot_state_publisher/issues/37>`_ from clearpathrobotics/static-default
  Publish fixed joints over tf_static by default
* Merge pull request `#34 <https://github.com/ros/robot_state_publisher/issues/34>`_ from ros/tf2-static-jade
  Port to tf2 and enable using static broadcaster
* Merge pull request `#32 <https://github.com/ros/robot_state_publisher/issues/32>`_ from `shadow-robot/fix_issue#19 <https://github.com/shadow-robot/fix_issue/issues/19>`_
  Check URDF to distinguish fixed joints from floating joints. Floating joint are ignored by the publisher.
* Merge pull request `#26 <https://github.com/ros/robot_state_publisher/issues/26>`_ from xqms/remove-debug
  get rid of argv[0] debug output on startup
* Contributors: David Lu!!, Ioan A Sucan, Jackie Kay, Max Schwarz, Paul Bovbel, Toni Oliver

1.11.1 (2016-02-22)
-------------------
* Merge pull request `#41 <https://github.com/ros/robot_state_publisher/issues/41>`_ from ros/fix_tests_indigo
  Re-enable and clean up rostests
* Correct failing tests
* Re-enabling rostests
* Fix API break in publishFixedTransforms
  A bool argument was added to
  RobotStatePublisher::publishFixedTransforms
  which broke API.
  I've added a default value of false, to match
  the default specified in the JointStateListener
  constructor.
* Contributors: Jackie Kay, Jonathan Bohren, Steven Peters

1.11.0 (2015-10-21)
-------------------
* Merge pull request `#28 <https://github.com/ros/robot_state_publisher/issues/28>`_ from clearpathrobotics/tf2-static

1.10.4 (2014-11-30)
-------------------
* Merge pull request `#21 <https://github.com/ros/robot_state_publisher/issues/21>`_ from rcodddow/patch-1
* Fix for joint transforms not being published anymore after a clock reset (e.g. when playing a bagfile and looping)
* Contributors: Ioan A Sucan, Robert Codd-Downey, Timm Linder

1.10.3 (2014-07-24)
-------------------
* add version depend on orocos_kdl >= 1.3.0
  Conflicts:
  package.xml
* Update KDL SegmentMap interface to optionally use shared pointers
  The KDL Tree API optionally uses shared pointers on platforms where
  the STL containers don't support incomplete types.
* Contributors: Brian Jensen, William Woodall

1.10.0 (2014-03-03)
-------------------
* minor style fixes
* Add support for mimic tag.
* Contributors: Ioan Sucan, Konrad Banachowicz
