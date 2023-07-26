^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package um7
^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2023-07-26)
------------------
* Added boost as a build_depend.
* Contributors: Tony Baltovski

1.0.0 (2023-07-25)
------------------
* Reverted versioning to continue where it left off
* Add instructions and update log for ros 2 version
* Fixed inclusion of serial library in CI
* Updated CI setup-ros tooling version
* Add github ci
* Setup custom tests
* Linting changes
* Updated test files
* Updating for ROS 2 Linting
* Updating to hpp file naming for linting
* Added UM6_driver
* Optimized for speed
* Update structure for separateUM6 and 7 nodes
* More detailed error message
* Setup for ROS 2 and UM7
* folder structure for new executable name
* folder structure for new package name
* Contributors: Hilary Luo

0.0.7 (2022-04-13)
------------------
* Add output in robot frame option (`#26 <https://github.com/ros-drivers/um7/issues/26>`_)
  * Correctly convert quaternion to ROS frame
  * Rename tf_ned_to_enu to tf_ned_to_nwu
  * Cleanup
  * Update to handle two options
  * lint
  * Change indentation
  * Fix formatting issues + add namespace to enum
* Contributors: Bianca Homberg

0.0.6 (2019-10-30)
------------------
* Fix error of mag topic not publish
* Contributors: Nicolas SIMON

0.0.5 (2019-09-27)
------------------
* Updated to be able to use MagneticField message.
* Added TravisCI badge to README.
* Updated TravisCI for Kinetic and Melodic.
* Fixed linter errors.
* Contributors: Tony Baltovski

0.0.2 (2015-02-20)
------------------
Initial release of um7 driver.
