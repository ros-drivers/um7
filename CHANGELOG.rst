^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package um7
^^^^^^^^^^^^^^^^^^^^^^^^^

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
