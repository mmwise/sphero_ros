## Installation
Installation is now based on catkin:

      cd <ws>/src
      git clone https://github.com/mmwise/sphero_ros
      cd <ws>
      catkin_make
      catkin_make install
      source <ws>/install/setup.bash
      ...

## Notes

 * Updated to use catkin, on ros-groovy.
 * Compiles and installs, not tested on an actual robot.
 * Changes Made:
   * sphere_node.py is now sphero.py -- the old name conflicts with the package name and this just doesn't work with an out of source build. If we want the node name to go back to sphero_node.py, then messages need to be moved into a separate package (perhaps sphero_msgs).
   * Source files in sphero_driver have been pushed into src/sphero_driver and an __init__.py  has been added to simplify installation.
 * TODO:
   * Update run_depends on sphero_driver so that system depends are installed.
   * Rename master to groovy-devel (is easier for releases):
     * rename master to groovy-devel: git branch -m master groovy-devel
     * push it to github: git push -u origin groovy-devel
     * go to github.com, change default branch to groovy-devel under settings
     * remove master branch: git push origin :master
   * Merge mikeferguson fork into groovy-devel branch
