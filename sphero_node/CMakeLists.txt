cmake_minimum_required(VERSION 2.8.3)
project(sphero_node)

find_package(catkin REQUIRED message_generation dynamic_reconfigure std_msgs)

add_message_files(FILES
                  SpheroCollision.msg
                 )

generate_messages(DEPENDENCIES std_msgs)
generate_dynamic_reconfigure_options(cfg/Reconfig.cfg)

catkin_package(DEPENDS std_msgs dynamic_reconfigure message_runtime)

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS nodes/sphero.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
