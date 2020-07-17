cmake_minimum_required(VERSION 3.0.2)
project(people_msgs)
find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs)

add_message_files(
   DIRECTORY msg
   FILES
    People.msg
    Person.msg
    PersonStamped.msg
    PositionMeasurement.msg
    PositionMeasurementArray.msg
)

generate_messages(
   DEPENDENCIES   geometry_msgs std_msgs
)

catkin_package(
    CATKIN_DEPENDS geometry_msgs message_runtime std_msgs
)
