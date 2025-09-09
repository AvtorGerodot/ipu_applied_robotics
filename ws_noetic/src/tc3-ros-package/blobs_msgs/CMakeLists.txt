cmake_minimum_required(VERSION 3.0.2)
project(blobs_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs)

add_message_files( FILES Blob.msg BlobArray.msg )
generate_messages( DEPENDENCIES std_msgs geometry_msgs )

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    geometry_msgs
    std_msgs
)



