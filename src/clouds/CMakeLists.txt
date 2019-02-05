include_directories(
  ${PCL_INCLUDE_DIRS}
  ${BULLET_INCLUDE_DIR}
  ${JSON_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${BULLETSIM_SOURCE_DIR}/src
  ${BULLET_DIR}/src
  ${ROS_ROOT_DIR}/include
  /home/chicheng/ARMLab/cpd_ws/devel/include
)

link_directories(
  ${PCL_LIBRARY_DIRS}   
  ${ROS_ROOT_DIR}/lib
)

add_definitions("-DEIGEN_DEFAULT_TO_ROW_MAJOR")

add_library(clouds geom.cpp get_table2.cpp get_chessboard_pose.cpp plane_finding.cpp utils_pcl.cpp cloud_ops.cpp utils_cv.cpp utils_ros.cpp table.cpp grabcut.cpp)
target_link_libraries(clouds utils ${PCL_LIBRARIES} ${OpenCV_LIBS} ${BULLET_LIBS} boost_signals ${Boost_PROGRAM_OPTIONS_LIBRARY})

#add_executable(preprocessor_node preprocessor_node.cpp)
#target_link_libraries(preprocessor_node clouds utils ${PCL_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES} tf cv_bridge boost_program_options)

add_executable(simple_preprocessor_node simple_preprocessor_node.cpp)
target_link_libraries(simple_preprocessor_node clouds utils ${PCL_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES} tf cv_bridge boost_program_options)

add_executable(preprocessor_color_node preprocessor_color_node.cpp)
target_link_libraries(preprocessor_color_node clouds utils ${PCL_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES} tf cv_bridge boost_program_options)

#add_executable(preprocessor_depth_node preprocessor_depth_node.cpp)
#target_link_libraries(preprocessor_depth_node clouds utils ${PCL_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES} tf cv_bridge image_transport boost_program_options)

add_executable(preprocessor_segmentation_node preprocessor_segmentation_node.cpp)
target_link_libraries(preprocessor_segmentation_node clouds utils ${PCL_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES} tf cv_bridge image_transport boost_program_options)

add_executable(cloud_merger_node cloud_merger_node.cpp)
target_link_libraries(cloud_merger_node clouds utils ${PCL_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES} tf cv_bridge boost_program_options)

add_executable(kinect_calibrate_node kinect_calibrate_node.cpp)
target_link_libraries(kinect_calibrate_node clouds utils ${PCL_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES} tf cv_bridge boost_program_options)

add_executable(kinect_alignment_node kinect_alignment_node.cpp)
target_link_libraries(kinect_alignment_node clouds utils ${PCL_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES} tf cv_bridge boost_program_options)

add_executable(save_cloud_node save_cloud_node.cpp)
target_link_libraries(save_cloud_node utils ${PCL_LIBRARIES} ${catkin_LIBRARIES} tf boost_program_options)

add_executable(viz_curves viz_curves.cpp)
target_link_libraries(viz_curves ${PCL_LIBRARIES} clouds boost_program_options utils)

add_executable(get_table_server get_table_server.cpp)
target_link_libraries(get_table_server clouds ${PCL_LIBRARIES} ${catkin_LIBRARIES} utils boost_program_options)

add_executable(cloud_adjuster_node sandbox/cloud_adjuster_node.cpp)
target_link_libraries(cloud_adjuster_node clouds utils ${PCL_LIBRARIES} ${catkin_LIBRARIES} boost_program_options)

add_executable(define_boundary define_boundary.cpp)
target_link_libraries(define_boundary clouds utils ${PCL_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES} tf cv_bridge boost_program_options)

