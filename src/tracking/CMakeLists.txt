find_package(Cpd REQUIRED)

#add_definitions("-DEIGEN_DONT_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT")

include_directories(
  ${BULLETSIM_SOURCE_DIR}/src
  ${PCL_INCLUDE_DIRS}
  ${TETGEN_DIR}
  ${ROS_ROOT_DIR}/include
  /home/chicheng/ARMLab/cpd_ws/devel/include
)

link_directories(
  ${TETGEN_DIR}
  ${ROS_ROOT_DIR}/lib
)

add_library(tracking 
	algorithm_common.cpp 
	config_tracking.cpp
	initialization.cpp
	plotting_tracking.cpp
	simple_physics_tracker.cpp 
	physics_tracker.cpp 
	surface_sampling.cpp 
	sparse_utils.cpp
	tracked_object.cpp 
	tracked_rope.cpp
	tracked_cloth.cpp
	tracked_towel.cpp
	tracked_sponge.cpp
	tracked_box.cpp
	utils_tracking.cpp
	visibility.cpp
	feature_extractor.cpp
	tracked_compound.cpp
        cam_sync.cpp
)

target_link_libraries(tracking simulation clouds cv_bridge  Cpd::Library-C++)
	
add_executable(tracker_node tracker_node.cpp)
target_link_libraries(tracker_node tracking simulation clouds tf cv_bridge ${OpenCV_LIBS} ${catkin_LIBRARIES} boost_signals boost_program_options)

add_executable(tracker_node_CPD tracker_node_CPD.cpp)
target_link_libraries(tracker_node_CPD tracking simulation clouds tf cv_bridge ${OpenCV_LIBS} ${catkin_LIBRARIES} boost_signals boost_program_options)

#add_executable(simulated_tracker simulated_tracker.cpp)
#target_link_libraries(simulated_tracker tracking simulation clouds tf cv_bridge ${OpenCV_LIBS} ${catkin_LIBRARIES} boost_signals boost_program_options)

#add_executable(simulated_hand_tracker simulated_hand_tracker.cpp)
#target_link_libraries(simulated_hand_tracker tracking simulation clouds tf cv_bridge ${OpenCV_LIBS} ${catkin_LIBRARIES} boost_signals boost_program_options)

#add_executable(hand_tracker_node hand_tracker_node.cpp)
#target_link_libraries(hand_tracker_node tracking simulation clouds tf cv_bridge ${OpenCV_LIBS} ${catkin_LIBRARIES} boost_signals boost_program_options)


