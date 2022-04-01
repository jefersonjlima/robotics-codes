
# rotation
add_executable(rotation_ex rotation_ex.cpp)
target_link_libraries(rotation_ex Eigen3::Eigen)

# translation
add_executable(translation_ex translation_ex.cpp)
target_link_libraries(translation_ex Eigen3::Eigen) 

# HT operator
add_executable(hTransformation_ex hTransformation_ex.cpp)
target_link_libraries(hTransformation_ex Eigen3::Eigen) 

# Differential Drive Kinematics
add_executable(diff_drive_kinematics_ex ../../robocore.cpp diff_drive_kinematics_ex.cpp)
if(WITH_PLOT_GRAPH)
 target_link_libraries(diff_drive_kinematics_ex ${PYTHON_LIBRARIES})
endif()

# DC Motor Simulation
add_executable(dc_motor_ex ../../robocore.cpp dc_motor_ex.cpp)
if(WITH_PLOT_GRAPH)
 target_link_libraries(dc_motor_ex ${PYTHON_LIBRARIES})
endif()
