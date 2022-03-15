
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
add_executable(diff-drive-Kinematics_ex diff-drive-Kinematics_ex.cpp)
target_link_libraries(diff-drive-Kinematics_ex 
					${PYTHON_LIBRARIES}
					Eigen3::Eigen) 
