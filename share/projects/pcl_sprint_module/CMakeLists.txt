set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(PCL 1.3 COMPONENTS common io visualization segmentation sample_consensus features filters)
if(PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})

	    add_library(mlr_pcl plane.cpp)
    target_link_libraries(mlr_pcl
        LINK_PRIVATE
        Core
        Hardware_kinect
	System
	Perception
        ${PCL_COMMON_LIBRARIES}
        ${PCL_IO_LIBRARIES}
        ${PCL_VISUALIZATION_LIBRARIES}
        ${ROS_PCL_LIBRARIES}
        ${PCL_SEGMENTATION_LIBRARY}
        ${PCL_FEATURES_LIBRARY}
        ${PCL_FILTERS_LIBRARY}
        ${PCL_SAMPLE_CONSENSUS_LIBRARY}
    )

    add_executable(pcl_p_test main.cpp)
    target_link_libraries(pcl_p_test mlr_pcl)

    #add_executable(viewer.exe viewer.cpp test_method.cpp)
    #target_link_libraries(viewer.exe ${PCL_LIBRARIES})
endif()


