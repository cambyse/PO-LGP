set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(PCL 1.3 COMPONENTS common io visualization segmentation sample_consensus features filters)
if(PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})

    if(WITH_ROS)
        pkg_check_modules(ROS_PCL roscpp sensor_msgs)
        if(ROS_PCL_FOUND)
            add_definitions(-DHAVE_ROS_PCL ${ROS_PCL_CFLAGS_OTHER})
            include_directories(${ROS_PCL_INCLUDE_DIRS})
            link_directories(${ROS_PCL_LIBRARY_DIRS})
        else()
            unset(ROS_PCL_LIBRARIES CACHE)
        endif()
    endif()

    add_library(mlr_pcl grab.cpp plane.cpp)
    target_link_libraries(mlr_pcl
        LINK_PRIVATE
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
endif()

add_executable(viewer viewer.cpp)
target_link_libraries(viewer ${PCL_LIBRARIES})
