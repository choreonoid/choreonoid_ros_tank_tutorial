# Step 1
choreonoid_add_simple_controller(RttTankController RttTankController.cpp)
target_link_libraries(RttTankController ${roscpp_LIBRARIES})

# Step 2
choreonoid_add_simple_controller(RttJointStatePublisher RttJointStatePublisher.cpp)
target_link_libraries(RttJointStatePublisher ${roscpp_LIBRARIES})

# Step 3
choreonoid_add_simple_controller(RttJointStateSubscriber RttJointStateSubscriber.cpp)
target_link_libraries(RttJointStateSubscriber ${roscpp_LIBRARIES} Choreonoid::CnoidBodyPlugin)

# Step 4
choreonoid_add_simple_controller(RttImuStatePublisher RttImuStatePublisher.cpp)
target_link_libraries(RttImuStatePublisher ${roscpp_LIBRARIES})

# Step 5
choreonoid_add_simple_controller(RttImuStateSubscriber RttImuStateSubscriber.cpp)
target_link_libraries(RttImuStateSubscriber ${roscpp_LIBRARIES} Choreonoid::CnoidBodyPlugin)

# Step 6
choreonoid_add_simple_controller(RttCameraImagePublisher RttCameraImagePublisher.cpp)
target_link_libraries(RttCameraImagePublisher ${roscpp_LIBRARIES} ${image_transport_LIBRARIES})

#choreonoid_add_simple_controller(Rtt3dSensorImagePublisher Rtt3dSensorImagePublisher.cpp)
#target_link_libraries(Rtt3dSensorImagePublisher ${roscpp_LIBRARIES} ${image_transport_LIBRARIES})

#choreonoid_add_simple_controller(Rtt3dSensorImageSubscriber Rtt3dSensorImageSubscriber.cpp)
#target_link_libraries(Rtt3dSensorImageSubscriber ${roscpp_LIBRARIES} ${image_transport_LIBRARIES})
