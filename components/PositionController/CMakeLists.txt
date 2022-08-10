idf_component_register(SRCS "ControllerMaster.cpp" "p2pController.cpp" "approxLinController.cpp" "statInOutLinController.cpp" "dynInOutLinController.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES RosMsgs OutputVelocity SensorPose DataLogger) 
