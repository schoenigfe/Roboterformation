idf_component_register(SRCS "Marvelmind.cpp" "SensorPoseSim.cpp" "KalmanFilter.cpp" 
                    INCLUDE_DIRS "include"
                    REQUIRES RosBridgeClient OutputVelocity esp-dsp)
