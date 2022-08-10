idf_component_register(SRCS "OutputVelocity.cpp" "OutputVelocitySim.cpp" "OutputVelocityImpl.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES RosMsgs MotorController)
