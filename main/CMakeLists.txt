idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES SensorPose Wifi RosBridgeClient RosMsgs StateMachine OutputVelocity MotorController PositionController LedStrip DataLogger nvs_flash)
