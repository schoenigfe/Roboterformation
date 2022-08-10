idf_component_register(SRCS "StateMachine.cpp" "States.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES RosMsgs OutputVelocity PositionController)
