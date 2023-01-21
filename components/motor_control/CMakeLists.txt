set(COMPONENT_ADD_INCLUDEDIRS
            "../../main"    
            "../../main/include"    
            "../../components/motor_control/include"
            "../../components/motor_control"
            "../../components/sbus_component/include"
            "../../components/sbus_component"
            "../../components/pca9685/include"
            "../../components/pca9685"
            "../../components/my_i2c_interface/include"
            "../../components/my_i2c_interface"
    )
    
set(COMPONENT_SRCS 
    motor_control_task.cpp
    motor_control.cpp
    )

register_component()