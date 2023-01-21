set(INCLUDEDIR_LIST
            "../../main"    
            "../../main/include"    
            "../../components/motor_control/include"
            "../../components/motor_control"
            "../../components/sbus_component/include"
            "../../components/sbus_component"
            "../../components/gpio_component/include"
            "../../components/gpio_component"
            "../../components/pca9685/include"
            "../../components/pca9685"
            "../../components/my_i2c_interface/include"
            "../../components/my_i2c_interface"
    )


set(SOURCES 
        main.cpp
        tasks.cpp
        task0.cpp
        task1.cpp  
        )

idf_component_register(SRCS ${SOURCES}
        INCLUDE_DIRS  
			. 
			./include 
            "../components/motor_control/include"
            "../components/motor_control"
            "../components/sbus_component/include"
            "../components/sbus_component"
            "../components/pca9685/include"
            "../components/pca9685"
            "../components/my_i2c_interface/include"
            "../components/my_i2c_interface"

        REQUIRES freertos esp_common
        )

# REQUIRES "driver" "esp_event")