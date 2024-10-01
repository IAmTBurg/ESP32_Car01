# ./src/CMakeLists.txt

idf_component_register(SRCS "main.cpp" "Task0.cpp" "Task1.cpp" "Tasks.cpp"
    INCLUDE_DIRS 
        "../include"
    REQUIRES 
        motor_control
        my_i2c_interface
        pca9685
        sbus_component
        freertos 
        esp_common
        driver
        esp_event
        # esp_log
)