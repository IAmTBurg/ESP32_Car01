# ./components/motor_control/CMakeLists.txt

idf_component_register(
    SRCS 
        "motor_control_task.cpp"
        "motor_control.cpp"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        driver
        esp_event
        # esp_log
)