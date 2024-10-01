# ./components/sbus_component/CMakeLists.txt

idf_component_register(
    SRCS 
        "sbus_component.cpp"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        driver
        esp_event
        # esp_log
)