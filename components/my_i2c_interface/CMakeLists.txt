# ./components/my_i2c_interface/CMakeLists.txt

idf_component_register(
    SRCS 
        "my_i2c_interface.cpp"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        driver
        esp_event
        # esp_log
)