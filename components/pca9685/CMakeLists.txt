# ./components/pca9685/CMakeLists.txt

idf_component_register(
    SRCS 
        "pca9685.cpp"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        driver
        esp_event
        # esp_log
)