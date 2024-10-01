import os

print("set_env_vars.py script is being executed")

def set_env_vars(*args, **kwargs):
    print("Executing set_env_vars.py")
    platformio_home = os.getenv("PLATFORMIO_HOME_DIR")
    print(f"PLATFORMIO_HOME_DIR: {platformio_home}")
    
    if platformio_home:
        idf_path = os.path.join(platformio_home, "packages", "framework-espidf")
        toolchain_path = os.path.join(platformio_home, "packages", "toolchain-xtensa-esp-elf", "bin")
        
        os.environ["IDF_PATH"] = idf_path
        os.environ["PATH"] = toolchain_path + os.pathsep + os.environ["PATH"]
        
        print(f"IDF_PATH set to: {idf_path}")
        print(f"PATH updated to include: {toolchain_path}")
    else:
        print("PLATFORMIO_HOME_DIR is not set")