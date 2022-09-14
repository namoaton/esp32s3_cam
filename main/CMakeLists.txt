set(COMPONENT_SRCS  take_photo_main.cc
                    main_functions.cc 
                    esp/image_provider.cc 
                    esp/app_camera_esp.c                    
                    sdcard_mount.cc
                    button.cc
                    src/app_button.cpp
                    src/app_lcd.cpp
                    )


set(COMPONENT_ADD_INCLUDEDIRS "." "include")

set(requires    console
esp_adc_cal)
register_component()

# idf_component_register(SRC_MAIN_DIRS ${src_dirs} INCLUDE_MAIN_DIRS ${include_dirs})
