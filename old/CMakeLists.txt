cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(plasmaPico)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project

# spa_test
add_executable(spa_test
        spa_test.c
        )

# pull in common dependencies
target_link_libraries(spa_test pico_stdlib
        hardware_pio
        hardware_pwm
        )

# create map/bin/hex file etc.
pico_add_extra_outputs(spa_test)

# spa_toggle.pio.h
add_executable(spa_toggle
        spa_toggle.pio.h
        )

# pull in common dependencies
target_link_libraries(spa_toggle pico_stdlib
        hardware_pio)

# create map/bin/hex file etc.
pico_add_extra_outputs(spa_toggle)