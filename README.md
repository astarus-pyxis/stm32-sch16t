# STM32-SCH16T

## C driver to interface SCH16T IMU with an STM32 microcontroller

The driver itself is made of the files sch16t.c and sch16t.h.

It requires the files errors.h, console.h and console.c, which are common files for all my drivers. They are used to set up the error type (in errors.h) returned by some of the functions of the driver and to display data with the microcontroller on a terminal (in console.h and console.c).

The file main.c is an example of main that uses the driver.

## How to use this driver in a project

To use this driver in an STM32 CMake project, the SPI peripheral for the sensor shall be configured by the user, with CubeMX or directly in the code. This is not done by the driver.

The C files  sch16t.c and console.c shall be placed in the Core > Src folder of the project, and sch16t.h, errors.h and console.h in the Core > Inc folder.

It also requires to add the sources to executable in the CMakeLists.txt file at the root of the project. To do this, the following at line 48 of this file


```
# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

```

shall be changed to


```
# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    "Core/Src/console.c"
    "Core/Src/sch16t.c"
)
```

## Licence & Warranty

This driver is licensed under GNU V3.0. It comes with no warranty.

Please note that this is a minimal driver to work with this IMU. It does not provide all the functions that can be performed by this device. Notably, the initialization sequence uses the default configuration of the sensor.

