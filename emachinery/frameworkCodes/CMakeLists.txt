cmake_minimum_required(VERSION 3.16)
project(acmsimcv5 C)
# C11 is current standard, see https://stackoverflow.com/questions/41535927/compatibility-of-c89-c90-c99-and-c11
set(CMAKE_C_STANDARD 11)
include_directories(c)
set(SOURCE_FILES c/main.c)
file(GLOB SOURCES
        c/*.h
        c/*.c
        )
add_executable(acmsimcv5 ${SOURCE_FILES} ${SOURCES})


# touch









# Create Cmake project will automatically generate this for ya:
#add_executable(acmsimcv5
#        c/ACMConfig.h
#        c/ACMSim.h
#        c/commands.c
#        c/commissioning.c
#        c/commissioning.h
#        c/global_variables_definitions.c
#        c/im_controller.c
#        c/im_controller.h
#        c/im_observer.c
#        c/im_observer.h
#        c/induction_motor.c
#        c/induction_motor.h
#        c/inverter.c
#        c/inverter.h
#        c/load.c
#        c/load.h
#        c/main.c
#        c/measurement.c
#        c/measurement.h
#        c/pid_regulator.c
#        c/pid_regulator.h
#        c/pmsm_comm.c
#        c/pmsm_comm.h
#        c/pmsm_controller.c
#        c/pmsm_controller.h
#        c/pmsm_observer.c
#        c/pmsm_observer.h
#        c/satlut.h
#        c/sweep_frequency.h
#        c/synchronous_motor.c
#        c/synchronous_motor.h
#        c/utility.c)
