################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
SYSCFG_SRCS += \
../empty.syscfg 

C_SRCS += \
../app.c \
../app_config.c \
../car_controller.c \
../car_state.c \
../debug_telemetry.c \
../emergency_stop.c \
../empty.c \
./ti_msp_dl_config.c \
C:/TI/mspm0_sdk_2_05_01_00/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c \
../encoder.c \
../fault.c \
../gimbal.c \
../gimbal_stepper.c \
../gimbal_tracker.c \
../gimbal_vision_adapter.c \
../gimbal_vision_pitch_tracker.c \
../gimbal_vision_yaw_tracker.c \
../heading_control.c \
../imu.c \
../key.c \
../line_controller.c \
../menu.c \
../mission_library.c \
../mission_manager.c \
../motion_action.c \
../motor.c \
../motor_control.c \
../obstacle_avoidance.c \
../obstacle_monitor.c \
../obstacle_safety.c \
../obstacle_scanner.c \
../oled.c \
../oled_ui.c \
../pid.c \
../runtime_snapshot.c \
../servo.c \
../track_sensor.c \
../ultrasonic.c \
../vision_pitch_tuning.c \
../vision_protocol.c \
../vision_receiver.c \
../vision_tuning_console.c \
../vision_uart.c \
../vision_yaw_tuning.c \
../watchdog_monitor.c \
../wheel_speed_estimator.c 

GEN_CMDS += \
./device_linker.cmd 

GEN_FILES += \
./device_linker.cmd \
./device.opt \
./ti_msp_dl_config.c 

C_DEPS += \
./app.d \
./app_config.d \
./car_controller.d \
./car_state.d \
./debug_telemetry.d \
./emergency_stop.d \
./empty.d \
./ti_msp_dl_config.d \
./startup_mspm0g350x_ticlang.d \
./encoder.d \
./fault.d \
./gimbal.d \
./gimbal_stepper.d \
./gimbal_tracker.d \
./gimbal_vision_adapter.d \
./gimbal_vision_pitch_tracker.d \
./gimbal_vision_yaw_tracker.d \
./heading_control.d \
./imu.d \
./key.d \
./line_controller.d \
./menu.d \
./mission_library.d \
./mission_manager.d \
./motion_action.d \
./motor.d \
./motor_control.d \
./obstacle_avoidance.d \
./obstacle_monitor.d \
./obstacle_safety.d \
./obstacle_scanner.d \
./oled.d \
./oled_ui.d \
./pid.d \
./runtime_snapshot.d \
./servo.d \
./track_sensor.d \
./ultrasonic.d \
./vision_pitch_tuning.d \
./vision_protocol.d \
./vision_receiver.d \
./vision_tuning_console.d \
./vision_uart.d \
./vision_yaw_tuning.d \
./watchdog_monitor.d \
./wheel_speed_estimator.d 

GEN_OPTS += \
./device.opt 

OBJS += \
./app.o \
./app_config.o \
./car_controller.o \
./car_state.o \
./debug_telemetry.o \
./emergency_stop.o \
./empty.o \
./ti_msp_dl_config.o \
./startup_mspm0g350x_ticlang.o \
./encoder.o \
./fault.o \
./gimbal.o \
./gimbal_stepper.o \
./gimbal_tracker.o \
./gimbal_vision_adapter.o \
./gimbal_vision_pitch_tracker.o \
./gimbal_vision_yaw_tracker.o \
./heading_control.o \
./imu.o \
./key.o \
./line_controller.o \
./menu.o \
./mission_library.o \
./mission_manager.o \
./motion_action.o \
./motor.o \
./motor_control.o \
./obstacle_avoidance.o \
./obstacle_monitor.o \
./obstacle_safety.o \
./obstacle_scanner.o \
./oled.o \
./oled_ui.o \
./pid.o \
./runtime_snapshot.o \
./servo.o \
./track_sensor.o \
./ultrasonic.o \
./vision_pitch_tuning.o \
./vision_protocol.o \
./vision_receiver.o \
./vision_tuning_console.o \
./vision_uart.o \
./vision_yaw_tuning.o \
./watchdog_monitor.o \
./wheel_speed_estimator.o 

GEN_MISC_FILES += \
./device.cmd.genlibs \
./ti_msp_dl_config.h \
./Event.dot 

OBJS__QUOTED += \
"app.o" \
"app_config.o" \
"car_controller.o" \
"car_state.o" \
"debug_telemetry.o" \
"emergency_stop.o" \
"empty.o" \
"ti_msp_dl_config.o" \
"startup_mspm0g350x_ticlang.o" \
"encoder.o" \
"fault.o" \
"gimbal.o" \
"gimbal_stepper.o" \
"gimbal_tracker.o" \
"gimbal_vision_adapter.o" \
"gimbal_vision_pitch_tracker.o" \
"gimbal_vision_yaw_tracker.o" \
"heading_control.o" \
"imu.o" \
"key.o" \
"line_controller.o" \
"menu.o" \
"mission_library.o" \
"mission_manager.o" \
"motion_action.o" \
"motor.o" \
"motor_control.o" \
"obstacle_avoidance.o" \
"obstacle_monitor.o" \
"obstacle_safety.o" \
"obstacle_scanner.o" \
"oled.o" \
"oled_ui.o" \
"pid.o" \
"runtime_snapshot.o" \
"servo.o" \
"track_sensor.o" \
"ultrasonic.o" \
"vision_pitch_tuning.o" \
"vision_protocol.o" \
"vision_receiver.o" \
"vision_tuning_console.o" \
"vision_uart.o" \
"vision_yaw_tuning.o" \
"watchdog_monitor.o" \
"wheel_speed_estimator.o" 

GEN_MISC_FILES__QUOTED += \
"device.cmd.genlibs" \
"ti_msp_dl_config.h" \
"Event.dot" 

C_DEPS__QUOTED += \
"app.d" \
"app_config.d" \
"car_controller.d" \
"car_state.d" \
"debug_telemetry.d" \
"emergency_stop.d" \
"empty.d" \
"ti_msp_dl_config.d" \
"startup_mspm0g350x_ticlang.d" \
"encoder.d" \
"fault.d" \
"gimbal.d" \
"gimbal_stepper.d" \
"gimbal_tracker.d" \
"gimbal_vision_adapter.d" \
"gimbal_vision_pitch_tracker.d" \
"gimbal_vision_yaw_tracker.d" \
"heading_control.d" \
"imu.d" \
"key.d" \
"line_controller.d" \
"menu.d" \
"mission_library.d" \
"mission_manager.d" \
"motion_action.d" \
"motor.d" \
"motor_control.d" \
"obstacle_avoidance.d" \
"obstacle_monitor.d" \
"obstacle_safety.d" \
"obstacle_scanner.d" \
"oled.d" \
"oled_ui.d" \
"pid.d" \
"runtime_snapshot.d" \
"servo.d" \
"track_sensor.d" \
"ultrasonic.d" \
"vision_pitch_tuning.d" \
"vision_protocol.d" \
"vision_receiver.d" \
"vision_tuning_console.d" \
"vision_uart.d" \
"vision_yaw_tuning.d" \
"watchdog_monitor.d" \
"wheel_speed_estimator.d" 

GEN_FILES__QUOTED += \
"device_linker.cmd" \
"device.opt" \
"ti_msp_dl_config.c" 

C_SRCS__QUOTED += \
"../app.c" \
"../app_config.c" \
"../car_controller.c" \
"../car_state.c" \
"../debug_telemetry.c" \
"../emergency_stop.c" \
"../empty.c" \
"./ti_msp_dl_config.c" \
"C:/TI/mspm0_sdk_2_05_01_00/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c" \
"../encoder.c" \
"../fault.c" \
"../gimbal.c" \
"../gimbal_stepper.c" \
"../gimbal_tracker.c" \
"../gimbal_vision_adapter.c" \
"../gimbal_vision_pitch_tracker.c" \
"../gimbal_vision_yaw_tracker.c" \
"../heading_control.c" \
"../imu.c" \
"../key.c" \
"../line_controller.c" \
"../menu.c" \
"../mission_library.c" \
"../mission_manager.c" \
"../motion_action.c" \
"../motor.c" \
"../motor_control.c" \
"../obstacle_avoidance.c" \
"../obstacle_monitor.c" \
"../obstacle_safety.c" \
"../obstacle_scanner.c" \
"../oled.c" \
"../oled_ui.c" \
"../pid.c" \
"../runtime_snapshot.c" \
"../servo.c" \
"../track_sensor.c" \
"../ultrasonic.c" \
"../vision_pitch_tuning.c" \
"../vision_protocol.c" \
"../vision_receiver.c" \
"../vision_tuning_console.c" \
"../vision_uart.c" \
"../vision_yaw_tuning.c" \
"../watchdog_monitor.c" \
"../wheel_speed_estimator.c" 

SYSCFG_SRCS__QUOTED += \
"../empty.syscfg" 


