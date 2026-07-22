################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := D:/TI/CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS

GEN_OPTS__FLAG := @"./device.opt" 
GEN_CMDS__FLAG := -Wl,-l"./device_linker.cmd" 

ORDERED_OBJS += \
"./app.o" \
"./app_config.o" \
"./car_controller.o" \
"./car_state.o" \
"./debug_telemetry.o" \
"./emergency_stop.o" \
"./empty.o" \
"./ti_msp_dl_config.o" \
"./startup_mspm0g350x_ticlang.o" \
"./encoder.o" \
"./fault.o" \
"./gimbal.o" \
"./gimbal_stepper.o" \
"./gimbal_tracker.o" \
"./gimbal_vision_adapter.o" \
"./gimbal_vision_pitch_tracker.o" \
"./gimbal_vision_yaw_tracker.o" \
"./heading_control.o" \
"./imu.o" \
"./key.o" \
"./menu.o" \
"./mission_library.o" \
"./mission_manager.o" \
"./motion_action.o" \
"./motor.o" \
"./motor_control.o" \
"./obstacle_avoidance.o" \
"./obstacle_monitor.o" \
"./obstacle_safety.o" \
"./obstacle_scanner.o" \
"./oled.o" \
"./oled_ui.o" \
"./pid.o" \
"./runtime_snapshot.o" \
"./servo.o" \
"./track_sensor.o" \
"./ultrasonic.o" \
"./vision_pitch_tuning.o" \
"./vision_protocol.o" \
"./vision_receiver.o" \
"./vision_tuning_console.o" \
"./vision_uart.o" \
"./vision_yaw_tuning.o" \
"./watchdog_monitor.o" \
"./wheel_speed_estimator.o" \
$(GEN_CMDS__FLAG) \
-Wl,-ldevice.cmd.genlibs \
-Wl,-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
diansaikaishi.out 

EXE_OUTPUTS__QUOTED += \
"diansaikaishi.out" 


# All Target
all: $(OBJS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "diansaikaishi.out"

# Tool invocations
diansaikaishi.out: $(OBJS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: Arm Linker'
	"D:/TI/CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -gdwarf-3 -Wall -Wl,-m"diansaikaishi.map" -Wl,-i"C:/TI/mspm0_sdk_2_05_01_00/source" -Wl,-i"D:/selfthings/github_project/work_ti/79/diansaikaishi" -Wl,-i"D:/selfthings/github_project/work_ti/79/diansaikaishi/Debug/syscfg" -Wl,-i"D:/TI/CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/lib" -Wl,--diag_wrap=off -Wl,--display_error_number -Wl,--warn_sections -Wl,--xml_link_info="diansaikaishi_linkInfo.xml" -Wl,--rom_model -o "diansaikaishi.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(GEN_MISC_FILES__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "app.o" "app_config.o" "car_controller.o" "car_state.o" "debug_telemetry.o" "emergency_stop.o" "empty.o" "ti_msp_dl_config.o" "startup_mspm0g350x_ticlang.o" "encoder.o" "fault.o" "gimbal.o" "gimbal_stepper.o" "gimbal_tracker.o" "gimbal_vision_adapter.o" "gimbal_vision_pitch_tracker.o" "gimbal_vision_yaw_tracker.o" "heading_control.o" "imu.o" "key.o" "menu.o" "mission_library.o" "mission_manager.o" "motion_action.o" "motor.o" "motor_control.o" "obstacle_avoidance.o" "obstacle_monitor.o" "obstacle_safety.o" "obstacle_scanner.o" "oled.o" "oled_ui.o" "pid.o" "runtime_snapshot.o" "servo.o" "track_sensor.o" "ultrasonic.o" "vision_pitch_tuning.o" "vision_protocol.o" "vision_receiver.o" "vision_tuning_console.o" "vision_uart.o" "vision_yaw_tuning.o" "watchdog_monitor.o" "wheel_speed_estimator.o" 
	-$(RM) "app.d" "app_config.d" "car_controller.d" "car_state.d" "debug_telemetry.d" "emergency_stop.d" "empty.d" "ti_msp_dl_config.d" "startup_mspm0g350x_ticlang.d" "encoder.d" "fault.d" "gimbal.d" "gimbal_stepper.d" "gimbal_tracker.d" "gimbal_vision_adapter.d" "gimbal_vision_pitch_tracker.d" "gimbal_vision_yaw_tracker.d" "heading_control.d" "imu.d" "key.d" "menu.d" "mission_library.d" "mission_manager.d" "motion_action.d" "motor.d" "motor_control.d" "obstacle_avoidance.d" "obstacle_monitor.d" "obstacle_safety.d" "obstacle_scanner.d" "oled.d" "oled_ui.d" "pid.d" "runtime_snapshot.d" "servo.d" "track_sensor.d" "ultrasonic.d" "vision_pitch_tuning.d" "vision_protocol.d" "vision_receiver.d" "vision_tuning_console.d" "vision_uart.d" "vision_yaw_tuning.d" "watchdog_monitor.d" "wheel_speed_estimator.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

