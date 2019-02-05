################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := E:/elec/ti/ccs740/ccsv7/tools/compiler/ti-cgt-arm_18.1.5.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./main.obj" \
"./tm4c123gh6pm_startup_ccs.obj" \
"./0.96'OLED/OLED.obj" \
"./Beep/Beep.obj" \
"./Control/Control.obj" \
"./Filter/filter.obj" \
"./MavLink_Receive/Mavlink_test.obj" \
"./Pid/pid.obj" \
"./Pwm/pwm.obj" \
"./Timer/Timer.obj" \
"./colorful_LED/colorful_LED.obj" \
"./delay/delay.obj" \
"./key/key.obj" \
"./sonar/sonar.obj" \
"./uart/uart_echo.obj" \
"./uart/uartstdio.obj" \
"../tm4c123gh6pm.cmd" \
$(GEN_CMDS__FLAG) \
-llibc.a \
-l"E:/elec/TM4/TivaWare/driverlib/ccs/Debug/driverlib.lib" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include 0.96'OLED/subdir_vars.mk
-include Beep/subdir_vars.mk
-include Control/subdir_vars.mk
-include Filter/subdir_vars.mk
-include MavLink_Receive/subdir_vars.mk
-include Pid/subdir_vars.mk
-include Pwm/subdir_vars.mk
-include Timer/subdir_vars.mk
-include colorful_LED/subdir_vars.mk
-include delay/subdir_vars.mk
-include key/subdir_vars.mk
-include sonar/subdir_vars.mk
-include uart/subdir_vars.mk
-include subdir_rules.mk
-include 0.96'OLED/subdir_rules.mk
-include Beep/subdir_rules.mk
-include Control/subdir_rules.mk
-include Filter/subdir_rules.mk
-include MavLink_Receive/subdir_rules.mk
-include Pid/subdir_rules.mk
-include Pwm/subdir_rules.mk
-include Timer/subdir_rules.mk
-include colorful_LED/subdir_rules.mk
-include delay/subdir_rules.mk
-include key/subdir_rules.mk
-include sonar/subdir_rules.mk
-include uart/subdir_rules.mk
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
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
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
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
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
Four_axes.out \

EXE_OUTPUTS__QUOTED += \
"Four_axes.out" \

BIN_OUTPUTS += \
Four_axes.hex \

BIN_OUTPUTS__QUOTED += \
"Four_axes.hex" \


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "Four_axes.out"

# Tool invocations
Four_axes.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: ARM Linker'
	"E:/elec/ti/ccs740/ccsv7/tools/compiler/ti-cgt-arm_18.1.5.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --define=ccs="ccs" --define=PART_TM4C123GH6PM -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi -z -m"Four_axes.map" --heap_size=0 --stack_size=512 -i"E:/elec/ti/ccs740/ccsv7/tools/compiler/ti-cgt-arm_18.1.5.LTS/lib" -i"E:/elec/ti/ccs740/ccsv7/tools/compiler/ti-cgt-arm_18.1.5.LTS/include" --reread_libs --diag_wrap=off --display_error_number --warn_sections --xml_link_info="Four_axes_linkInfo.xml" --rom_model -o "Four_axes.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

Four_axes.hex: $(EXE_OUTPUTS)
	@echo 'Building files: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo 'Invoking: ARM Hex Utility'
	"E:/elec/ti/ccs740/ccsv7/tools/compiler/ti-cgt-arm_18.1.5.LTS/bin/armhex"  -o "Four_axes.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "main.obj" "tm4c123gh6pm_startup_ccs.obj" "0.96'OLED\OLED.obj" "Beep\Beep.obj" "Control\Control.obj" "Filter\filter.obj" "MavLink_Receive\Mavlink_test.obj" "Pid\pid.obj" "Pwm\pwm.obj" "Timer\Timer.obj" "colorful_LED\colorful_LED.obj" "delay\delay.obj" "key\key.obj" "sonar\sonar.obj" "uart\uart_echo.obj" "uart\uartstdio.obj" 
	-$(RM) "main.d" "tm4c123gh6pm_startup_ccs.d" "0.96'OLED\OLED.d" "Beep\Beep.d" "Control\Control.d" "Filter\filter.d" "MavLink_Receive\Mavlink_test.d" "Pid\pid.d" "Pwm\pwm.d" "Timer\Timer.d" "colorful_LED\colorful_LED.d" "delay\delay.d" "key\key.d" "sonar\sonar.d" "uart\uart_echo.d" "uart\uartstdio.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

