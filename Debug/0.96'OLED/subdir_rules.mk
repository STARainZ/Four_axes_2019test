################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
0.96'OLED/OLED.obj: ../0.96'OLED/OLED.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"E:/elec/ti/ccs740/ccsv7/tools/compiler/ti-cgt-arm_18.1.5.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="E:/elec/ti/workspace_v7/Four_axes" --include_path="E:/elec/TM4/TivaWare" --include_path="E:/elec/ti/ccs740/ccsv7/tools/compiler/ti-cgt-arm_18.1.5.LTS/include" --define=ccs="ccs" --define=PART_TM4C123GH6PM -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="0.96'OLED/OLED.d_raw" --obj_directory="0.96'OLED" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

