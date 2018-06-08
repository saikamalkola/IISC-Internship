################################################################################
# Automatically-generated file. Do not edit!
################################################################################

CG_TOOL_ROOT := /home/kamal/ti/ccsv7/tools/compiler/gcc-arm-none-eabi-6-2017-q1-update

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./freertos_demo.o" \
"./led_task.o" \
"./switch_task.o" \
"./tm4c1233h6pm_startup_ccs_gcc.o" \
"./uartstdio.o" \
"./FreeRTOS/croutine.o" \
"./FreeRTOS/event_groups.o" \
"./FreeRTOS/list.o" \
"./FreeRTOS/queue.o" \
"./FreeRTOS/tasks.o" \
"./FreeRTOS/timers.o" \
"./FreeRTOS/portable/GCC/ARM_CM4F/port.o" \
"./FreeRTOS/portable/MemMang/heap_2.o" \
"./drivers/buttons.o" \
"./drivers/rgb.o" \
-Wl,-T"../tm4c1233h6pm.lds" \
$(GEN_CMDS__FLAG) \

-include ../makefile.init

RM := rm -rf
RMDIR := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include FreeRTOS/subdir_vars.mk
-include FreeRTOS/portable/GCC/ARM_CM4F/subdir_vars.mk
-include FreeRTOS/portable/MemMang/subdir_vars.mk
-include drivers/subdir_vars.mk
-include subdir_rules.mk
-include FreeRTOS/subdir_rules.mk
-include FreeRTOS/portable/GCC/ARM_CM4F/subdir_rules.mk
-include FreeRTOS/portable/MemMang/subdir_rules.mk
-include drivers/subdir_rules.mk
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
freertos_on_tm4c123.out \

EXE_OUTPUTS__QUOTED += \
"freertos_on_tm4c123.out" \

BIN_OUTPUTS += \
freertos_on_tm4c123.hex \

BIN_OUTPUTS__QUOTED += \
"freertos_on_tm4c123.hex" \


# All Target
all: $(OBJS) $(LDS_SRCS) $(A_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "freertos_on_tm4c123.out"

# Tool invocations
freertos_on_tm4c123.out: $(OBJS) $(LDS_SRCS) $(A_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: GNU Linker'
	"/home/kamal/ti/ccsv7/tools/compiler/gcc-arm-none-eabi-6-2017-q1-update/bin/arm-none-eabi-gcc" -DPART_TM4C1233H6PM -Dgcc -DTARGET_IS_TM4C123_RB1 -Os -ffunction-sections -fdata-sections -fsingle-precision-constant -g -gdwarf-3 -gstrict-dwarf -Wall -specs="nosys.specs" -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Wl,-Map,"freertos_on_tm4c123.map" -Wl,--gc-sections -o"freertos_on_tm4c123.out" $(ORDERED_OBJS) $(A_SRCS)
	@echo 'Finished building target: "$@"'
	@echo ' '

freertos_on_tm4c123.hex: $(EXE_OUTPUTS)
	@echo 'Building files: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo 'Invoking: GNU Objcopy Utility'
	"/home/kamal/ti/ccsv7/tools/compiler/gcc-arm-none-eabi-6-2017-q1-update/bin/arm-none-eabi-objcopy"  $(EXE_OUTPUTS__QUOTED) "freertos_on_tm4c123.hex"
	@echo 'Finished building: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "freertos_demo.o" "led_task.o" "switch_task.o" "tm4c1233h6pm_startup_ccs_gcc.o" "uartstdio.o" "FreeRTOS/croutine.o" "FreeRTOS/event_groups.o" "FreeRTOS/list.o" "FreeRTOS/queue.o" "FreeRTOS/tasks.o" "FreeRTOS/timers.o" "FreeRTOS/portable/GCC/ARM_CM4F/port.o" "FreeRTOS/portable/MemMang/heap_2.o" "drivers/buttons.o" "drivers/rgb.o" 
	-$(RM) "freertos_demo.d" "led_task.d" "switch_task.d" "tm4c1233h6pm_startup_ccs_gcc.d" "uartstdio.d" "FreeRTOS/croutine.d" "FreeRTOS/event_groups.d" "FreeRTOS/list.d" "FreeRTOS/queue.d" "FreeRTOS/tasks.d" "FreeRTOS/timers.d" "FreeRTOS/portable/GCC/ARM_CM4F/port.d" "FreeRTOS/portable/MemMang/heap_2.d" "drivers/buttons.d" "drivers/rgb.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

