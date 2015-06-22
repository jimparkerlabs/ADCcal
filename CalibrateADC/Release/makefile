################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
CalibrateADC.lss \

FLASH_IMAGE += \
CalibrateADC.hex \

EEPROM_IMAGE += \
CalibrateADC.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: CalibrateADC.elf secondary-outputs

# Tool invocations
CalibrateADC.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,CalibrateADC.map -L/Users/jimparker/Dropbox/Jim_Parker_Labs/AVR/includes -mmcu=atmega168 -o "CalibrateADC.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

CalibrateADC.lss: CalibrateADC.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S CalibrateADC.elf  >"CalibrateADC.lss"
	@echo 'Finished building: $@'
	@echo ' '

CalibrateADC.hex: CalibrateADC.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex CalibrateADC.elf  "CalibrateADC.hex"
	@echo 'Finished building: $@'
	@echo ' '

CalibrateADC.eep: CalibrateADC.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex CalibrateADC.elf  "CalibrateADC.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: CalibrateADC.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega168 CalibrateADC.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(ASM_DEPS)$(EEPROM_IMAGE)$(FLASH_IMAGE)$(ELFS)$(LSS)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS) CalibrateADC.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
