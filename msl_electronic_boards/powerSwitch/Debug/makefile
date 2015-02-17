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
FLASH_IMAGE += \
powerSwitch.hex \

SIZEDUMMY += \
sizedummy \

AVRDUDEDUMMY += \
avrdudedummy \


# All Target
all: powerSwitch.elf secondary-outputs

# Tool invocations
powerSwitch.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,powerSwitch.map -T /usr/i686-pc-linux-gnu/avr/lib/ldscripts/avr4.x -B /usr/avr/lib/avr4 -L/usr/avr/lib/avr4 -L/usr/i686-pc-linux-gnu/avr/lib -mmcu=atmega8 -o"powerSwitch.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

powerSwitch.hex: powerSwitch.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -O ihex powerSwitch.elf  "powerSwitch.hex"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: powerSwitch.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=berkeley -t powerSwitch.elf
	@echo 'Finished building: $@'
	@echo ' '

avrdudedummy: powerSwitch.elf
	@echo 'Invoking: AVRDude'
	/usr/bin/avrdude -pm8 -cavrispv2 -Pusb -Uflash:w:powerSwitch.hex:a
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(ASM_DEPS)$(FLASH_IMAGE)$(ELFS)$(AVRDUDEDUMMY)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS) powerSwitch.elf
	-@echo ' '

secondary-outputs: $(FLASH_IMAGE) $(SIZEDUMMY) $(AVRDUDEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
