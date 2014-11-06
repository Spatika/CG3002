################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS := 


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../FreeRTOS2560.cpp \
../heap_1.c \
../I2Cdev.cpp \
../list.c \
../MPU6050.cpp \
../NewPing.cpp \
../port.c \
../queue.c \
../tasks.c \
../twi.c \
../Wire.cpp


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
FreeRTOS2560.o \
heap_1.o \
I2Cdev.o \
list.o \
MPU6050.o \
NewPing.o \
port.o \
queue.o \
tasks.o \
twi.o \
Wire.o

OBJS_AS_ARGS +=  \
FreeRTOS2560.o \
heap_1.o \
I2Cdev.o \
list.o \
MPU6050.o \
NewPing.o \
port.o \
queue.o \
tasks.o \
twi.o \
Wire.o

C_DEPS +=  \
FreeRTOS2560.d \
heap_1.d \
I2Cdev.d \
list.d \
MPU6050.d \
NewPing.d \
port.d \
queue.d \
tasks.d \
twi.d \
Wire.d

C_DEPS_AS_ARGS +=  \
FreeRTOS2560.d \
heap_1.d \
I2Cdev.d \
list.d \
MPU6050.d \
NewPing.d \
port.d \
queue.d \
tasks.d \
twi.d \
Wire.d

OUTPUT_FILE_PATH +=FreeRTOS2560.elf

OUTPUT_FILE_PATH_AS_ARGS +=FreeRTOS2560.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

# AVR32/GNU C Compiler























./%.o: .././%.c
	@echo Building file: $<
	@echo Invoking: AVR8/GNU C Compiler : 4.8.1
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE)  -x c -funsigned-char -funsigned-bitfields -DDEBUG  -I"../../../../../../../../Program Files (x86)/Arduino/hardware/arduino/cores/arduino" -I"../../../../../../../../Program Files (x86)/Arduino/hardware/arduino/variants/mega" -I"E:\FreeRTOSV8.1.2\FreeRTOS\Demo\Common\include" -I"E:\FreeRTOSV8.1.2\FreeRTOS\Source\portable\GCC\ATMega2560" -I"E:\FreeRTOSV8.1.2\FreeRTOS\Source" -I"E:\FreeRTOSV8.1.2\FreeRTOS\Source\include" -I"../../../../../../../../Program Files (x86)/Arduino/hardware/arduino/cores/RTOSSerial" -I"../../../../../../../../Program Files (x86)/Arduino/hardware/arduino/cores/cserial" -I"../../../../../../../../Program Files (x86)/Arduino/hardware/arduino/cores/NewPing" -I"../../../../../../../../Program Files (x86)/Arduino/hardware/arduino/cores/Wire" -I"../../../../../../../../Program Files (x86)/Arduino/hardware/arduino/cores/Kalman" -I"../../../../../../../../Program Files (x86)/Arduino/hardware/arduino/cores/I2Cdev" -I"../../../../../../../../Program Files (x86)/Arduino/hardware/arduino/cores/MPU6050"  -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -mrelax -g2 -Wall -mmcu=atmega2560 -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

./%.o: .././%.cpp
	@echo Building file: $<
	@echo Invoking: AVR8/GNU C Compiler : 4.8.1
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-g++.exe$(QUOTE) -funsigned-char -funsigned-bitfields -DDEBUG  -I"../../../../../../../../Program Files (x86)/Arduino/hardware/arduino/cores/arduino" -I"../../../../../../../../Program Files (x86)/Arduino/hardware/arduino/variants/mega" -I"E:\FreeRTOSV8.1.2\FreeRTOS\Demo\Common\include" -I"E:\FreeRTOSV8.1.2\FreeRTOS\Source\portable\GCC\ATMega2560" -I"E:\FreeRTOSV8.1.2\FreeRTOS\Source\include" -I"../../../../../../../../Program Files (x86)/Arduino/hardware/arduino/cores/RTOSSerial" -I"../../../../../../../../Program Files (x86)/Arduino/hardware/arduino/cores/cserial" -I"../../../../../../../../Program Files (x86)/Arduino/hardware/arduino/cores/NewPing" -I"../../../../../../../../Program Files (x86)/Arduino/hardware/arduino/cores/Wire" -I"../../../../../../../../Program Files (x86)/Arduino/hardware/arduino/cores/Kalman" -I"../../../../../../../../Program Files (x86)/Arduino/hardware/arduino/cores/I2Cdev" -I"../../../../../../../../Program Files (x86)/Arduino/hardware/arduino/cores/MPU6050"  -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -mrelax -g2 -Wall -mmcu=atmega2560 -c -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP)
	@echo Building target: $@
	@echo Invoking: AVR8/GNU Linker : 4.8.1
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-g++.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="FreeRTOS2560.map" -Wl,--start-group -Wl,-larduino -Wl,-lm  -Wl,--end-group -Wl,-L"../../../Arduino/Arduino/Debug"  -Wl,--gc-sections -mrelax -mmcu=atmega2560  
	@echo Finished building target: $@
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "FreeRTOS2560.elf" "FreeRTOS2560.hex"
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "FreeRTOS2560.elf" "FreeRTOS2560.eep" || exit 0
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-objdump.exe" -h -S "FreeRTOS2560.elf" > "FreeRTOS2560.lss"
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "FreeRTOS2560.elf" "FreeRTOS2560.srec"
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-size.exe" "FreeRTOS2560.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "FreeRTOS2560.elf" "FreeRTOS2560.a" "FreeRTOS2560.hex" "FreeRTOS2560.lss" "FreeRTOS2560.eep" "FreeRTOS2560.map" "FreeRTOS2560.srec" "FreeRTOS2560.usersignatures"
	