################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../hal/BstSensor.cpp \
../hal/sensors.cpp 

OBJS += \
./hal/BstSensor.o \
./hal/sensors.o 

CPP_DEPS += \
./hal/BstSensor.d \
./hal/sensors.d 


# Each subdirectory must supply rules for building sources it contributes
hal/%.o: ../hal/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ $(CFLAGS) -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


