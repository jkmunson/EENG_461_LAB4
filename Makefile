
CC		= arm-none-eabi-gcc
LD		= arm-none-eabi-ld
AR		= arm-none-eabi-ar
AS		= arm-none-eabi-as
OBJSIZE	= arm-none-eabi-size
OBJDUMP	= arm-none-eabi-objdump
OBJCOPY	= arm-none-eabi-objcopy
CPU		= -mcpu=cortex-m4
FPU		= -mfpu=fpv4-sp-d16 -mfloat-abi=hard

AFLAGS	= -mthumb ${CPU} ${FPU} -MD

CFLAGS	= -mthumb ${CPU} ${FPU} -Og -ffunction-sections -fno-builtin -fdata-sections -MD -std=c17 -Wall -Wextra -Wdouble-promotion -pedantic -c -Dgcc -g -I driver -I src -I sys 
#-fno-builtin

CFLAGS	+= -DPART_TM4C123GH6PM -DTARGET_IS_BLIZZARD_RA1

LDFLAGS=--gc-sections 
#-L /usr/lib/arm-none-eabi/newlib -lc -lm

AFLAGS+=${patsubst %,-I%,${subst :, ,${IPATH}}}
CFLAGS+=${patsubst %,-I%,${subst :, ,${IPATH}}}

export CC LD AR AS OBJCOPY
export CFLAGS AFLAGS LDFLAGS

BUILD_DIR  := objs
SRCS       := $(wildcard src/*.c) $(wildcard sys/*.c)
HEADERS    := $(wildcard src/*.h) $(wildcard src/*.h) Makefile
OBJECTS    := $(addprefix $(BUILD_DIR)/,$(subst .c,.o, $(SRCS)))
TARGET     := lab4binary

all: credir $(BUILD_DIR)/$(TARGET).axf

# Compile
$(BUILD_DIR)/%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -o $(@) -MD -MF $(addprefix $(BUILD_DIR)/,$(subst .c,.d,$<)) $(firstword $^)

# Linking
$(BUILD_DIR)/$(TARGET).axf: $(OBJECTS)
	$(LD) $(OBJECTS) -T sys/tm4c123.ld --entry ResetISR $(LDFLAGS) -o $(@) -Map=$(BUILD_DIR)/$(TARGET).map
	$(OBJCOPY) -O binary ${@} ${@:.axf=.bin}
	$(OBJDUMP) -Sd -W $(@) > ${BUILD_DIR}/$(TARGET).lss
	$(OBJSIZE) ${@}

# Load binary to device
flash:
	sudo lm4flash $(BUILD_DIR)/$(TARGET).bin

clean:
	rm -rf $(BUILD_DIR)/*
	rm lab3_report.pdf

credir:
	mkdir -p $(BUILD_DIR)
	mkdir -p $(BUILD_DIR)/src
	mkdir -p $(BUILD_DIR)/sys

app_info: $(BUILD_DIR)/${TARGET}.axf
	arm-none-eabi-readelf -a $(^)

%.pdf: docs/src/%.md Makefile
	pandoc $< -o $@ --highlight-style tango --pdf-engine=xelatex 

report: lab3_report.pdf

.PHONY: all clean flash compiling app_info report
