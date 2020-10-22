target=bareCortexM

OBJ=src/exception.o src/interrupt.o src/main.o src/rs485.o src/startup.o src/system_call.o 

MCU=stm32f103c8t6
DEBUG=-O0 -g -ggdb
INCLUDE=-I libstm32pp/include
CFLAGS=-mcpu=cortex-m3 -mthumb $(DEBUG) -fsingle-precision-constant $(INCLUDE) -DSTM32F1XX
CXXFLAGS=$(CFLAGS) -std=c++11 -fabi-version=0 -fno-exceptions -fno-rtti
LDFLAGS=-T linker/$(MCU).ld -Xlinker --gc-sections -Wl,-Map,bareCortexM.map

CC=arm-none-eabi-gcc
CXX=arm-none-eabi-g++
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump
SIZE=arm-none-eabi-size

.PHONY: proj

all: proj

proj: $(target).elf

$(target).elf: $(OBJ)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $^
	$(OBJCOPY) -O ihex $@ $(target).hex
	$(OBJCOPY) -O binary $@ $(target).bin
	$(OBJDUMP) -St $@ >$(target).lst
	$(SIZE) $@

clean:
	rm -f $(target).elf $(OBJ) $(target).hex $(target).bin $(target).lst $(target).map

