TARGET=h101
EXECUTABLE=$(TARGET).elf

DBG_TARGET=h101_debug
DBG_EXECUTABLE=$(DBG_TARGET).elf

CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump
DB=arm-none-eabi-gdb


DEFS = -DUSE_STDPERIPH_DRIVER
STARTUP = startup.s

MCU = cortex-m3
topdir = ..
INCLUDES = -I$(topdir)/H101_dual/src/ \
	-I$(topdir)/CMSIS/ \
	-I$(topdir)/Peripherals/inc/

#BASE_CFLAGS    = -mcpu=$(MCU) -nostartfiles -mthumb -mlittle-endian -mthumb-interwork -ffast-math -fdata-sections -ffunction-sections $(DEFS) $(INCLUDES) -std=gnu99
BASE_CFLAGS    = -mcpu=$(MCU) -mthumb -mlittle-endian -mthumb-interwork -ffast-math -fdata-sections -ffunction-sections $(DEFS) $(INCLUDES) -std=gnu99
DBG_CFLAGS     = -Og -g3 -DDEBUG
RELEASE_CFLAGS = -Os

#BASE_LDFLAGS    = --specs=nano.specs --specs=nosys.specs -lc_nano -lm -Wl,--no-wchar-size-warning,-T,flash.ld,-Map,$(TARGET).map,--gc-sections 
BASE_LDFLAGS    = --specs=nano.specs --specs=nosys.specs -lc_nano -lm -Wl,--no-wchar-size-warning,-T,flash.ld,-Map,$(TARGET).map,--gc-sections 
DBG_LDFLAGS     = --specs=rdimon.specs -lrdimon -u _printf_float
RELEASE_LDFLAGS = -flto 

SRC = 	$(wildcard $(topdir)/H101_dual/src/*.c) \
	$(wildcard $(topdir)/H101_dual/src/config.h) \
	$(wildcard $(topdir)/H101_dual/src/*.cpp) \
	$(topdir)/CMSIS/system_gd32f1x0.c \
	core_cm3.c \
	$(topdir)/Peripherals/src/gd32f1x0_adc.c \
	$(topdir)/Peripherals/src/gd32f1x0_dma.c \
	$(topdir)/Peripherals/src/gd32f1x0_fmc.c \
	$(topdir)/Peripherals/src/gd32f1x0_gpio.c \
	$(topdir)/Peripherals/src/gd32f1x0_i2c.c \
	$(topdir)/Peripherals/src/gd32f1x0_rcc.c \
	$(topdir)/Peripherals/src/gd32f1x0_timer.c \
	$(topdir)/Peripherals/src/gd32f1x0_usart.c

OBJDIR = .
OBJ = $(addprefix "$(OBJDIR)/",$(addsuffix .o,$(basename $(SRC))))
OBJ += Startup.o



release: CFLAGS = $(RELEASE_CFLAGS)

release: LDFLAGS = $(RELEASE_LDFLAGS)

release: $(TARGET)


debug: CFLAGS = $(DBG_CFLAGS)

debug: LDFLAGS = $(DBG_LDFLAGS)

debug: $(DBG_TARGET)


all: debug release


$(TARGET).hex: $(EXECUTABLE)
	$(CP) -O ihex $^ $@

$(TARGET): $(EXECUTABLE)
	$(CP) -O binary $^ $@


$(EXECUTABLE): $(SRC) $(STARTUP) $(HEADERS)
	$(CC) $(BASE_CFLAGS) $(CFLAGS) $^ $(BASE_LDFLAGS) $(LDFLAGS) -o $@
	arm-none-eabi-size $(EXECUTABLE)

$(DBG_TARGET).hex: $(DBG_EXECUTABLE)
	$(CP) -O ihex $^ $@

$(DBG_TARGET): $(DBG_EXECUTABLE)
	$(CP) -O binary $^ $@


$(DBG_EXECUTABLE): $(SRC) $(STARTUP) $(HEADERS)
	$(CC) $(BASE_CFLAGS) $(CFLAGS) $^ $(BASE_LDFLAGS) $(LDFLAGS) -o $@
	arm-none-eabi-size $(DBG_EXECUTABLE)

clean:
	rm -f Startup.lst $(TARGET) $(TARGET).lst $(OBJ) $(AUTOGEN) \
		$(TARGET).out $(TARGET).hex  $(TARGET).map \
		$(TARGET).dmp $(EXECUTABLE) \
		$(DBG_TARGET) $(DBG_TARGET).lst \
		$(DBG_TARGET).out $(DBG_TARGET).hex  $(DBG_TARGET).map \
		$(DBG_TARGET).dmp $(DBG_EXECUTABLE)

flash: $(TARGET)
	openocd -f /usr/share/openocd/scripts/interface/stlink-v2.cfg -f /usr/share/openocd/scripts/target/stm32f1x.cfg -c init -c "reset halt" -c "flash write_image erase $(TARGET) 0x08000000" -c "verify_image $(TARGET) 0x08000000" -c "reset run" -c shutdown

flash-erase-bm:
	$(DB) -ex "set confirm off" -ex "target extended-remote /dev/ttyACM0" -ex "monitor swdp_scan" -ex "attach 1" -ex "monitor option erase" -ex "detach" -ex "quit"

flash-bm: $(EXECUTABLE)
	$(DB) $(EXECUTABLE) -ex "set confirm off" -ex "target extended-remote /dev/ttyACM0" -ex "monitor swdp_scan" -ex "attach 1" -ex "load" -ex "detach" -ex "quit"

flash-debug-bm: $(DBG_EXECUTABLE)
	$(DB) $(DBG_EXECUTABLE) -ex "set confirm off" -ex "target extended-remote /dev/ttyACM0" -ex "monitor swdp_scan" -ex "attach 1" -ex "load" -ex "detach" -ex "quit"

debug-bm: $(DBG_EXECUTABLE)
	$(DB) $(DBG_EXECUTABLE) -ex "target extended-remote /dev/ttyACM0" -ex "monitor swdp_scan" -ex "attach 1"

