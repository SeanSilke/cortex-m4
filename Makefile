CC = arm-none-eabi-gcc
OC = arm-none-eabi-objcopy
AS = arm-none-eabi-as

PROJECT_DIR=$(shell pwd)

OPTIONS =-mcpu=cortex-m4 -mfloat-abi=hard -mthumb -mfpu=fpv4-sp-d16 -std=gnu99
OPTIONS +=-DUSE_STDPERIPH_DRIVER -DSTM32F40_41xxx -DUSE_STM324xG_EVAL
OPTIONS +=-nostartfiles

INCLUDES = -I .
INCLUDES +=-I ./Libraries/CMSIS/Device/ST/STM32F4xx/Include
INCLUDES +=-I ./Libraries/CMSIS/Include
INCLUDES +=-I ./Libraries/STM32F4xx_StdPeriph_Driver/inc
INCLUDES +=-I ./Utilities/STM3240_41_G_EVAL

LINKERS = -L./Libraries/STM32F4xx_StdPeriph_Driver/src
LINKERS += -L./Utilities/STM3240_41_G_EVAL

LIBS = -lutil
LIBS += -lperiph

LDFLAGS = -T my_ldscrypt.ld

BINNAME = out.bin
HEXNAME = out.hex
.PHONY: all clean


OBJS = main.o stm32f4xx_it.o system_stm32f4xx.o \
			startup_stm32f40_41xxx.o

all: ${BINNAME}
	${OC} -O ihex  ${BINNAME} ${HEXNAME}


${BINNAME}: libutil libperiph ${OBJS}
	${CC} ${OPTIONS} ${INCLUDES} ${OBJS} ${LDFLAGS} ${LINKERS} ${LIBS} -o ${BINNAME}

%.o: %.c
	${CC} ${INCLUDES} -c ${OPTIONS} $<

libutil:
	@echo "---Building libutil---"
	@cd ./Utilities/STM3240_41_G_EVAL && $(MAKE) PROJECT_DIR=${PROJECT_DIR}

libperiph:
	@echo "---Building libperiph---"
	@cd ./Libraries/STM32F4xx_StdPeriph_Driver/src && $(MAKE) PROJECT_DIR=${PROJECT_DIR}

load: ${BINNAME}
	@/home/sorlov/code/f1/stm32flash/stm32flash -b 115200 -w ${HEXNAME} -v /dev/ttyS0

clean:
	@-rm -rf *.o
	@-rm ${BINNAME}
	@-rm ${HEXNAME}
	@-cd ./Libraries/STM32F4xx_StdPeriph_Driver/src && $(MAKE) clean
	@-cd ./Utilities/STM3240_41_G_EVAL && $(MAKE) clean
