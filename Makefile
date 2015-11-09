# export PATH=${PATH}:/home/sorlov/Tools/gcc-arm-none-eabi-4_9-2015q2/bin
CC = arm-none-eabi-gcc
OC = arm-none-eabi-objcopy
OPTIONS =-mcpu=cortex-m4 -mfloat-abi=hard -mthumb -mfpu=fpv4-sp-d16 -std=gnu99
OPTIONS +=-DUSE_STDPERIPH_DRIVER -DSTM32F40_41xxx -DUSE_STM324xG_EVAL
INCLUDES = -I .
INCLUDES +=-I ../STM32F4xx_DSP_StdPeriph_Lib_V1.6.0/Libraries/CMSIS/Include/
INCLUDES +=-I ../STM32F4xx_DSP_StdPeriph_Lib_V1.6.0/Project/STM32F4xx_StdPeriph_Templates/
INCLUDES +=-I ../STM32F4xx_DSP_StdPeriph_Lib_V1.6.0/Libraries/STM32F4xx_StdPeriph_Driver/inc/
INCLUDES +=-I ../STM32F4xx_DSP_StdPeriph_Lib_V1.6.0/Utilities/STM32_EVAL/Common/
INCLUDES +=-I ../STM32F4xx_DSP_StdPeriph_Lib_V1.6.0/Utilities/STM32_EVAL/Common/
INCLUDES +=-I ./lib
INCLUDES +=-I ./Include
BINNAME = target_bin_1
HEXNAME = out.hex
.PHONY: all clean # To declare all, clean are not files


OBJS = main.o  stm324xg_eval.o   stm32f4xx_it.o   stm32f4xx_usart.o misc.o \
stm32f4xx_gpio.o  stm32f4xx_rcc.o  system_stm32f4xx.o stm32f4xx_dma.o


all: ${BINNAME} load
	${OC} -O ihex  ${BINNAME} ${HEXNAME}


${BINNAME}: ${OBJS}
	${CC} ${OPTIONS} ${INCLUDES} ${OBJS} -o ${BINNAME}

%.o: %.c
	${CC} ${INCLUDES} -c ${OPTIONS} $<

load: ${BINNAME}
	@/home/sorlov/code/f1/stm32flash/stm32flash -b 115200 -w ${HEXNAME} -v /dev/ttyS0

clean:
	-rm -rf *.o
	-rm ${BINNAME}
	-rm ${HEXNAME}
# @testComand
