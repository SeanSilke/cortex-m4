CC = arm-none-eabi-gcc
AR = arm-none-eabi-ar

INCLUDES = -I ${PROJECT_DIR}/Libraries/STM32F4xx_StdPeriph_Driver/inc
INCLUDES +=-I ${PROJECT_DIR}/Libraries/CMSIS/Device/ST/STM32F4xx/Include
INCLUDES +=-I ${PROJECT_DIR}/Libraries/CMSIS/Include
INCLUDES +=-I ${PROJECT_DIR}

OPTIONS =-mcpu=cortex-m4 -mfloat-abi=hard -mthumb -mfpu=fpv4-sp-d16 -std=gnu99
OPTIONS +=-DUSE_STDPERIPH_DRIVER -DSTM32F40_41xxx 
#OPTIONS +=-DUSE_STM324xG_EVAL
OPTIONS +=-ggdb

OBJS =  misc.o stm32f4xx_dma.o stm32f4xx_gpio.o stm32f4xx_rcc.o \
 				stm32f4xx_usart.o

.PHONY: all clean

all: ${OBJS}
	${AR} rc libperiph.a ${OBJS}

%.o: %.c
	${CC} ${INCLUDES} -c ${OPTIONS} $<

clean:
	-rm *.o
	-rm libperiph.a
