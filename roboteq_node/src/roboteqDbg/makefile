include ../misc/makefile.inc

LIBS		:= ${LIBS} -lncurses 
LIBS_DIR	:= ${LIBS_DIR}
INCS_DIR	:= ${INCS_DIR} -I../../include/
CFLAGS		:= ${CFLAGS} 
LDFLAGS		:= ${LDFLAGS}

ifeq (${PLATFORM},Darwin)
	INCS_DIR    := ${INCS_DIR} 
	LIBS_DIR    := ${LIBS_DIR}
endif

#****************************************************************************
# Targets of the build
#****************************************************************************

OUTPUT := roboteqDbg 

all: ${OUTPUT}

#****************************************************************************
# Source files
#****************************************************************************
SRCS := main.cpp\
	mainWindow.cpp\
	roboteqLogger.cpp\
	../roboteqCom/roboteqCom.cpp\
	../roboteqCom/roboteqThread.cpp\
	../serialConnector/serialPort.cpp

# Add on the sources for libraries
SRCS := ${SRCS}

OBJS := $(addsuffix .o,$(basename ${SRCS}))

#****************************************************************************
# Output
#****************************************************************************
${OUTPUT}: ${OBJS}
	${LD} -o ./$@ ${LDFLAGS} ${OBJS} ${LIBS_DIR} ${LIBS}
	

#****************************************************************************
# common rules
#****************************************************************************

# Rules for compiling source files to object files
%.o : %.cpp
	${CXX} -c ${CFLAGS} ${INCS_DIR} $< -o $@

clean:
	rm -f ${CLEAN_OBJ} ./${OUTPUT} ../roboteqCom/*.o  ../serialConnector/*.o

