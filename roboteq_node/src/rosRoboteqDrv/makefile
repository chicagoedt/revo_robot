include ../../makefile.inc

LIBS		:= ${LIBS} -lncurses 
LIBS_DIR	:= ${LIBS_DIR}
INCS_DIR	:= ${INCS_DIR} -I../roboteqCom -I../serialconnector
CFLAGS		:= ${CFLAGS} 
LDFLAGS		:= ${LDFLAGS}

ifeq (${PLATFORM},Darwin)
	INCS_DIR    := ${INCS_DIR} 
	LIBS_DIR    := ${LIBS_DIR}
endif

#****************************************************************************
# Targets of the build
#****************************************************************************

OUTPUT := rosRoboteqDrv 

all: ${OUTPUT}

#****************************************************************************
# Source files
#****************************************************************************
SRCS := main.cpp\
    rosRoboteqDrv.cpp\
    ../roboteqCom/roboteqCom.cpp\
    ../roboteqCom/roboteqThread.cpp\
    ../serialconnector/serialPort.cpp

# Add on the sources for libraries
SRCS := ${SRCS}

OBJS := $(addsuffix .o,$(basename ${SRCS}))

#****************************************************************************
# Output
#****************************************************************************
${OUTPUT}: ${OBJS}
	${LD} -o ../bin/${OUT_DIR}/$@ ${LDFLAGS} ${OBJS} ${LIBS_DIR} ${LIBS}
	

#****************************************************************************
# common rules
#****************************************************************************

# Rules for compiling source files to object files
%.o : %.cpp
	${CXX} -c ${CFLAGS} ${INCS_DIR} $< -o $@

clean:
	rm -f ${CLEAN_OBJ} ../bin/${OUT_DIR}/${OUTPUT}

