ARCH        := $(shell uname -m)
PLATFORM    := $(shell uname)

#ARCH_FLAGS  := -m32
OUT_DIR     := x32
LIBS_DIR    :=
INCS_DIR    :=

ifeq (${ARCH},x86_64)
    ARCH_FLAGS  := -m64
    #LIBS_DIR   := -L/usr/lib64
    OUT_DIR     := x64
endif

LIBS_DIR    := ${LIBS_DIR} -L/usr/local/lib
INCS_DIR    := ${INCS_DIR} -I/usr/local/include

CXX         := g++ -std=c++11 ${ARCH_FLAGS}
LD          := g++ -std=c++11 ${ARCH_FLAGS}
AR          := ar rcs
RANLIB      := ranlib

LIBS        := -Bdynamic -lpthread
CFLAGS      := -g -pedantic -Wno-deprecated -Wno-long-long -pipe -Wall -D_DEBUG -D_REENTRANT
SOFLAGS     := -fPIC
LDFLAGS     :=
SQLLIB      :=
CLEAN_OBJ   := core *.o


