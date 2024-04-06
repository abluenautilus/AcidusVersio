# Project Name
TARGET = AcidusVersio

#Open303
OPEN303 = /Users/jonask/Music/Daisy/Projects/Versio/AcidusVersio/open303
OPEN303_SOURCE = ${OPEN303}/Source/DSPCode
CXXFLAGS += -I{OPEN303_SOURCE}

# Sources
CPP_SOURCES = AcidusVersio.cpp
CPP_SOURCES +=${wildcard /Users/jonask/Music/Daisy/Projects/Versio/AcidusVersio/open303/Source/DSPCode/*.cpp}

#DEBUG=1

# Library Locations
LIBDAISY_DIR = /Users/jonask/Music/Daisy/libDaisy
DAISYSP_DIR = /Users/jonask/Music/Daisy/DaisySP

# Linker flags
# This is not really required, used only for profiling! Increases executable size by ~8kB
# LDFLAGS = -u _printf_float

C_INCLUDES += -I${OPEN303_SOURCE}

# Core location, and generic Makefile.
OPT = -Os
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include ${SYSTEM_FILES_DIR}/Makefile

