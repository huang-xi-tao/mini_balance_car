# Copyright 1996-2022 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

### Generic Makefile.include for Webots controllers, physics plugins, robot
### window libraries, remote control libraries and other libraries
### to be used with GNU make
###
### Platforms: Windows, macOS, Linux
### Languages: C, C++
###
### Authors: Olivier Michel, Yvan Bourquin, Fabien Rohrer
###          Edmund Ronald, Sergei Poskriakov
###
###-----------------------------------------------------------------------------
###
### This file is meant to be included from the Makefile files located in the
### Webots projects subdirectories. It is possible to set a number of variables
### to customize the build process, i.e., add source files, compilation flags,
### include paths, libraries, etc. These variables should be set in your local
### Makefile just before including this Makefile.include. This Makefile.include
### should never be modified.
###
### Here is a description of the variables you may set in your local Makefile:
###
### ---- C Sources ----
### if your program uses several C source files:
### C_SOURCES = my_plugin.c my_clever_algo.c my_graphics.c
###
### ---- C++ Sources ----
### if your program uses several C++ source files:
### CXX_SOURCES = my_plugin.cc my_clever_algo.cpp my_graphics.c++
###
### ---- Compilation options ----
### if special compilation flags are necessary:
### CFLAGS = -Wno-unused-result
###
### ---- Linked libraries ----
### if your program needs additional libraries:
### INCLUDE = -I"/my_library_path/include"
### LIBRARIES = -L"/path/to/my/library" -lmy_library -lmy_other_library
###
### ---- Linking options ----
### if special linking flags are needed:
### LFLAGS = -s
###
### ---- Webots included libraries ----
### if you want to use the Webots C API in your C++ controller program:
### USE_C_API = true
###
### ---- Debug mode ----
### if you want to display the gcc command line for compilation and link, as
### well as the rm command details used for cleaning:
### VERBOSE = 1
###
###-----------------------------------------------------------------------------

### Do not modify: this includes Webots global Makefile.include
#C_SOURCES = my_plugin.c my_clever_algo.c my_graphics.c
#null :=
#space := $(null) $(null)

# Auto-build files in ./source recursively.

ifeq ($(libgen),y)
CC := g++
CFLAGS := -Wall -Wextra -std=c++11 -fPIC

# Directories
SRC_DIR := src
INC_DIR := inc
OUTPUT_DIR := ./lib/output
# Source files
SRCS := $(shell find $(SRC_DIR) -name '*.cpp')
OBJS := $(patsubst $(SRC_DIR)/%.cpp,$(OUTPUT_DIR)/%.o,$(SRCS))
# Include directories
INC_DIRS := $(shell find $(INC_DIR) -type d)
INC_FLAGS := $(addprefix -I,$(INC_DIRS))
# Targets
TARGET := $(OUTPUT_DIR)/libdiablo.a

# Rules
all: $(TARGET)

$(TARGET): $(OBJS)
	ar rcs $@ $^

$(OUTPUT_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) $(INC_FLAGS) -c $< -o $@

$(info Generate libdiablo)
endif


ifeq ($(libtest),y)
USE_C_API = true
include ./simulation/user.mk

CXXFLAGS += -DDEBUG_IN_SIM -fPIC

C_SOURCES = $(ALLCSRC)
CXX_SOURCES = $(ALLCPPSRC) 

CXXFLAGS += -DDEBUG_IN_SIM

INCLUDE += -I"./inc/middleware/."
INCLUDE += -I"./inc/robot/."
INCLUDE += -I"./inc/controller/."
INCLUDE += -I"./inc/estimator/."
INCLUDE += -I"./inc/planner/."
INCLUDE += -I"./inc/task/."
INCLUDE += -I"./inc/param/."
INCLUDE += -I"./inc/utility/"
INCLUDE += -I"./inc/utility/Math/."
INCLUDE += -I"./lib/."

LIBRARIES += -L"./lib/output" -ldiablo#libyam-cpp.so
# INCLUDE += -I"/usr/include/yaml-cpp"
CXXFLAGS += $(INCLUDE)
$(info CXXFLAGS=$(CXXFLAGS))

WEBOTS_HOME_PATH:=F:/Webots2023a
include $(WEBOTS_HOME_PATH)/resources/Makefile.include

null :=
space := $(null) $(null)
$(info Use libdiablo to control the robot)

else
USE_C_API = true
include ./src/robot/robot.mk
include ./src/middleware/middle.mk
include ./simulation/user.mk
include ./src/task/task.mk
include ./src/planner/planner.mk
include ./src/estimator/estimator.mk
include ./src/controller/controller.mk
include ./src/utility/utility.mk

CXXFLAGS += -DDEBUG_IN_SIM
CXXFLAGS +=  -Wall -Wextra -Wno-reorder -Wno-unused-variable -Wno-unused-parameter  -Wno-missing-field-initializers  -Wno-init-self -Wno-implicit-fallthrough -Wno-switch -Wno-parentheses -Wno-unused-but-set-variable
CXXFLAGS += -Wno-sign-compare -Wno-unused-but-set-parameter
C_SOURCES = $(ALLCSRC)
CXX_SOURCES = $(ALLCPPSRC) 

CXXFLAGS += -DDEBUG_IN_SIM

INCLUDE += -I"./inc/middleware/."
INCLUDE += -I"./inc/robot/."
INCLUDE += -I"./inc/controller/."
INCLUDE += -I"./inc/estimator/."
INCLUDE += -I"./inc/planner/."
INCLUDE += -I"./inc/task/."
INCLUDE += -I"./inc/param/."
INCLUDE += -I"./inc/utility/"
INCLUDE += -I"./inc/utility/Math/."
INCLUDE += -I"./lib/."

CXXFLAGS += $(INCLUDE)
$(info CXXFLAGS=$(CXXFLAGS))

# $(info INCLUDE=$(INCLUDE))

# LIBRARIES += -L"/opt/openrobots/lib" -lyaml-cpp#libyam-cpp.so
# INCLUDE += -I"/usr/include/yaml-cpp"

WEBOTS_HOME_PATH:=/usr/local/webots
include $(WEBOTS_HOME_PATH)/resources/Makefile.include

null :=
space := $(null) $(null)

$(info Use source code to control the robot)
endif

