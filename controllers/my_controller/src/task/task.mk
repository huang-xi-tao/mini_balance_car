# Source files located under $(CORE_ROOT) are automatically added.
ifeq ($(Task_ROOT),)
  Task_ROOT := ./src/task/
endif

rwildcard    = $(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))
TaskCSRC    := $(call rwildcard,$(Task_ROOT),*.c)
TaskCPPSRC  := $(call rwildcard,$(Task_ROOT),*.cpp)
TaskINC     := $(sort $(dir $(call rwildcard,$(Task_ROOT),*)))

# Shared variables.
ALLCSRC     += $(TaskCSRC)
ALLCPPSRC   += $(TaskCPPSRC)
ALLINC      += $(TaskINC)
