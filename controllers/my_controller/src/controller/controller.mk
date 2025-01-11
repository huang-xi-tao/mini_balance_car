# Source files located under $(CORE_ROOT) are automatically added.
ifeq ($(Controller_ROOT),)
  Controller_ROOT := ./src/controller/
endif

rwildcard    = $(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))
ControllerCSRC    := $(call rwildcard,$(Controller_ROOT),*.c)
ControllerCPPSRC  := $(call rwildcard,$(Controller_ROOT),*.cpp)
ControllerINC     := $(sort $(dir $(call rwildcard,$(Controller_ROOT),*)))

# Shared variables.
ALLCSRC     += $(ControllerCSRC)
ALLCPPSRC   += $(ControllerCPPSRC)
ALLINC      += $(ControllerINC)
