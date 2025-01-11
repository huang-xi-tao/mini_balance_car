# Source files located under $(CORE_ROOT) are automatically added.
ifeq ($(CORE_ROOT),)
  CORE_ROOT := ./src/robot/
endif

rwildcard    = $(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))
CORECSRC    := $(call rwildcard,$(CORE_ROOT),*.c)
CORECPPSRC  := $(call rwildcard,$(CORE_ROOT),*.cpp)
COREINC     := $(sort $(dir $(call rwildcard,$(CORE_ROOT),*)))

# Shared variables.
ALLCSRC     += $(CORECSRC)
ALLCPPSRC   += $(CORECPPSRC)
ALLINC      += $(COREINC)
