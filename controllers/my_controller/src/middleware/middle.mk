# Source files located under $(CORE_ROOT) are automatically added.
ifeq ($(MIDDLE_ROOT),)
  MIDDLE_ROOT := ./src/middleware/
endif

rwildcard    = $(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))
MIDDLECSRC    := $(call rwildcard,$(MIDDLE_ROOT),*.c)
MIDDLECPPSRC  := $(call rwildcard,$(MIDDLE_ROOT),*.cpp)
MIDDLEINC     := $(sort $(dir $(call rwildcard,$(MIDDLE_ROOT),*)))

# Shared variables.
ALLCSRC     += $(MIDDLECSRC)
ALLCPPSRC   += $(MIDDLECPPSRC)
ALLINC      += $(MIDDLEINC)
