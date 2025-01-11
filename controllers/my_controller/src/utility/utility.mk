# Source files located under $(CORE_ROOT) are automatically added.
ifeq ($(Utility_ROOT),)
  Utility_ROOT := ./src/utility/
endif

rwildcard    = $(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))
UtilityCSRC    := $(call rwildcard,$(Utility_ROOT),*.c)
UtilityCPPSRC  := $(call rwildcard,$(Utility_ROOT),*.cpp)
UtilityINC     := $(sort $(dir $(call rwildcard,$(Utility_ROOT),*)))

# Shared variables.
ALLCSRC     += $(UtilityCSRC)
ALLCPPSRC   += $(UtilityCPPSRC)
ALLINC      += $(UtilityINC)
