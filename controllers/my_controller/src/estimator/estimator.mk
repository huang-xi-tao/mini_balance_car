# Source files located under $(CORE_ROOT) are automatically added.
ifeq ($(Estimator_ROOT),)
  Estimator_ROOT := ./src/estimator/
endif

rwildcard    = $(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))
EstimatorCSRC    := $(call rwildcard,$(Estimator_ROOT),*.c)
EstimatorCPPSRC  := $(call rwildcard,$(Estimator_ROOT),*.cpp)
EstimatorINC     := $(sort $(dir $(call rwildcard,$(Estimator_ROOT),*)))

# Shared variables.
ALLCSRC     += $(EstimatorCSRC)
ALLCPPSRC   += $(EstimatorCPPSRC)
ALLINC      += $(EstimatorINC)
