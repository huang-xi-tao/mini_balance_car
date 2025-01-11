# Source files located under $(CORE_ROOT) are automatically added.
ifeq ($(Planner_ROOT),)
  Planner_ROOT := ./src/planner/
endif

rwildcard    = $(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))
PlannerCSRC    := $(call rwildcard,$(Planner_ROOT),*.c)
PlannerCPPSRC  := $(call rwildcard,$(Planner_ROOT),*.cpp)
PlannerINC     := $(sort $(dir $(call rwildcard,$(Planner_ROOT),*)))

# Shared variables.
ALLCSRC     += $(PlannerCSRC)
ALLCPPSRC   += $(PlannerCPPSRC)
ALLINC      += $(PlannerINC)
