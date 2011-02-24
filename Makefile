## Mathlib Makefile
## Matt Peddie

UNAME := $(shell uname)
Q ?= @
SRC = filters.c misc_math.c quat.c spatial_rotations.c xyz.c

AP_PROJECT_DIR ?= ../
CONFTRON_DIR ?= $(AP_PROJECT_DIR)/conftron

OBJ = $(SRC:%.c=%.o) 

## Compile pedantically and save pain later
WARNINGFLAGS ?= -Wall -Wextra -Werror 
DEBUGFLAGS ?= -g -DDEBUG # -pg to generate profiling information
FEATUREFLAGS ?= 
OPTFLAGS ?= 
INCLUDES ?= 
LDFLAGS ?= -lm

## thanks so much, Steve Jobs.  
ifeq ($(UNAME),Darwin)
	LDFLAGS += -L/opt/local/lib
	LDFLAGS += -L/usr/local/lib
	INCLUDES += -I/opt/local/include
	INCLUDES += -isystem /usr/local/include
else
	OPTFLAGS += -march=native -O3
endif

include $(CONFTRON_DIR)/includes

LDFLAGS += $(WARNINGFLAGS)
CFLAGS ?= $(WARNINGFLAGS) $(DEBUGFLAGS) $(FEATUREFLAGS) $(INCLUDES) $(OPTFLAGS) -std=gnu99 
CC ?= gcc

.PHONY: clean 

all: $(OBJ)

%.o : %.c
	@echo CC $@
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

conftron:
	$(MAKE) -C $(CONFTRON_DIR) AIRCRAFT=$(AIRCRAFT)

conf: conftron
config: conftron

clean:
	rm -f $(OBJ) 

megaclean: clean
	$(MAKE) -C $(CONFTRON_DIR) clean
