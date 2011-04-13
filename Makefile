## mathlib makefile

UNAME := $(shell uname)
Q ?= @
SRC = filters.c misc_math.c quat.c spatial_rotations.c xyz.c
LIB = libmathlib.a

OBJ = $(SRC:%.c=%.o) 

WARNINGFLAGS = -Wall -Wextra -Wshadow -Wstrict-prototypes -Werror
DEBUGFLAGS = -g
OPTFLAGS = -O3 -march=native

CFLAGS = $(WARNINGFLAGS) $(DEBUGFLAGS) $(INCLUDES) $(OPTFLAGS) -std=gnu99 
CC = gcc

.PHONY: clean 

mathlib: $(OBJ)
	@echo AR $@
	$(Q)ar rcs $(LIB) $(OBJ)

%.o : %.c
	@echo CC $@
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(LIB)
	rm -f $(OBJ) 
