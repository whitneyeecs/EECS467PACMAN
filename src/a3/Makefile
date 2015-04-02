include ../common.mk

# flags for building the gtk library
CFLAGS = $(CFLAGS_STD) -O3 -DNDEBUG
CXXFLAGS = $(CXXFLAGS_STD) -O3 -DNDEBUG
LDFLAGS = $(LDFLAGS_STD) 

LIBDEPS = $(call libdeps, $(LDFLAGS))

LIB_A3 = $(LIB_PATH)/liba3.a
LIB_A3_OBJS = navigation.o map.o \

ALL = $(LIB_A3)

all: $(ALL)

$(LIB_A3): $(LIB_A3_OBJS) $(LIBDEPS)
	@echo "\t$@"
	@ar rc $@ $^

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
