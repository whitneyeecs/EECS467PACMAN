include ../common.mk

CXXFLAGS = $(CXXFLAGS_STD) \
	$(CFLAGS_VX_GTK) \
	$(CFLAGS_IMAGESOURCE) \
     $(CFLAGS_MATH) \
     $(CFLAGS_COMMON) \
     $(CFLAGS_LCMTYPES) \
     $(CFLAGS_GSL) \
     -O2 -w
LDFLAGS  = $(LDFLAGS_STD) \
	$(LDFLAGS_VX_GTK) \
	  $(LDFLAGS_IMAGESOURCE) \
	  $(LDFLAGS_MATH) \
	  $(LDFLAGS_COMMON) \
		$(LDFLAGS_LCMTYPES) \
		$(LDFLAGS_GSL)

#LIB_MAEBOT_DATA = $(LIB_PATH)/libmaebotdata.a
#LIB_MAEBOT_DATA_OBJ = maebot_data.o

#LIB_LASER_MATCHER = $(LIB_PATH)/lasermatcher.a
#LIB_LASER_MATCHER_OBJ = laser_matcher.o

#LIB_OCCUPANCY_MAP = $(LIB_PATH)/occupancymap.a
#LIB_OCCUPANCY_MAP_OBJ = occupancy_map.o


#LIB_MAPPING = $(LIB_PATH)/libmapping.a

LIB_A2 = $(LIB_PATH)/liba2.a
LIB_A2_OBJ = Node.o Navigation.o

LIBDEPS  = $(call libdeps, $(LDFLAGS))

BIN_NODE = $(BIN_PATH)/Node 
BIN_NAVIGATION = $(BIN_PATH)/Navigation

ALL = $(LIB_A2) $(BIN_NODE) $(BIN_NAVIGATION)

all: $(ALL)

$(LIB_A2): $(LIB_A2_OBJ) $(LIBDEPS)
	@echo "\t$@"
	@ar rc $@ $^

$(BIN_NODE): node.o $(LIB_A2) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_NAVIGATION): navigation.o $(LIB_A2) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
