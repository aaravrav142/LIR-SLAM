# Makefile used to build mex files

TARGETS = all clean
.PHONY: $(TARGETS)

ifndef OSTYPE
  OSTYPE = $(shell uname -s|awk '{print tolower($$0)}')
  #export OSTYPE
endif

ifeq ($(OSTYPE),linux)
  MEXSUFFIX = mexglx
endif
ifeq ($(OSTYPE),darwin)
  MEXSUFFIX = mexmaci
endif

MEX = mex
CXXFLAGS = -O 

all: map_correlation \
			connected_regions \
			cjpeg \
			djpeg \
			time \
			dijkstra_graph \
			dijkstra_matrix \
			dijkstra_nonholonomic16 \
			astar_graph \
			astar_nonholonomic16 \
			subs_interp3_circular \
			subsinterp 

subs_interp3_circular: subs_interp3_circular.cc
	$(MEX) $(CXXFLAGS) $^

subsinterp:	subs_interp.cc
	$(MEX) $(CXXFLAGS) $^ 

dijkstra_graph: dijkstra_graph.cc
	$(MEX) $(CXXFLAGS) $^

dijkstra_matrix: dijkstra_matrix.cc
	$(MEX) $(CXXFLAGS) $^

dijkstra_nonholonomic16: dijkstra_nonholonomic16.cc
	$(MEX) $(CXXFLAGS) $^

astar_graph: astar_graph.cc
	$(MEX) $(CXXFLAGS) $^

astar_nonholonomic16: astar_nonholonomic16.cc
	$(MEX) $(CXXFLAGS) $^

zlibCompress: zlibCompress.cc 
	$(MEX) $(CXXFLAGS) $^

zlibUncompress: zlibUncompress.cc
	$(MEX) $(CXXFLAGS) $^

map_correlation: map_correlation.cpp 
	$(MEX) $(CXXFLAGS) $^

uvcCam: uvcCam.cpp v4l2.cpp
	$(MEX) $(CXXFLAGS) $^

cjpeg: cjpeg.cpp
	$(MEX) $(CXXFLAGS) $^ -ljpeg

djpeg: djpeg.cpp
	$(MEX) $(CXXFLAGS) $^ -ljpeg

connected_regions: connected_regions.cc RegionProps.cc ConnectRegions.cc
	$(MEX) $(CXXFALGS) $^ 

time: time.cc timeScalar.cc
	$(MEX) $(CXXFLAGS)  $^

clean:
	rm -f *.$(MEXSUFFIX) *.o
