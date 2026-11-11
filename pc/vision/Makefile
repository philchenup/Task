CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -Wpedantic

.PHONY: test clean

test: plane_level_test
	./plane_level_test

plane_level_test: plane_level.cpp plane_level_test.cpp plane_level.h
	$(CXX) $(CXXFLAGS) -o $@ plane_level.cpp plane_level_test.cpp

clean:
	rm -f plane_level_test
