CXX = g++
CXXFLAGS = -std=c++20 -Iinc -Wall -Wextra -O2
SRC = src/serial.cpp src/feetech.cpp src/kinematics.cpp src/main.cpp
OBJ = $(SRC:.cpp=.o)
TARGET = lekiwi-wasd
all: $(TARGET)
$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<
clean:
	rm -f $(OBJ) $(TARGET)
run: $(TARGET)
	sudo ./$(TARGET)
.PHONY: all clean run
