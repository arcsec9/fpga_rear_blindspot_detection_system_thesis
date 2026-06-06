CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++17 -Iesp32

TARGET = fw_app

# Find all cpp files under esp32
SRC = $(shell find esp32 -name "*.cpp")
OBJ = $(SRC:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJ)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)

run: $(TARGET)
	./$(TARGET)