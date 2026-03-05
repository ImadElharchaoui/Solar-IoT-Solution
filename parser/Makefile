CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Wpedantic
SRC_DIR   := .
BUILD_DIR := build

SRCS    := $(wildcard $(SRC_DIR)/*.cpp)
OBJS    := $(patsubst $(SRC_DIR)/%.cpp, $(BUILD_DIR)/%.o, $(SRCS))
TARGET  := $(BUILD_DIR)/phocoslink_parser

DEBUG_FLAGS   := -g3 -O0 -DDEBUG -fsanitize=address,undefined
RELEASE_FLAGS := -O2 -DNDEBUG -flto -s

FLAGS := $(DEBUG_FLAGS)

.PHONY: all release run clean

all: FLAGS := $(DEBUG_FLAGS)
all: $(TARGET)

release: FLAGS := $(RELEASE_FLAGS)
release: $(TARGET)
	@echo "Release binary: $(TARGET)"

$(TARGET): $(OBJS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(FLAGS) -o $@ $^
	@echo "Built: $@"

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(FLAGS) -c $< -o $@

run: all
ifndef LOG
	$(error Usage: make run LOG=<logfile.txt>)
endif
	./$(TARGET) $(LOG)

clean:
	rm -rf $(BUILD_DIR)
	@echo "Cleaned."

