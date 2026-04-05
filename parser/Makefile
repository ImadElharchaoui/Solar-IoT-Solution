CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Werror -Wsign-conversion -Wold-style-cast -Wconversion -Wshadow -Iinclude -MMD -MP
LDFLAGS  :=

BUILD := build

TARGET_PARSER := $(BUILD)/mppt_parser
TARGET_LIVE   := $(BUILD)/mppt_live
TARGET_MQTT   := $(BUILD)/mppt_mqtt

SHARED_SRCS := src/space_parser.cpp  \
               src/eeprom_parser.cpp \
               src/settings_parser.cpp \
               src/printer.cpp       \
               src/json_builder.cpp

MQTT_SRCS := src/mqtt_client.cpp

SHARED_OBJS := $(patsubst src/%.cpp, $(BUILD)/%.o, $(SHARED_SRCS))
MQTT_OBJS   := $(patsubst src/%.cpp, $(BUILD)/%.o, $(MQTT_SRCS))

ALL_OBJS := $(SHARED_OBJS) $(MQTT_OBJS) $(BUILD)/main.o $(BUILD)/main_live.o $(BUILD)/main_mqtt.o
DEPS     := $(ALL_OBJS:.o=.d)

MQTT_LIBS := -lpaho-mqttpp3 -lpaho-mqtt3as

JSON_HPP := include/json.hpp
JSON_URL := https://github.com/nlohmann/json/releases/download/v3.12.0/json.hpp

.PHONY: all release debug clean distclean

all: release

release: CXXFLAGS += -O2 -DNDEBUG
release: LDFLAGS  += -s
release: $(JSON_HPP) $(TARGET_PARSER) $(TARGET_LIVE) $(TARGET_MQTT)

debug: CXXFLAGS += -O0 -g -fsanitize=address,undefined
debug: LDFLAGS  += -fsanitize=address,undefined
debug: $(JSON_HPP) $(TARGET_PARSER) $(TARGET_LIVE) $(TARGET_MQTT)

$(TARGET_PARSER): $(BUILD)/main.o $(SHARED_OBJS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $^ -o $@
	@echo "  Linked  $@"

$(TARGET_LIVE): $(BUILD)/main_live.o $(SHARED_OBJS) $(MQTT_OBJS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $^ $(MQTT_LIBS) -o $@
	@echo "  Linked  $@"

$(TARGET_MQTT): $(BUILD)/main_mqtt.o $(SHARED_OBJS) $(MQTT_OBJS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $^ $(MQTT_LIBS) -o $@
	@echo "  Linked  $@"

-include $(DEPS)

$(BUILD)/%.o: src/%.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) -c $< -o $@
	@echo "  CC      $<"

$(JSON_HPP):
	@echo "  Fetching json.hpp..."
	curl -fL $(JSON_URL) -o $@

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)
	@echo "  Cleaned $(BUILD)/"

distclean: clean
	rm -f $(JSON_HPP)
	@echo "  Removed $(JSON_HPP)"
