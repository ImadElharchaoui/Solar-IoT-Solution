CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Werror -Wsign-conversion -Wold-style-cast \
            -Wconversion -Wshadow -Iinclude -Iproto_gen -MMD -MP
LDFLAGS  := -lpaho-mqttpp3 -lpaho-mqtt3as $(shell pkg-config --libs protobuf)

BUILD     := build
SRC       := src
PROTO_DIR := ../proto
GEN_CPP   := proto_gen

TARGETS := $(BUILD)/mppt_parser $(BUILD)/mppt_live $(BUILD)/mppt_mqtt

PROTO_SRC := $(GEN_CPP)/mppt.pb.cc
PROTO_HDR := $(GEN_CPP)/mppt.pb.h
PROTO_OBJ := $(BUILD)/mppt.pb.o

COMMON_SRCS := $(SRC)/eeprom_parser.cpp $(SRC)/json_builder.cpp \
               $(SRC)/mqtt_client.cpp   $(SRC)/space_parser.cpp  \
               $(SRC)/settings_parser.cpp $(SRC)/printer.cpp
COMMON_OBJS := $(patsubst $(SRC)/%.cpp,$(BUILD)/%.o,$(COMMON_SRCS)) $(PROTO_OBJ)

.PHONY: all proto clean distclean release

all: proto $(TARGETS)

proto: $(PROTO_DIR)/mppt.proto
	@mkdir -p $(GEN_CPP)
	protoc --proto_path=$(PROTO_DIR) --cpp_out=$(GEN_CPP) $<
	protoc --proto_path=$(PROTO_DIR) --python_out=$(PROTO_DIR) $<

$(PROTO_OBJ): $(PROTO_SRC)
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(PROTO_SRC): proto

$(BUILD)/%.o: $(SRC)/%.cpp | proto
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD)/mppt_parser: $(BUILD)/main.o $(COMMON_OBJS)
	$(CXX) $^ $(LDFLAGS) -o $@

$(BUILD)/mppt_live: $(BUILD)/main_live.o $(COMMON_OBJS)
	$(CXX) $^ $(LDFLAGS) -o $@

$(BUILD)/mppt_mqtt: $(BUILD)/main_mqtt.o $(COMMON_OBJS)
	$(CXX) $^ $(LDFLAGS) -o $@

release: CXXFLAGS += -O3
release: all

clean:
	rm -rf $(BUILD) $(GEN_CPP)
	rm -f $(PROTO_DIR)/mppt_pb2.py
	@echo "  Cleaned build artefacts and generated files"

distclean: clean

-include $(BUILD)/*.d
