CXX ?= g++
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra -Werror -fno-exceptions -fno-rtti
LDFLAGS ?=

BUILD_DIR := build
TARGET := $(BUILD_DIR)/main

SRCS := main.cpp io.cpp
OBJS := $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(SRCS))
DEPS := io.h params.h

.PHONY: all clean sanitize

all: $(TARGET)

$(TARGET): $(OBJS) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS) $(LDFLAGS)

$(BUILD_DIR)/%.o: %.cpp $(DEPS) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

sanitize: CXXFLAGS += -fsanitize=address,undefined -g
sanitize: clean $(TARGET)

clean:
	rm -rf $(BUILD_DIR)
