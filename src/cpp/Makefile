CXX ?= g++
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra -Werror -fno-exceptions -fno-rtti
LDFLAGS ?=

BUILD_DIR := build
TARGET := $(BUILD_DIR)/main

SRCS := main.cpp io.cpp
OBJS := $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(SRCS))
DEPS := io.h params.h mat.h quat.h

REPO_ROOT := $(abspath ../..)
TEST_SRC := $(REPO_ROOT)/tests/cpp/test_math.cpp
TEST_TARGET := $(BUILD_DIR)/test_math

.PHONY: all clean sanitize test

all: $(TARGET)

$(TARGET): $(OBJS) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS) $(LDFLAGS)

$(BUILD_DIR)/%.o: %.cpp $(DEPS) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

test: $(TEST_TARGET)
	./$(TEST_TARGET)

$(TEST_TARGET): $(TEST_SRC) mat.h quat.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I. -I$(REPO_ROOT) -o $@ $< $(LDFLAGS)

sanitize: CXXFLAGS += -fsanitize=address,undefined -g
sanitize: clean $(TARGET)

clean:
	rm -rf $(BUILD_DIR)
