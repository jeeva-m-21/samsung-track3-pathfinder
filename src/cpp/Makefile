CXX ?= g++
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra -Werror -fno-exceptions -fno-rtti
LDFLAGS ?=

BUILD_DIR := build
TARGET := $(BUILD_DIR)/main

SRCS := main.cpp io.cpp init.cpp eskf.cpp
OBJS := $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(SRCS))
DEPS := io.h params.h mat.h quat.h init.h eskf.h

REPO_ROOT := $(abspath ../..)
TEST_MATH_SRC := $(REPO_ROOT)/tests/cpp/test_math.cpp
TEST_MATH_TARGET := $(BUILD_DIR)/test_math

TEST_ESKF_SRC := $(REPO_ROOT)/tests/cpp/test_eskf.cpp
TEST_ESKF_TARGET := $(BUILD_DIR)/test_eskf

.PHONY: all clean sanitize test

all: $(TARGET)

$(TARGET): $(OBJS) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS) $(LDFLAGS)

$(BUILD_DIR)/%.o: %.cpp $(DEPS) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

test: $(TEST_MATH_TARGET) $(TEST_ESKF_TARGET)
	./$(TEST_MATH_TARGET)
	./$(TEST_ESKF_TARGET)

$(TEST_MATH_TARGET): $(TEST_MATH_SRC) mat.h quat.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I. -I$(REPO_ROOT) -o $@ $< $(LDFLAGS)

$(TEST_ESKF_TARGET): $(TEST_ESKF_SRC) $(BUILD_DIR)/init.o $(BUILD_DIR)/eskf.o $(DEPS) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I. -I$(REPO_ROOT) -o $@ $(TEST_ESKF_SRC) $(BUILD_DIR)/init.o $(BUILD_DIR)/eskf.o $(LDFLAGS)

sanitize: CXXFLAGS += -fsanitize=address,undefined -g
sanitize: clean $(TARGET) $(TEST_MATH_TARGET) $(TEST_ESKF_TARGET)

clean:
	rm -rf $(BUILD_DIR)
