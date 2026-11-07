CXX ?= g++
CXXFLAGS ?= -O3 -std=c++17 -Wall -Wextra -Werror -fno-exceptions -fno-rtti

.PHONY: all clean test sanitize

all:
	$(MAKE) -C cpp
	cp -f cpp/build/main ./main
	cp -f cpp/build/main ../main 2>/dev/null || true

clean:
	$(MAKE) -C cpp clean
	rm -f ./main ../main 2>/dev/null || true

test:
	$(MAKE) -C cpp test

sanitize:
	$(MAKE) -C cpp sanitize
