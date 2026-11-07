.PHONY: all clean test sanitize

all:
	$(MAKE) -C src/cpp

clean:
	$(MAKE) -C src/cpp clean

test:
	$(MAKE) -C src/cpp test

sanitize:
	$(MAKE) -C src/cpp sanitize
