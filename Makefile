.PHONY: all clean test sanitize

all:
	$(MAKE) -C src

clean:
	$(MAKE) -C src clean

test:
	$(MAKE) -C src test

sanitize:
	$(MAKE) -C src sanitize
