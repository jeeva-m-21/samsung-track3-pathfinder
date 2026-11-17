.PHONY: all clean test release

all:
	$(MAKE) -C src

release:
	$(MAKE) -C src release

clean:
	$(MAKE) -C src clean
	@rm -rf __pycache__ *.pyc 2>/dev/null || true

test:
	$(MAKE) -C src test
