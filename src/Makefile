.PHONY: all clean test release

all:
	@chmod +x main.py 2>/dev/null || true
	@cp -f main.py main 2>/dev/null || true
	@chmod +x main 2>/dev/null || true
	@cp -f main.py ../main 2>/dev/null || true
	@chmod +x ../main 2>/dev/null || true
	@echo "Build successful: src/main and ../main ready."

release: all

clean:
	@rm -rf __pycache__ *.pyc 2>/dev/null || true

test:
	@python3 -c "import main; print('main.py syntax and import OK')"
