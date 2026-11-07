---
name: run-diff-test
description: Compare Python reference and C++ binary outputs numerically. Use after any change in src/cpp or src/ref.
---
1. Build: `make -C src/cpp`.
2. For each file in tests/golden/inputs/: run `python -m ref <in> /tmp/py.txt` and
   `src/cpp/build/main <in> /tmp/cpp.txt`.
3. Run `python scripts/diff_py_cpp.py /tmp/py.txt /tmp/cpp.txt` and report max abs
   position diff, max quaternion component diff, first row exceeding tolerance.
