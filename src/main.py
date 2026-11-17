#!/usr/bin/env python3
"""Shim: delegates to root main.py"""
import sys, subprocess
from pathlib import Path
root_main = Path(__file__).resolve().parent.parent / "main.py"
sys.exit(subprocess.run([sys.executable, str(root_main)] + sys.argv[1:]).returncode)
