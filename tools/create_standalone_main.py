#!/usr/bin/env python3
"""Assembles all components into a single standalone main.py script."""

import json
import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

# Read default params
with open(REPO_ROOT / "configs" / "params.json", "r", encoding="utf-8") as f:
    default_params = json.load(f)

# List of files in topological dependency order
MODULES = [
    "mathlib.py",
    "parser.py",
    "init.py",
    "strapdown.py",
    "eskf.py",
    "updates.py",
    "monitor.py",
    "gap.py",
    "blind.py",
    "baseline_naive.py",
    "baseline_mahony.py",
    "pipeline.py",
]

HEADER = f'''#!/usr/bin/env python3
"""Track 03 (Path Finder) Standalone Single-File Submission.

Self-contained pure-Python 15-State Error-State Kalman Filter (ESKF)
with heavy-tailed Student-t covariance updates, adaptive dynamic noise scaling,
regime-aware velocity bounds, zero-velocity updates, dropout gap bridge,
and blind dead-reckoning predictor.

Zero external dependencies beyond Python standard library and NumPy.
Memory profile: <= 4.0 MB heap via chunked stream processing.

Usage:
    python main.py <input.csv|input.zip|dir> [output.txt]
    python main.py <input.csv> [output.txt] [--ablation naive|mahony|eskf_basic|eskf_full] [--append-blind]
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import os
import sys
import tempfile
import time
import zipfile
from dataclasses import dataclass, field
from enum import Enum, IntFlag
from pathlib import Path
from typing import Any, Dict, Iterable, Iterator, List, NamedTuple, Optional, TextIO, Tuple, Union

import numpy as np

# Embedded default parameters (no external params.json file required)
DEFAULT_PARAMS: Dict[str, Any] = {repr(default_params)}


def get_default_params_path() -> Path:
    """Return path to configs/params.json if present, else fallback."""
    cand = Path(__file__).resolve().parent / "configs" / "params.json"
    return cand


def load_params(params: Optional[Union[Dict[str, Any], str, Path]] = None) -> Dict[str, Any]:
    """Load parameter dictionary or return default parameters."""
    if params is None:
        return dict(DEFAULT_PARAMS)
    if isinstance(params, (str, Path)):
        p = Path(params)
        if p.is_file():
            with open(p, "r", encoding="utf-8") as f:
                return json.load(f)
        return dict(DEFAULT_PARAMS)
    return dict(params)
'''

CLI_CODE = '''
# ==============================================================================
# Standalone CLI and Input Resolution
# ==============================================================================

def _is_zip(p: Path) -> bool:
    try:
        with open(p, "rb") as f:
            return f.read(2) == b"PK"
    except Exception:
        return False


def _extract_zip_csv(p: Path) -> str:
    tmp = tempfile.mkdtemp(prefix="imu_")
    with zipfile.ZipFile(p) as z:
        names = z.namelist()
        candidates = ["imu0/data.csv", "mav0/imu0/data.csv", "data.csv"]
        chosen = None
        for cand in candidates:
            matches = [n for n in names if n.endswith(cand)]
            if matches:
                chosen = matches[0]
                break
        if chosen is None:
            matches = [n for n in names if n.endswith(".csv")]
            if matches:
                chosen = matches[0]
        if chosen is None:
            raise FileNotFoundError(f"No CSV found inside zip: {p}")
        out = os.path.join(tmp, "data.csv")
        with z.open(chosen) as src, open(out, "wb") as dst:
            dst.write(src.read())
    return out


def resolve_imu_csv(raw: Union[str, Path]) -> str:
    p = Path(raw)
    if not p.exists():
        raise FileNotFoundError(f"Input not found: {raw}")
    if p.is_dir():
        for cand in [p / "imu0" / "data.csv", p / "mav0" / "imu0" / "data.csv", p / "data.csv"]:
            if cand.is_file():
                return str(cand)
        raise FileNotFoundError(f"No IMU CSV in directory: {raw}")
    if p.is_file() and (str(raw).lower().endswith(".zip") or _is_zip(p)):
        return _extract_zip_csv(p)
    return str(p)


class _CustomParser(argparse.ArgumentParser):
    def error(self, message):
        sys.stderr.write(f"error: {message}\\n")
        sys.exit(1)


def main():
    parser = _CustomParser(description="Track 03 Path Finder Estimator")
    parser.add_argument("input", nargs="?", default=None, help="Input IMU CSV, ZIP, or directory")
    parser.add_argument("output", nargs="?", default="estimated_trajectory.txt", help="Output trajectory path")
    parser.add_argument("--append-blind", action="store_true", help="Extrapolate blind window after stream")
    parser.add_argument(
        "--ablation",
        type=str,
        default="eskf_full",
        choices=["naive", "mahony", "eskf_basic", "eskf_full"],
        help="Ablation mode",
    )
    parser.add_argument("--params", type=str, default=None, help="Custom params JSON")

    args = parser.parse_args()
    if not args.input:
        parser.print_usage(sys.stderr)
        sys.exit(1)

    try:
        csv_path = resolve_imu_csv(args.input)
        run_pipeline(
            csv_path,
            args.output,
            ablation=args.ablation,
            append_blind=args.append_blind,
            params=args.params,
        )
    except Exception as e:
        sys.stderr.write(f"Execution failed: {e}\\n")
        sys.exit(1)


if __name__ == "__main__":
    main()
'''


def clean_source(code: str, mod_name: str) -> str:
    # 1. Remove multi-line from-imports
    code = re.sub(r'from (?:src\.ref|tools)\.[a-zA-Z0-9_.]+\s+import\s*\([^)]*\)', '', code)
    # 2. Remove single-line from-imports
    code = re.sub(r'from (?:src\.ref|tools)\.[a-zA-Z0-9_.]+\s+import\s+[^\n]+', '', code)

    # 3. For baseline_naive and baseline_mahony, rename compute_initial_tilt to _compute_initial_tilt_and_g
    if mod_name in ("baseline_naive.py", "baseline_mahony.py"):
        code = code.replace("compute_initial_tilt(a_mean:", "_compute_initial_tilt_and_g(a_mean:")
        code = code.replace("compute_initial_tilt(a_init_mean)", "_compute_initial_tilt_and_g(a_init_mean)")

    # 4. Remove duplicate definitions of load_params and get_default_params_path
    if mod_name in ("parser.py", "init.py", "blind.py", "eskf.py", "gap.py"):
        code = re.sub(r'def get_default_params_path\(\)[^:]*:.*?(?=\ndef |\nclass |\n[A-Z_]+ =|\Z)', '', code, flags=re.DOTALL)
        code = re.sub(r'def load_params\([^)]*\)[^:]*:.*?(?=\ndef |\nclass |\n[A-Z_]+ =|\Z)', '', code, flags=re.DOTALL)

    # 5. Remove standalone if __name__ == '__main__': blocks in helper files
    if mod_name not in ("pipeline.py",):
        code = re.sub(r'if __name__ == ["\']__main__["\']:\s*main\(\)', '', code)
        code = re.sub(r'def main\(\)[^:]*:.*?(?=\ndef |\nclass |\Z)', '', code, flags=re.DOTALL)

    lines = code.splitlines()
    cleaned = []
    for line in lines:
        stripped = line.strip()
        # Remove standard duplicate imports
        if stripped in (
            "import os",
            "import sys",
            "import math",
            "import json",
            "import tempfile",
            "import time",
            "import zipfile",
            "import argparse",
            "import itertools",
            "import numpy as np",
            "from typing import Any, Dict, Iterable, Iterator, List, NamedTuple, Optional, TextIO, Tuple, Union",
            "from typing import Any, Dict, List, NamedTuple, Optional, Tuple, Union",
            "from typing import Any, Dict, List, Optional, Tuple, Union",
            "from typing import Any, Dict, NamedTuple, Optional, Tuple, Union",
            "from typing import Any, Dict, Optional, Tuple, Union",
            "from typing import List, Tuple, Union",
            "from typing import Tuple, Union",
            "from typing import Union",
            "from enum import Enum, IntFlag",
            "from enum import Enum",
            "from enum import IntFlag",
            "from dataclasses import dataclass",
            "from dataclasses import dataclass, field",
            "from pathlib import Path",
            "from __future__ import annotations",
        ):
            continue

        cleaned.append(line)

    result = "\n".join(cleaned)
    return f"\n# ==============================================================================\n# Module: {mod_name}\n# ==============================================================================\n" + result


def assemble():
    parts = [HEADER]
    for mod in MODULES:
        p = REPO_ROOT / "src" / "ref" / mod
        with open(p, "r", encoding="utf-8") as f:
            code = f.read()
        parts.append(clean_source(code, mod))

    parts.append(CLI_CODE)
    full_code = "\n".join(parts)

    out_p = REPO_ROOT / "main.py"
    with open(out_p, "w", encoding="utf-8") as f:
        f.write(full_code)
    print(f"Assembled standalone main.py: {len(full_code.splitlines())} lines.")


if __name__ == "__main__":
    assemble()
