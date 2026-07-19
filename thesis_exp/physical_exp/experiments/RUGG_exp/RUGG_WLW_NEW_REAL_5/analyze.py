#!/usr/bin/env python3
"""Run this experiment only. Edit analyze_impl.py for trial-specific changes."""
from pathlib import Path
import os
import runpy
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
sys.path.insert(0, str(ROOT / "common"))
os.chdir(HERE)
sys.argv = [str(HERE / "analyze_impl.py"), "--exp", "RUGG_WLW_NEW_REAL_5", *sys.argv[1:]]
runpy.run_path(str(HERE / "analyze_impl.py"), run_name="__main__")
