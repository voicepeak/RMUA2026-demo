#!/usr/bin/env python3
"""Compatibility entry; all seeds use the same generic race launcher."""
from pathlib import Path
import runpy

if __name__=='__main__':
    runpy.run_path(str(Path(__file__).with_name('start_race.py')),run_name='__main__')
