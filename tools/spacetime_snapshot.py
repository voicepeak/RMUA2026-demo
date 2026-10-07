#!/usr/bin/env python3
"""Compatibility import for the installed, shared frozen snapshot codec."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros_ws/src/route_follower/scripts'))
from planning_snapshot_io import read_snapshot,write_snapshot
