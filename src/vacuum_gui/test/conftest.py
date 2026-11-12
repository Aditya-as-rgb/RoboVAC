"""Make ``vacuum_gui`` importable from the source tree (no install needed for the tests)."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
