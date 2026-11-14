"""Make ``vacuum_gui`` importable from the source tree (no install needed for the tests).

Qt is only imported for the widgets, and the tests build them offscreen: a clone with no
display (SSH, CI, a headless box) must not segfault in the Qt platform plugin.
"""

import os
import sys

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
