import os
import sys

# let `pytest src/vacuum_coverage/test` work from any directory without installing the package
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
