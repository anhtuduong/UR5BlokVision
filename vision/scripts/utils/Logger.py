"""!
@file Logger.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@brief Utility functions for logging
@date 2023-05-04
"""

# Resolve paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[3]
if ROOT not in sys.path:
    sys.path.append(ROOT)  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

# Global constants
LOG_PATH = os.path.abspath(os.path.join(ROOT, "vision/logs"))

# Class Logger
class Logger:
    """!
    @brief This class contains utility functions for logging.
    """
    def info(text):
        print(f"[INFO]\t{text}")

    def debug(text):
        print(f"[DEBUG]\t{text}")

    def error(text):
        print(f"[ERROR]\t{text}")

    def warning(text):
        print(f"[WARNING]\t{text}")
    
    # TODO: implement log to file