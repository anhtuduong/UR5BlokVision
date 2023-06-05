"""!
@package tests.test_logger
@file tests/test_logger.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@brief This module contains the unit tests for the logger module.
@date 2023-05-04
"""

# Resolve paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

# Import
import unittest
from utilities.Logger import Logger

class TestLogger(unittest.TestCase):
    """!
    @brief This class contains the unit tests for the logger module.
    """

    def test_logger(self):
        """!
        @brief This method tests the info method.
        """
        assert Logger.info("test") == "[INFO] test"
        assert Logger.warning("test") == "[WARNING] test"
        assert Logger.error("test") == "[ERROR] test"
        assert Logger.debug("test") == "[DEBUG] test"
        