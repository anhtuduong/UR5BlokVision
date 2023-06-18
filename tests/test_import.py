"""!
@package tests.test_import
@file tests/test_import.py
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
    print(sys.path)

# Import
import unittest

class TestImport(unittest.TestCase):
    """!
    @brief This class contains the unit tests for the imports.
    """

    def test_import_motion(self):
        """!
        @brief This method tests the import of motion_ur5 package.
        """
        from motion_ur5.src.test import test_root
        assert test_root() == ROOT
        
    def test_import_vision(self):
        """!
        @brief This method tests the import of vision_ur5 package.
        """
        from vision_ur5.src.test import test_root
        assert test_root() == ROOT


if __name__ == '__main__':
    unittest.main()