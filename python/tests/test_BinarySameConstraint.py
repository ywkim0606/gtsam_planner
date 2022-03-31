"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

BinarySameConstraint unit tests.
Author: Yoonwoo Kim
"""
import unittest

import numpy as np

import gtsam
from gtsam import DecisionTreeFactor, DiscreteValues
import gtsam_planner
from gtsam_planner import BinarySameConstraint
from gtsam.utils.test_case import GtsamTestCase


class TestBinarySameConstraint(GtsamTestCase):
    """Tests for BinarySameConstraints"""

    def setUp(self):
        self.key0 = (0, 2)
        self.key1 = (1, 2)
        self.constraint = BinarySameConstraint(self.key0, self.key1)

    def test_operatorSameVal(self):
        """Checks if factor returns 1.0 when two variables have the same value"""
        values = DiscreteValues()
        values[self.key0[0]] = 3
        values[self.key1[0]] = 3
        self.assertEqual(self.constraint(values), 1.0)
    
    def test_operatorDiffVal(self):
        """Checks if factor returns 0.0 when two variables have the different value"""
        values = DiscreteValues()
        values[self.key0[0]] = 2
        values[self.key1[0]] = 3
        self.assertEqual(self.constraint(values), 0.0)
    
    def test_toDecisionTree(self):
        """Tests if factor can be transformed to decision tree factor"""
        expected = self.constraint.toDecisionTreeFactor()
        self.assertIsInstance(expected, DecisionTreeFactor)
        self.gtsamAssertEquals(DecisionTreeFactor([self.key0, self.key1], "1 0 0 1"), expected)


if __name__ == "__main__":
    unittest.main()
