"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

SingleValueConstraint unit tests.
Author: Yoonwoo Kim
"""
import unittest

import numpy as np

import gtsam
from gtsam import DecisionTreeFactor, DiscreteValues
import gtsam_planner
from gtsam_planner import SingleValueConstraint
from gtsam.utils.test_case import GtsamTestCase


class TestSingleValueConstraint(GtsamTestCase):
    """Tests for Single Value Constraints"""

    def setUp(self):
        self.key = (0, 4)
        self.val = 3
        self.constraint = SingleValueConstraint(self.key, self.val)

    def test_operatorTrue(self):
        values = DiscreteValues()
        values[self.key[0]] = 3
        self.assertEqual(self.constraint(values), 1.0)
    
    def test_operatorFalse(self):
        values = DiscreteValues()
        values[self.key[0]] = 2
        self.assertEqual(self.constraint(values), 0.0)
    
    def test_toDecisionTree(self):
        expected = self.constraint.toDecisionTreeFactor()
        self.assertIsInstance(expected, DecisionTreeFactor)
        self.gtsamAssertEquals(DecisionTreeFactor(self.key, "0 0 0 1"), expected)


if __name__ == "__main__":
    unittest.main()
