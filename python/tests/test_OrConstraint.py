"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

MultiValueConstraint unit tests.
Author: Yoonwoo Kim
"""
import unittest

import numpy as np

import gtsam
from gtsam import DecisionTreeFactor, DiscreteValues, DiscreteKeys
import gtsam_example
from gtsam_example import OrConstraint
from gtsam.utils.test_case import GtsamTestCase


class TestOrConstraint(GtsamTestCase):
    """Tests for Or Constraints"""

    def setUp(self):
        """
        Create DiscreteKeys, DecisionTreeFactor for AND and OR, and or them together
        """
        self.keys = DiscreteKeys()
        key_list = [(0, 2), (1, 2), (2, 2)]
        for key in reversed(key_list):
            self.keys.push_back(key)
        f_and = DecisionTreeFactor([key_list[0], key_list[1]], "0 0 0 1")
        f_or = DecisionTreeFactor([key_list[1], key_list[2]], "0 1 1 1")
        self.constraint = OrConstraint([f_and, f_or])

    def test_operator(self):
        """
        tests if key(0) = 1, key(1) = 0, and key(2) = 1 gives 1
        for AND or OR constraint
        """
        values = DiscreteValues()
        values[self.keys.at(0)[0]] = 1
        values[self.keys.at(1)[0]] = 0
        values[self.keys.at(2)[0]] = 1
        self.assertEqual(self.constraint(values), 1.0)
    
    def test_toDecisionTree(self):
        """
        Check conversion to decision tree factor, checked manually
        """
        expected = self.constraint.toDecisionTreeFactor()
        self.assertIsInstance(expected, DecisionTreeFactor)
        self.gtsamAssertEquals(DecisionTreeFactor(self.keys, "0 0 1 1 1 1 1 1"), expected)


if __name__ == "__main__":
    unittest.main()
