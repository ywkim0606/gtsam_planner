"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

NullOperatorConstraint unit tests.
Author: Yoonwoo Kim
"""
import unittest

import numpy as np

import gtsam
from gtsam import DecisionTreeFactor, DiscreteValues, DiscreteKeys
import gtsam_planner
from gtsam_planner import NullConstraint
from gtsam.utils.test_case import GtsamTestCase


class TestNullOperatorConstraint(GtsamTestCase):
    """Tests for MutexConstraints"""

    def setUp(self):
        """
        Say first two variables are variables for state_t and second two variables
        are variables for state_t+1
        """
        self.keys = DiscreteKeys()
        key_list = [(0, 2), (1, 2), (2, 2), (3, 2)]
        for key in key_list:
            self.keys.push_back(key)
        self.constraint = NullConstraint(self.keys)

    def test_operatorTrue0(self):
        """
        Checks if factor returns 1.0 when variables in state_t and state_t+1 have
        same values
        """
        values = DiscreteValues()
        values[self.keys.at(0)[0]] = 0
        values[self.keys.at(1)[0]] = 1
        values[self.keys.at(2)[0]] = 0
        values[self.keys.at(3)[0]] = 1
        self.assertEqual(self.constraint(values), 1.0)
    
    def test_operatorTrue1(self):
        """
        Checks if factor returns 1.0 when variables in state_t and state_t+1 have
        same values
        """
        values = DiscreteValues()
        values[self.keys.at(0)[0]] = 1
        values[self.keys.at(1)[0]] = 0
        values[self.keys.at(2)[0]] = 1
        values[self.keys.at(3)[0]] = 0
        self.assertEqual(self.constraint(values), 1.0)
    
    def test_operatorFalse0(self):
        """
        Checks if factor returns 0.0 when variables in state_t and state_t+1 have 
        different values
        """
        values = DiscreteValues()
        values[self.keys.at(0)[0]] = 1
        values[self.keys.at(1)[0]] = 0
        values[self.keys.at(2)[0]] = 0
        values[self.keys.at(3)[0]] = 1
        self.assertEqual(self.constraint(values), 0.0)
    
    def test_operatorFalse1(self):
        """
        Checks if factor returns 0.0 when variables in state_t and state_t+1 have 
        different values
        """    
        values = DiscreteValues()
        values[self.keys.at(0)[0]] = 1
        values[self.keys.at(1)[0]] = 0
        values[self.keys.at(2)[0]] = 0
        values[self.keys.at(3)[0]] = 0
        self.assertEqual(self.constraint(values), 0.0)
    
    def test_operatorMoreVal(self):
        """
        Checks if factor functions as expected when there are more values than what
        are considered in constraint
        """    
        values = DiscreteValues()
        values[self.keys.at(0)[0]] = 0
        values[self.keys.at(1)[0]] = 0
        values[self.keys.at(2)[0]] = 0
        values[self.keys.at(3)[0]] = 0
        values[4] = 0
        values[5] = 0
        self.assertEqual(self.constraint(values), 1.0)
        self.assertEqual(self.constraint.toDecisionTreeFactor()(values), 1.0)

    def test_toDecisionTree(self):
        """
        Tests if factor can be transformed to decision tree factor
        Checked manually.
        """
        expected = self.constraint.toDecisionTreeFactor()        
        self.assertIsInstance(expected, DecisionTreeFactor)
        self.gtsamAssertEquals(DecisionTreeFactor(self.keys, "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"), expected)


if __name__ == "__main__":
    unittest.main()
