"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

FrameConstraint unit tests.
Author: Yoonwoo Kim
"""
import unittest

import numpy as np

import gtsam
from gtsam import DecisionTreeFactor, DiscreteValues, DiscreteKeys
import gtsam_planner
from gtsam_planner import FrameConstraint, NullConstraint
from gtsam.utils.test_case import GtsamTestCase


class TestFrameConstraint(GtsamTestCase):
    """
    Tests for FrameConstraint which constrains frame variables except for
    the ones affected by the operator.
    """

    def setUp(self):
        self.keys0 = gtsam.DiscreteKeys()
        self.keys1 = gtsam.DiscreteKeys()
        all_keys = gtsam.DiscreteKeys()
        self.single_key = (5, 2)
        key_list = [(0, 2), (1, 2), (2, 2), (3, 2), (4, 2)]

        for key in key_list:
            all_keys.push_back(key)

        for key in key_list:
            if key[0] < 4:
                self.keys0.push_back(key)
            if key[0] > 0 and key[0] < 6:
                self.keys1.push_back(key)

        null_0 = gtsam_planner.NullConstraint(self.keys0)
        null_1 = gtsam_planner.NullConstraint(self.keys1)

        self.frame = gtsam_planner.FrameConstraint(self.single_key, all_keys, [null_0, null_1])
    
    def test_operatorTrue0(self):
        """Checks if factor returns 1.0 when variables have tentative values"""
        values = gtsam.DiscreteValues()
        values[self.single_key[0]] = 0
        values[self.keys0.at(0)[0]] = 0
        values[self.keys0.at(1)[0]] = 1
        values[self.keys0.at(2)[0]] = 0
        values[self.keys0.at(3)[0]] = 1
        
        self.assertEqual(self.frame(values), 1.0)
        self.assertEqual(self.frame.toDecisionTreeFactor()(values), 1.0)
    
    def test_operatorFalse0(self):
        """Checks if factor returns 0.0 when variables does not have tentative values"""
        values = gtsam.DiscreteValues()
        values[self.single_key[0]] = 0
        values[self.keys0.at(0)[0]] = 0
        values[self.keys0.at(1)[0]] = 1
        values[self.keys0.at(2)[0]] = 1
        values[self.keys0.at(3)[0]] = 1
        self.assertEqual(self.frame(values), 0.0)
        self.assertEqual(self.frame.toDecisionTreeFactor()(values), 0.0)
    
    def test_operatorTrue1(self):
        """Checks if factor returns 1.0 when variables have tentative values"""
        values = gtsam.DiscreteValues()
        values[self.single_key[0]] = 1
        values[self.keys1.at(0)[0]] = 1
        values[self.keys1.at(1)[0]] = 0
        values[self.keys1.at(2)[0]] = 1
        values[self.keys1.at(3)[0]] = 0
        self.assertEqual(self.frame(values), 1.0)
        self.assertEqual(self.frame.toDecisionTreeFactor()(values), 1.0)
    
    def test_operatorFalse1(self):
        """Checks if factor returns 0.0 when variables does not have tentative values"""
        values = gtsam.DiscreteValues()
        values[self.single_key[0]] = 1
        values[self.keys1.at(0)[0]] = 1
        values[self.keys1.at(1)[0]] = 0
        values[self.keys1.at(2)[0]] = 1
        values[self.keys1.at(3)[0]] = 1
        self.assertEqual(self.frame(values), 0.0)
        self.assertEqual(self.frame.toDecisionTreeFactor()(values), 0.0)


if __name__ == "__main__":
    unittest.main()
