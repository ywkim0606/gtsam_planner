"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

OperatorConstraint unit tests.
Author: Yoonwoo Kim
"""
import unittest

import numpy as np

import gtsam
from gtsam import DecisionTreeFactor, DiscreteValues, DiscreteKeys
import gtsam_planner
from gtsam_planner import OperatorConstraint
from gtsam.utils.test_case import GtsamTestCase


class TestOperatorConstraint(GtsamTestCase):
    """Tests for OperatorChooseConstraint which is basically choosing from list of MultiValueConstraints"""

    def setUp(self):
        """
        state_t = [key0(0,2), key1(1,2)]
        state_tp = [key2(2,2), key3(3,2)]
        The operator will check if key1, and key3 indicating the same variables changed from 0 to 1
        while key0 and key2 stayed the same (either both 0 or both 1)
        """
        self.multi_keys = gtsam.DiscreteKeys()
        self.null_keys = gtsam.DiscreteKeys()
        self.dkeys = gtsam.DiscreteKeys()
        state_t = [(0, 2), (1, 2)]
        state_tp = [(2, 2), (3, 2)]
        self.multi_keys.push_back(state_t[1])
        self.multi_keys.push_back(state_tp[1])
        self.multi_vals = [0, 1]
        self.null_keys.push_back(state_t[0])
        self.null_keys.push_back(state_tp[0])
        for key in state_t+state_tp:
            self.dkeys.push_back(key)
        self.op = gtsam_planner.OperatorConstraint(self.multi_keys, self.multi_vals, self.null_keys, self.dkeys)
    
    def test_operatorTrue0(self):
        """Checks if factor returns 1.0 if only value at state_t[1] changed to state_tp[1]"""
        values = gtsam.DiscreteValues()
        values[self.multi_keys.at(0)[0]] = 0
        values[self.multi_keys.at(1)[0]] = 1
        values[self.null_keys.at(0)[0]] = 0
        values[self.null_keys.at(1)[0]] = 0
        self.assertEqual(self.op(values), 1.0)
        self.assertEqual(self.op.toDecisionTreeFactor()(values), 1.0)

    def test_operatorTrue1(self):
        """Checks if factor returns 1.0 if only value at state_t[1] changed to state_tp[1]"""
        values = gtsam.DiscreteValues()
        values[self.multi_keys.at(0)[0]] = 0
        values[self.multi_keys.at(1)[0]] = 1
        values[self.null_keys.at(0)[0]] = 1
        values[self.null_keys.at(1)[0]] = 1
        self.assertEqual(self.op(values), 1.0)
        self.assertEqual(self.op.toDecisionTreeFactor()(values), 1.0)
    
    def test_operatorFalse0(self):
        """
        Checks if factor returns 1.0 if value at state_t[1] changed to state_tp[1]
        and value at state_t[0] also changed to state_tp[0]
        """
        values = gtsam.DiscreteValues()
        values[self.multi_keys.at(0)[0]] = 0
        values[self.multi_keys.at(1)[0]] = 1
        values[self.null_keys.at(0)[0]] = 0
        values[self.null_keys.at(1)[0]] = 1
        self.assertEqual(self.op(values), 0.0)
        self.assertEqual(self.op.toDecisionTreeFactor()(values), 0.0)

    def test_operatorFalse1(self):
        """
        Checks if factor returns 1.0 if value at state_t[1] changed to state_tp[1]
        and value at state_t[0] also changed to state_tp[0]
        """
        values = gtsam.DiscreteValues()
        values[self.multi_keys.at(0)[0]] = 0
        values[self.multi_keys.at(1)[0]] = 1
        values[self.null_keys.at(0)[0]] = 1
        values[self.null_keys.at(1)[0]] = 0
        self.assertEqual(self.op(values), 0.0)
        self.assertEqual(self.op.toDecisionTreeFactor()(values), 0.0)


if __name__ == "__main__":
    unittest.main()
