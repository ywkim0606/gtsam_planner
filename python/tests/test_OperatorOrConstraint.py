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
from gtsam_planner import OperatorOrConstraint, MultiValueConstraint
from gtsam.utils.test_case import GtsamTestCase


class TestMultiValueConstraint(GtsamTestCase):
    """Tests for OperatorConstraint which is basically choosing from list of MultiValueConstraints"""

    def setUp(self):
        self.keys0 = gtsam.DiscreteKeys()
        self.keys1 = gtsam.DiscreteKeys()
        self.keys2 = gtsam.DiscreteKeys()
        all_keys = gtsam.DiscreteKeys()
        self.single_key = (5, 3)
        key_list = [(0, 2), (1, 2), (2, 3), (3, 2), (4, 2)]

        for key in key_list:
            all_keys.push_back(key)

        for key in key_list:
            if key[0] < 3:
                self.keys0.push_back(key)
            if key[0] > 0 and key[0] < 4:
                self.keys1.push_back(key)
            if key[0] > 1 and key[0] < 5:
                self.keys2.push_back(key)
        vals0 = [0, 1, 2]
        vals1 = [0, 2, 0]
        vals2 = [2, 1, 1]
        multi_0 = gtsam_planner.MultiValueConstraint(self.keys0, vals0)
        multi_1 = gtsam_planner.MultiValueConstraint(self.keys1, vals1)
        multi_2 = gtsam_planner.MultiValueConstraint(self.keys2, vals2)

        self.op_add = gtsam_planner.OperatorOrConstraint(self.single_key, all_keys, [multi_0, multi_1, multi_2])
    
    def test_operatorTrue0(self):
        """Checks if factor returns 1.0 when variables have tentative values"""
        values = gtsam.DiscreteValues()
        values[self.single_key[0]] = 0
        values[self.keys0.at(0)[0]] = 0
        values[self.keys0.at(1)[0]] = 1
        values[self.keys0.at(2)[0]] = 2
        self.assertEqual(self.op_add(values), 1.0)
        self.assertEqual(self.op_add.toDecisionTreeFactor()(values), 1.0)
    
    def test_operatorFalse0(self):
        """Checks if factor returns 0.0 when variables does not have tentative values"""
        values = gtsam.DiscreteValues()
        values[self.single_key[0]] = 0
        values[self.keys0.at(0)[0]] = 0
        values[self.keys0.at(1)[0]] = 0
        values[self.keys0.at(2)[0]] = 2
        self.assertEqual(self.op_add(values), 0.0)
        self.assertEqual(self.op_add.toDecisionTreeFactor()(values), 0.0)
    
    def test_operatorTrue1(self):
        """Checks if factor returns 1.0 when variables have tentative values"""
        values = gtsam.DiscreteValues()
        values[self.single_key[0]] = 1
        values[self.keys1.at(0)[0]] = 0
        values[self.keys1.at(1)[0]] = 2
        values[self.keys1.at(2)[0]] = 0
        self.assertEqual(self.op_add(values), 1.0)
        self.assertEqual(self.op_add.toDecisionTreeFactor()(values), 1.0)
    
    def test_operatorFalse1(self):
        """Checks if factor returns 0.0 when variables does not have tentative values"""
        values = gtsam.DiscreteValues()
        values[self.single_key[0]] = 1
        values[self.keys1.at(0)[0]] = 0
        values[self.keys1.at(1)[0]] = 1
        values[self.keys1.at(2)[0]] = 0
        self.assertEqual(self.op_add(values), 0.0)
        self.assertEqual(self.op_add.toDecisionTreeFactor()(values), 0.0)
    
    def test_operatorTrue2(self):
        """Checks if factor returns 1.0 when variables have tentative values"""
        values = gtsam.DiscreteValues()
        values[self.single_key[0]] = 2
        values[self.keys2.at(0)[0]] = 2
        values[self.keys2.at(1)[0]] = 1
        values[self.keys2.at(2)[0]] = 1
        self.assertEqual(self.op_add(values), 1.0)
        self.assertEqual(self.op_add.toDecisionTreeFactor()(values), 1.0)
    
    def test_operatorFalse2(self):
        """Checks if factor returns 0.0 when variables does not have tentative values"""
        values = gtsam.DiscreteValues()
        values[self.single_key[0]] = 2
        values[self.keys2.at(1)[0]] = 1
        values[self.keys2.at(2)[0]] = 0
        self.assertEqual(self.op_add(values), 0.0)
        self.assertEqual(self.op_add.toDecisionTreeFactor()(values), 0.0)

    def test_operatorTrue0_Table(self):
        """Checks if factor returns 1.0 when variables have tentative values"""
        values = gtsam.DiscreteValues()
        values[self.single_key[0]] = 0
        values[self.keys0.at(0)[0]] = 0
        values[self.keys0.at(1)[0]] = 1
        values[self.keys0.at(2)[0]] = 2
        self.assertEqual(self.op_add(values), 1.0)
        self.assertEqual(self.op_add.toTableFactor()(values), 1.0)
    
    def test_operatorFalse0_Table(self):
        """Checks if factor returns 0.0 when variables does not have tentative values"""
        values = gtsam.DiscreteValues()
        values[self.single_key[0]] = 0
        values[self.keys0.at(0)[0]] = 0
        values[self.keys0.at(1)[0]] = 0
        values[self.keys0.at(2)[0]] = 2
        self.assertEqual(self.op_add(values), 0.0)
        self.assertEqual(self.op_add.toTableFactor()(values), 0.0)
    
    def test_operatorTrue1_Table(self):
        """Checks if factor returns 1.0 when variables have tentative values"""
        values = gtsam.DiscreteValues()
        values[self.single_key[0]] = 1
        values[self.keys1.at(0)[0]] = 0
        values[self.keys1.at(1)[0]] = 2
        values[self.keys1.at(2)[0]] = 0
        self.assertEqual(self.op_add(values), 1.0)
        self.assertEqual(self.op_add.toTableFactor()(values), 1.0)
    
    def test_operatorFalse1_Table(self):
        """Checks if factor returns 0.0 when variables does not have tentative values"""
        values = gtsam.DiscreteValues()
        values[self.single_key[0]] = 1
        values[self.keys1.at(0)[0]] = 0
        values[self.keys1.at(1)[0]] = 1
        values[self.keys1.at(2)[0]] = 0
        self.assertEqual(self.op_add(values), 0.0)
        self.assertEqual(self.op_add.toTableFactor()(values), 0.0)
    
    def test_operatorTrue2_Table(self):
        """Checks if factor returns 1.0 when variables have tentative values"""
        values = gtsam.DiscreteValues()
        values[self.single_key[0]] = 2
        values[self.keys2.at(0)[0]] = 2
        values[self.keys2.at(1)[0]] = 1
        values[self.keys2.at(2)[0]] = 1
        self.assertEqual(self.op_add(values), 1.0)
        self.assertEqual(self.op_add.toTableFactor()(values), 1.0)
    
    def test_operatorFalse2_Table(self):
        """Checks if factor returns 0.0 when variables does not have tentative values"""
        values = gtsam.DiscreteValues()
        values[self.single_key[0]] = 2
        values[self.keys2.at(1)[0]] = 1
        values[self.keys2.at(2)[0]] = 0
        self.assertEqual(self.op_add(values), 0.0)
        self.assertEqual(self.op_add.toTableFactor()(values), 0.0)


if __name__ == "__main__":
    unittest.main()
