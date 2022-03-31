"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

SASParser unit tests.
Author: Yoonwoo Kim
"""
import unittest

import sys, os
sys.path.append(os.path.abspath(os.path.join('python')))

import re
import copy
from collections import defaultdict
from abc import ABC, abstractmethod
import SASParser
from SASParser import SAS, Operator

class TestSingleValueConstraint(unittest.TestCase):
    """Tests for Single Value Constraints"""

    def setUp(self):
        output_dir = os.path.abspath(os.path.join('python', 'sas', 'block_example.sas'))
        self.sas = SAS()
        self.sas.read_file(output_dir)
    
    def test_initialState(self):
        expected = {0: 4, 1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 4, 7: 4, 8: 4}
        initial_state = self.sas.initial_state
        self.assertEqual(expected, initial_state)
    
    def test_goalState(self):
        expected = {6: 1, 7: 2, 8: 3}
        goal_state = self.sas.goal
        self.assertEqual(expected, goal_state)
    
    def test_varCardinalities(self):
        expected = {0: 5, 1: 2, 2: 2, 3: 2, 4: 2, 5: 2, 6: 5, 7: 5, 8: 5}
        cardinalities = self.sas.var_cardinalities
        self.assertEqual(expected, cardinalities)
    
    def test_variables(self):
        expected = {0: ['Atom holding(a)',
                        'Atom on(a, b)',
                        'Atom on(a, c)',
                        'Atom on(a, d)',
                        'Atom ontable(a)'],
                    1: ['Atom clear(a)', 'NegatedAtom clear(a)'],
                    2: ['Atom clear(b)', 'NegatedAtom clear(b)'],
                    3: ['Atom clear(c)', 'NegatedAtom clear(c)'],
                    4: ['Atom clear(d)', 'NegatedAtom clear(d)'],
                    5: ['Atom handempty()', 'NegatedAtom handempty()'],
                    6: ['Atom holding(b)',
                        'Atom on(b, a)',
                        'Atom on(b, c)',
                        'Atom on(b, d)',
                        'Atom ontable(b)'],
                    7: ['Atom holding(c)',
                        'Atom on(c, a)',
                        'Atom on(c, b)',
                        'Atom on(c, d)',
                        'Atom ontable(c)'],
                    8: ['Atom holding(d)',
                        'Atom on(d, a)',
                        'Atom on(d, b)',
                        'Atom on(d, c)',
                        'Atom ontable(d)']}
        variables = self.sas.variables
        self.assertEqual(expected, variables)
    
    def test_mutexGroup(self):
        expected = [[[1, 0], [0, 0], [6, 1], [7, 1], [8, 1]],
                    [[2, 0], [0, 1], [6, 0], [7, 2], [8, 2]],
                    [[3, 0], [0, 2], [6, 2], [7, 0], [8, 3]],
                    [[4, 0], [0, 3], [6, 3], [7, 3], [8, 0]],
                    [[5, 0], [0, 0], [6, 0], [7, 0], [8, 0]]]
        mutex_group = self.sas.mutex_group
        self.assertEqual(expected, mutex_group)
    
    def test_operator(self):
        operators = self.sas.operators
        self.assertEqual(32, len(operators))
        
        op = operators[0]
        expected_name = 'pick-up a'
        name = op.name
        self.assertEqual(expected_name, name)
        
        expected_precondition = {1: 0, 5: 0, 0: 4}
        precondition = op.precondition
        self.assertEqual(expected_precondition, precondition)

        expected_effect = {1: 1, 5: 1, 0: 0}
        effect = op.effect
        self.assertEqual(expected_effect, effect)

if __name__ == "__main__":
    unittest.main()
