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
from itertools import product
from collections import defaultdict
from abc import ABC, abstractmethod

# import gtsam
import gtsam
from gtsam import *


import SASParser
from SASParser import SAS, Operator
import SasToGtsam
from SasToGtsam import SASToGTSAM
from gtsam.utils.test_case import GtsamTestCase


class TestSASToGTSAM(GtsamTestCase):
    """Tests for Single Value Constraints"""

    def setUp(self):
        sas = SAS()
        sas_dir = os.path.abspath(os.path.join('python', 'sas', 'block_example.sas'))
        sas.read_file(sas_dir)
        self.converter = SASToGTSAM(sas)
        self.init_state = self.converter.generate_state(0)
        self.next_state = self.converter.generate_state(1)
        self.op_key = self.converter.generate_operator_key(0)

        input = []
        for _, vars in self.converter.vars.items():
            input.append(list(range(len(vars))))
        # tried this but this crashes the kernel
        # for _, vars in self.converter.vars.items():
        #     input.append(list(range(len(vars))))
        self.prods = list(product(*input))
    
    def createVal(self, states, prod):
        values = gtsam.DiscreteValues()
        for state, val in zip(states, prod):
            values[state[0]] = val
        return values

    def createOperatorVal(self, state1, state2, op):
        state = self.converter.state_keys
        values = gtsam.DiscreteValues()
        for var, val in op.precondition.items():
            if val == -1:
                continue
            values[state1[state.index(var)][0]] = val
        for var, val in op.effect.items():
            if val == -1:
                continue
            values[state2[state.index(var)][0]] = val
        return values
    
    def valid(self, prod, mutex_group):
        count = 0
        for var, val in mutex_group:
            if prod[var] == val:
                count += 1
            if count > 1:
                return 0.0
        return 1.0

    def test_generateState(self):
        """
        Initial state should be made out of variables
        """
        assert len(self.init_state) == len(self.converter.vars)

    def test_generateOperatorState(self):
        """
        op_key is a has len(converter.ops) number of cardinality
        """
        # there are 32 possible operators
        assert self.op_key[1] == len(self.converter.ops)

    def test_generateInitial(self):
        """
        Tests if initial_factor returns true for only the initial state and
        no other states (combination of values)
        """
        state = self.converter.state_keys
        initial_factor = self.converter.generate_initial_factor(self.init_state)
        total = 0.0
        for prod in self.prods:
            values = self.createVal(self.init_state, prod)
            output = initial_factor(values)
            if output == 1:
                for var, val in self.converter.init.items():
                    assert values[self.init_state[state.index(var)][0]] == val
            total += output
        assert total == 1

    def test_generateGoal(self):
        """
        Tests if goal_factor returns true for only the goal state and
        no other states (combination of values)
        """
        state = self.converter.state_keys
        goal_factor = self.converter.generate_goal_factor(self.next_state)
        total = 0.0
        for prod in self.prods:
            values = self.createVal(self.next_state, prod)
            output = goal_factor(values)
            if output == 1:
                for var, val in self.converter.goal.items():
                    assert values[self.next_state[state.index(var)][0]] == val
            total += output
        assert total > 1

    def test_generateOperatorConstraint(self):
        for operator in self.converter.ops:
            op_f, null_f, op_keys, frame_keys = self.converter.generate_op_null(self.init_state, self.next_state, operator)
            values = self.createOperatorVal(self.init_state, self.next_state, operator)
            assert op_f(values) == 1.0
    
    # def test_generateMutex(self):
    #     mutex_factors = self.converter.generate_mutex_factor(self.init_state)
    #     for prod in tqdm_notebook(self.prods, desc='possible states'):
    #         for mutex_factor, mutex_group in zip(mutex_factors, converter.mutex_groups):
    #             check_valid = self.valid(prod, mutex_group)
    #             values = self.createVal(self.init_state, prod)
    #             factor_valid = mutex_factor(values)
    #             tree_factor = mutex_factor.toDecisionTreeFactor()
    #             tree_valid = tree_factor(values)
    #             assert tree_valid == factor_valid == check_valid    

if __name__ == "__main__":
    unittest.main()
