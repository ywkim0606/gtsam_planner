import sys
import re
import copy
from collections import defaultdict
from abc import ABC, abstractmethod

class Operator():
    def __init__(self, name, num_prevail, prevail, num_effect, precondition, effect, cost):
        self.name = name
        self.num_prevail = num_prevail
        self.prevail = prevail
        self.num_effect = num_effect
        self.precondition = precondition
        self.effect = effect
        self.cost = cost

class SAS(ABC):
    def __init__(self):
        
        self.version = 0
        self.metric = 0
        
        self.initial_state = {}
        self.goal = {}
        self.mutex_group = []
        self.var_cardinalities = {}
        self.variables = {}
        self.operators = []
        
    def read_file(self, output_dir):
        with open(output_dir, 'r') as f:
            sas = f.readlines()
            for line in sas:
                begin = re.match("begin_(.*)", line)
                end = re.match("end_(.*)", line)
                if begin:
                    contents = []
                elif end:
                    self.parse(end.group(1), contents)
                else:
                    contents.append(line.rstrip('\n'))
    
    def parse(self, group, contents):
        if hasattr(self, "parse_" + group):
            function = getattr(self, "parse_" + group)
            function(contents)
            
    def parse_version(self, contents):
        assert(len(contents) == 1)
        self.version = int(contents[0])
    
    def parse_metric(self, contents):
        assert(len(contents) == 1)
        self.metric = int(contents[0])
        
    def parse_state(self, contents):
        for i, content in enumerate(contents):
            self.initial_state[i] = int(content)
    
    def parse_goal(self, contents):
        for content in contents[1:]:
            var_vals = [int(var_val) for var_val in content.split(' ')]
            self.goal[var_vals[0]] = var_vals[1]
    
    def parse_mutex_group(self, contents):
        group = []
        for content in contents[1:]:
            var_vals = [int(var_val) for var_val in content.split(' ')]
            group.append(var_vals)
        self.mutex_group.append(group)
        
    def parse_variable(self, contents):
        m = re.match("var([0-9]+)", contents[0])
        var_idx = int(m.group(1))
        axiom = int(contents[1])
        self.var_cardinalities[var_idx] = int(contents[2])
        values = []
        for value in contents[3:]:
            if axiom > -1:
                pass
#                 raise NotImplementedError("axiom not implemented")
            else:
                values.append(value)
        self.variables[var_idx] = values
    
    def parse_operator(self, contents):
        prevail = {}
        precondition = {}
        effect = {}
        # conditional_effects = []
        # ex) move rooma roomb
        name = contents[0]
        # ex) 0 --> number of prevail conditions
        num_prevail = int(contents[1])
        # prevail condition: var = val
        for content in contents[2:2+num_prevail]:
            (var, val) = [int(num) for num in content.split(' ')]
            prevail[var] = val
        # ex) 2 --> number of effect conditions
        num_effect = int(contents[num_prevail+2])
        with_conditional = False
        for content in contents[num_prevail+3:num_prevail+num_effect+3]:
            # number of effect condition
            num_conditions = int(content[0])
            assoc_conditions = {}
            rest = [int(num) for num in content.split(' ')][1:]
            if num_conditions > 0:
                with_conditional = True
            for i in range(num_conditions):
                var, val = rest[:2]
                rest = rest[2:]
                assoc_conditions[var] = val
            var, pre_val, eff_val = rest
            precondition[var] = pre_val
            effect[var] = eff_val
        cost = int(contents[-1])
        if with_conditional:
            raise NotImplementedError("Operator with conditional effects not implemented")
        else:
            operator = Operator(name, num_prevail, prevail, num_effect, precondition, effect, cost)
            self.operators.append(operator)