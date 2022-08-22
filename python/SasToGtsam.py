# typing
from typing import List

# import gtsam
import gtsam
from gtsam import *

# import gtbook
import gtbook
from gtbook.discrete import Variables

# import local package
import gtsam_planner
from gtsam_planner import *

class SASToGTSAM():
    def __init__(self, sas):
        self.sas = sas
        self.init = self.sas.initial_state
        self.goal = self.sas.goal
        self.vars = self.sas.variables
        self.ops = self.sas.operators
        self.mutex_groups = self.sas.mutex_group
        self.ops_names = []
        for op in self.ops:
            self.ops_names.append(op.name)
        self.state_keys = list(self.vars.keys())
        self.variables = Variables()

    def generate_state(self, timestep: int) -> List:
        state = []
        for var, val in self.vars.items():
            state_var = self.variables.discrete(str(var)+"_"+str(timestep), val)
            state.append(state_var)
        return state
    
    def generate_operator_key(self, timestep: int):
        op_var = self.variables.discrete("op_"+str(timestep), self.ops_names)
        return op_var
    
    def generate_initial_factor(self, initial_state):
        keys = gtsam.DiscreteKeys()
        for key in initial_state:
            keys.push_back(key)
        init_values = list(self.init.values())
        init_f = gtsam_planner.MultiValueConstraint(keys, init_values)
        return init_f

    def generate_goal_factor(self, goal_state):
        keys = gtsam.DiscreteKeys()
        vals = []
        for goal_var, goal_val in self.goal.items():
            keys.push_back(goal_state[self.state_keys.index(goal_var)])
            vals.append(goal_val)
        goal_f = gtsam_planner.MultiValueConstraint(keys, vals)
        return goal_f
    
    def generate_op_null(self, state_t, state_tp, operator):
        vals = []
        op_keys = set()
        multi_keys = gtsam.DiscreteKeys()
        for pre_var, pre_val in operator.precondition.items():
            key = state_t[self.state_keys.index(pre_var)]
            op_keys.add(key)
            if pre_val == -1:
                continue
            multi_keys.push_back(key)
            vals.append(pre_val)
        
        for eff_var, eff_val in operator.effect.items():
            key = state_tp[self.state_keys.index(eff_var)]
            op_keys.add(key)
            if eff_val == -1:
                continue
            multi_keys.push_back(key)
            vals.append(eff_val)
        
        if operator.num_prevail > 0:
            for var, val in operator.prevail.items():
                key_t = state_t[self.state_keys.index(var)]
                key_tp = state_tp[self.state_keys.index(var)]
                op_keys.add(key_t)
                op_keys.add(key_tp)
                multi_keys.push_back(key_t)
                multi_keys.push_back(key_tp)
                vals.extend([val,val])

        assert len(op_keys) % 2 == 0
        frame_keys = set()
        null_keys = gtsam.DiscreteKeys()
        for var in state_t+state_tp:
            if var not in op_keys:
                null_keys.push_back(var)
                frame_keys.add(var)

        op_f = gtsam_planner.MultiValueConstraint(multi_keys, vals)
        null_f = gtsam_planner.NullConstraint(null_keys)
        return op_f, null_f, op_keys, frame_keys

    def generate_mutex_factor(self, state_t):
        state = list(self.vars.keys())
        mutex_variables = []
        mutex_values = []
        
        for mutex_group in self.mutex_groups:
            var_group = []
            val_group = []
            for var, val in mutex_group:
                state_var = state_t[state.index(var)]
                var_group.append(state_var)
                val_group.append(val)
            mutex_variables.append(var_group)
            mutex_values.append(val_group)
        
        factors = []
        for mutex_var, mutex_val in zip(mutex_variables, mutex_values):
            keys = gtsam.DiscreteKeys()
            for var in mutex_var:
                keys.push_back(var)
            mutex = gtsam_planner.MutexConstraint(keys, mutex_val)
            factors.append(mutex)
        return factors
    
    def generate_frame_op_factor(self, state_t, state_tp, op_key):
        op_consts = []
        null_consts = []
        
        op_keys_set = set()
        op_keys_set.add(op_key)
        
        frame_keys_set = set()
        frame_keys_set.add(op_key)
        
        for op in self.ops:
            op_const, null_const, op_keys, frame_keys = self.generate_op_null(state_t, state_tp, op)
            op_consts.append(op_const)
            null_consts.append(null_const)
            op_keys_set = op_keys_set.union(op_keys)
            frame_keys_set = frame_keys_set.union(frame_keys)

        op_dkeys = gtsam.DiscreteKeys()
        for key in op_keys_set:
            op_dkeys.push_back(key)
        
        frame_dkeys = gtsam.DiscreteKeys()
        for key in frame_keys_set:
            frame_dkeys.push_back(key)

        op_factor = gtsam_planner.OperatorOrConstraint(op_key, op_dkeys, op_consts)
        frame_factor = gtsam_planner.FrameConstraint(op_key, frame_dkeys, null_consts)
        return op_factor, frame_factor