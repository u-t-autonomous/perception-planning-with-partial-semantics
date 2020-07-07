# for planner
import sys
import time
import numpy as np
import copy
import itertools
from scipy.special import comb
from scipy.stats import entropy
from scipy.spatial import distance
import random
import gurobipy as grb
import pandas as pd
import matplotlib.pyplot as plt
from partial_semantics import *

# define simulation parameters
n_iter = 1
infqual_hist_all = []
risk_hist_all = []
timestep_all = []
plan_count_all = []
task_flag_all = []

for iter in range(n_iter):

    # create problem setting
    model = MDP('gridworld')
    model.semantic_representation(prior_belief='noisy-ind') # changed for scenario 2
    perc_flag = True
    bayes_flag = True
    replan_flag = True
    div_test_flag = True
    act_info_flag = True
    spec_true = [[],[]]
    for s in range(len(model.states)):
        if model.label_true[s,0] == True:
            spec_true[0].append(s)
        if model.label_true[s,1] == True:
            spec_true[1].append(s)

    visit_freq = np.ones(len(model.states))

##############################################################################

    # simulation results
    term_flag = False
    task_flag = False
    timestep = 0
    max_timestep = 250
    plan_count = 0
    div_thresh = 0.001
    n_sample = 10
    risk_thresh = 0.1
    state_hist = []
    state_hist.append(model.init_state)
    action_hist = [[],[]] # [[chosen action],[taken action]]
    infqual_hist = []
    infqual_hist.append(info_qual(model.label_belief))
    risk_hist = []

    while not term_flag:

        if perc_flag:
            # estimate map
            label_est = estimate_AP(model.label_belief, method='risk-averse')
            spec_est = [[],[]]
            for s in range(len(model.states)):
                if label_est[s,0] == True:
                    spec_est[0].append(s)
                if label_est[s,1] == True:
                    spec_est[1].append(s)
            # print("obstacle:   ",spec_est[0])
            # print("target:     ",spec_est[1])

        if replan_flag or (not replan_flag and plan_count==0):
            # find an optimal policy
            (vars_val, opt_policy) = verifier(copy.deepcopy(model), spec_est)
            # print(opt_policy[0:20])
            plan_count += 1

        if act_info_flag:
            # risk evaluation
            prob_sat = stat_verifier(model,state_hist[-1],opt_policy,spec_est,n_sample)
            risk = np.abs(vars_val[state_hist[-1]] - prob_sat); print(vars_val[state_hist[-1]],prob_sat)
            risk_hist.append(risk)
            print("Risk due to Perception Uncertainty:   ",risk)

            # perception policy
            if risk > risk_thresh:
                # implement perception policy
                timestep += 1
                state = state_hist[-1]
                action = 0
            else:
                pass
        timestep += 1
        print("Timestep:   ",timestep)
        state = state_hist[-1]
        opt_act = opt_policy[state]
        if 0 in opt_act and len(opt_act)>1:
            opt_act = opt_act[1:]

        action = np.random.choice(opt_act)

        action_hist[0].append(action)
        next_state = np.random.choice(model.states, p=model.transitions[state,action])
        # identify taken action
        for a in model.actions[model.enabled_actions[state]]:
            if model.action_effect(state,a)[0] == next_state:
                action_taken = a
        action_hist[1].append(action_taken)
        state_hist.append(next_state)

############################################################################## uncommented for scenario 2
        # get new information
        # (obs,p_obs_model) = obs_modeling(model)
        #
        # # update belief
        # next_label_belief = belief_update(model.label_belief, obs,
        #                                  p_obs_model, bayes_flag)

############################################################################### commented for scenario 2
        # array from Lidar
        # lid = np.zeros((dim1,dim2), dtype=np.float64) # {0=empty, -1=unseen, 1=obstacle}
        lid_adapted = np.reshape(lid, len(model.states))

        # update belief
        next_label_belief = copy.deepcopy(model.label_belief)
        visit_freq_next = copy.deepcopy(visit_freq) + 1
        for s in model.states:
            # update frequency
            if lid_adapted[s] == -1:
                visit_freq_next[s] -= 1
            elif lid_adapted[s] == 0:
                pass
            # update for 'obstacle'
            elif lid_adapted[s] == 1:
                next_label_belief[s,0] = (next_label_belief[s,0]*visit_freq[s] + 1) / visit_freq_next[s]
            # # update for 'target'
            # elif lid_adapted[s] == 2:
            #     next_label_belief[s,1] = (next_label_belief[s,1]*visit_freq[s] + 1) / visit_freq_next[s]
            else:
                raise NameError("Given value in the Lidar output is unaccepted")
        visit_freq = copy.deepcopy(visit_freq_next)

##############################################################################
        # move to next state
        # use action_hist[1][-1] # {0 : 'stop', 1 : 'up', 2 : 'right', 3 : 'down', 4 : 'left'}

        # divergence test on belief
        if div_test_flag:
            div = info_div(model.label_belief,next_label_belief)
            print("Belief Divergence:   ",div)
            if info_div(model.label_belief,next_label_belief) > div_thresh:
                replan_flag = True
            else:
                replan_flag = False
        model.label_belief = np.copy(next_label_belief)
        infqual_hist.append(info_qual(model.label_belief))

        # check task realization
        if model.label_true[state_hist[-1],0] == True:
            term_flag = True
            print("at a state with an obstacle")

        if model.label_true[state_hist[-1],1] == True:
            task_flag = True
            term_flag = True
            print("at a target state")

        if timestep == max_timestep:
            term_flag = True
            print("timestep exceeded the maximum limit")

    print("Number of Time Steps:   ",timestep)
    print("Number of Replanning:   ",plan_count)

    infqual_hist_all.append(infqual_hist)
    risk_hist_all.append(risk_hist)
    timestep_all.append(timestep)
    plan_count_all.append(plan_count)
    task_flag_all.append(int(task_flag))

task_rate = np.mean(task_flag_all)
