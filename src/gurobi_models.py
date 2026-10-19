"""
GUROBI Single-Flow and Multi-Flow ILP Model Objects Factory
"""

import re
import json
from typing import Any, Union, Dict
from dataclasses import dataclass

import gurobipy as gp
from gurobipy import GRB

import ijson

from kcmc_instance import KCMC_Instance
from gurobi_model_wrapper import GurobiModelWrapper



"""
# ######################################################################################################################
# MULTI-FLOW
"""


def gurobi_multi_flow(
    kcmc_instance:KCMC_Instance,
    time_limit=60, threads=1, LOGFILE=None,
    y_binary=True
) -> tuple[GurobiModelWrapper, Any, Any]:

    # Prepare the model object, using the wrapper
    model_mf = GurobiModelWrapper('KCMC MULTI-FLOW', kcmc_instance.kcmc_k, kcmc_instance.kcmc_m, kcmc_instance, time_limit, threads, LOGFILE)

    # Extract the component sets
    P = sorted(kcmc_instance.pois.keys())
    I = sorted(kcmc_instance.sensors.keys())
    s = sorted(kcmc_instance.sinks.keys())
    assert len(s) == 1, 'THIS VERSION ONLY SUPPORTS SINGLE-SINK!'

    A_c = kcmc_instance.poi_edges
    A_s = kcmc_instance.sink_edges
    A_g = kcmc_instance.sensor_edges
    A = A_c + A_g + A_s

    iC = kcmc_instance.inverse_coverage_graph
    L = [str(m) for m in range(kcmc_instance.kcmc_m)]

    # Set the variables
    X = model_mf.add_vars(I, L, name='x', vtype=GRB.BINARY)
    if y_binary:
        Y = model_mf.add_vars(A, P, L, name='y', vtype=GRB.BINARY)
    else:
        Y = model_mf.add_vars(A, P, L, name='y')

    # Set the objective function
    model_mf.set_objective(X.sum('*', '*'), GRB.MINIMIZE)

    # Set the CONSTRAINTS -------------------------------------------------------------------------

    # Disjunction -----------------------------------------
    disjunction = model_mf.add_constraints(
        (X.sum(i, '*') <= 1
         for i in I),
        name="disjunction"
    )

    # Flow ------------------------------------------------
    flow_p = model_mf.add_constraints(
        ((  gp.quicksum(Y.select(p, '*', p, l))
          - gp.quicksum(Y.select('*', p, p, l))) == 1
         for p in P
         for l in L),
        name="flow_p"
    )

    flow_i = model_mf.add_constraints(
        ((  gp.quicksum(Y.select(i, '*', p, l))
          - gp.quicksum(Y.select('*', i, p, l))) == 0
         for i in I
         for p in P
         for l in L),
        name="flow_i"
    )

    flow_s = model_mf.add_constraints(
        ((  gp.quicksum(Y.select(s, '*', p, l))
          - gp.quicksum(Y.select('*', s, p, l))) == -1
         for p in P
         for l in L),
        name="flow_s"
    )

    # Projection ------------------------------------------
    projection = model_mf.add_constraints(
        (Y.sum(i, '*', p, l) <= X.sum(i, l)
         for i in I
         for p in P
         for l in L),
        name="projection"
    )

    # K-Coverage ------------------------------------------
    k_coverage = model_mf.add_constraints(
        (gp.quicksum(X.select(sorted(iC[p]), '*')) >= kcmc_instance.kcmc_k
         for p in P),
        name="k_coverage"
    )

    # Return the model
    return model_mf, X, Y


"""
# ######################################################################################################################
# SINGLE-FLOW
"""


def gurobi_single_flow(
    kcmc_instance:KCMC_Instance, time_limit=60, threads=1, LOGFILE=None,
    y_binary=True
) -> tuple[GurobiModelWrapper, Any, Any]:

    # Prepare the model object, using the wrapper
    model_sf = GurobiModelWrapper('KCMC SINGLE-FLOW', kcmc_instance.kcmc_k, kcmc_instance.kcmc_m, kcmc_instance, time_limit, threads, LOGFILE)

    # Extract the component sets
    P = sorted(kcmc_instance.pois.keys())
    I = sorted(kcmc_instance.sensors.keys())
    s = sorted(kcmc_instance.sinks.keys())
    assert len(s) == 1, 'THIS VERSION ONLY SUPPORTS SINGLE-SINK!'

    A_c = kcmc_instance.poi_edges
    A_s = kcmc_instance.sink_edges
    A_g = kcmc_instance.sensor_edges
    A = A_c + A_g + A_s

    iC = kcmc_instance.inverse_coverage_graph
    # L = [str(m) for m in range(kcmc_m)] not used in this formulation

    # Set the variables
    X = model_sf.add_vars(I, name='x', vtype=GRB.BINARY)
    if y_binary:
        Y = model_sf.add_vars(A, P, name='y', vtype=GRB.BINARY)
    else:
        Y = model_sf.add_vars(A, P, name='y')

    # Set the objective function
    model_sf.set_objective(X.sum('*'), GRB.MINIMIZE)

    # Set the CONSTRAINTS -------------------------------------------------------------------------

    # Flow ------------------------------------------------
    flow_p = model_sf.add_constraints(
        ((  gp.quicksum(Y.select(p, '*', p))
          - gp.quicksum(Y.select('*', p, p))) == kcmc_instance.kcmc_m
         for p in P),
        name="flow_p"
    )

    flow_s = model_sf.add_constraints(
        ((  gp.quicksum(Y.select(s, '*', p))
          - gp.quicksum(Y.select('*', s, p))) == -1 * kcmc_instance.kcmc_m
         for p in P),
        name="flow_s"
    )

    flow_i = model_sf.add_constraints(
        ((  gp.quicksum(Y.select(i, '*', p))
          - gp.quicksum(Y.select('*', i, p))) == 0
         for i in I
         for p in P),
        name="flow_i"
    )

    # Projection ------------------------------------------
    projection = model_sf.add_constraints(
        (Y.sum(i, '*', p) <= X.sum(i)
         for i in I
         for p in P),
        name="projection"
    )

    # K-Coverage ------------------------------------------
    k_coverage = model_sf.add_constraints(
        (gp.quicksum(X.select(sorted(iC[p]), '*')) >= kcmc_instance.kcmc_k
         for p in P),
        name="k_coverage"
    )

    # Return the model
    return model_sf, X, Y
