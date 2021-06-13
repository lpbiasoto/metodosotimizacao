import gurobipy as gb
import numpy as np
import logging

class expando(object):
    pass

def minimo(a, b):
    if a < b:
        return a
    else:
        return b

class ProblemData():
    items = range(1,6)
    time = range(1,11)
    s = {1:4.5, 2:6.2, 3:4.2, 4:4.0, 5:9.5}
    h = {1:0.9, 2:0.4, 3:0.8, 4:0.5, 5:0.1}
    b = {1:0.8, 2:0.6, 3:0.7, 4:0.2, 5:0.5}
    sp = {1:9.2, 2:6.7, 3:6.7, 4:7.2, 5:4.3}
    Ct = 48
    d = {
        (1,1):1, (1,2):6, (1,3):10, (1,4):6, (1,5):5, (1,6):7, (1,7):5, (1,8):9, (1,9):4, (1,10):5,
        (2,1):2, (2,2):9, (2,3):0, (2,4):0, (2,5):0, (2,6):0, (2,7):8, (2,8):8, (2,9):2, (2,10):9,
        (3,1):3, (3,2):4, (3,3):7, (3,4):0, (3,5):2, (3,6):7, (3,7):2, (3,8):4, (3,9):4, (3,10):10,
        (4,1):4, (4,2):4, (4,3):9, (4,4):4, (4,5):8, (4,6):9, (4,7):3, (4,8):4, (4,9):8, (4,10):6,
        (5,1):5, (5,2):8, (5,3):3, (5,4):3, (5,5):10, (5,6):8, (5,7):8, (5,8):7, (5,9):9, (5,10):2
        }
    d_tau = {1:0, 2:0, 3:0, 4:0, 5:0}
    for i in items:
        for t in time:
            d_tau[i] = d_tau[i] + d[i,t]

class Relax_and_Fix:
    def __init__(self, t, prev_m, bloco):
        self.bloco = bloco
        self.iter = t
        self.prev_model = prev_m
        self.data = ProblemData()
        self.variables = expando()
        self.constraints = expando()
        self.results = expando()
        self._load_data()
        self._build_model()

    def optimize(self, simple_results=False):
        self.model.optimize()
        x = [self.variables.x[x] for x in self.variables.x]
        y = [self.variables.y[x] for x in self.variables.y]
        # print(x)
        print("Vetor y[i,t]: ", y)

        pass

    def _load_data(self):
        self.data.upper_bounds = []
        self.data.lower_bounds = []
        self.data.xs = []
        self.data.ys = []

    def _build_model(self):
        self.model = gb.Model()
        self._build_variables()
        self._build_objective()
        self._build_constraints()
        self.model.update()
        self.model.write("original.lp")

    def _build_variables(self):
        m = self.model

        self.model._I = range(1,6)
        self.model._T = range(1,11)
        
        self.variables.x = {}
        self.variables.Ix = {}
        self.variables.y = {}

        m._x = {}
        m._Ix = {}
        m._y = {}

        for t in self.model._T:
            if t <= (self.iter - bloco):
                for i in self.model._I:
                    if t > (self.iter - 2*self.bloco):
                        # m._x[i,t] = self.prev_model.variables.x[i,t].x
                        # m._Ix[i,t] = self.prev_model.variables.Ix[i,t].x
                        m._y[i,t] = self.prev_model.variables.y[i,t].x
                    else:
                        # m._x[i,t] = self.prev_model.variables.x[i,t]
                        # m._Ix[i,t] = self.prev_model.variables.Ix[i,t]
                        m._y[i,t] = self.prev_model.variables.y[i,t]
                    m._Ix[i,t] = m.addVar(
                    lb = 0.0, ub=gb.GRB.INFINITY,
                    vtype = gb.GRB.CONTINUOUS,
                    name = "Ix")
                    m._x[i,t] = m.addVar(
                    lb = 0.0, ub=gb.GRB.INFINITY,
                    vtype = gb.GRB.CONTINUOUS,
                    name = "x")
            elif t > (self.iter - bloco) and t <= self.iter:
                m._x.update(
                    m.addVars(((i,t) for i in m._I),
                    lb = 0.0, ub=gb.GRB.INFINITY,
                    vtype = gb.GRB.CONTINUOUS,
                    name = "x")
                )
                
                m._Ix.update(
                    m.addVars(((i,t) for i in m._I),
                    lb = 0.0, ub=gb.GRB.INFINITY,
                    vtype = gb.GRB.CONTINUOUS,
                    name = "stock_x")
                )

                m._y.update(
                    m.addVars(((i,t) for i in m._I),
                    lb = 0.0, ub=1.0,
                    vtype = gb.GRB.BINARY,
                    name = "y")
                )

            else:
                m._x.update(
                    m.addVars(((i,t) for i in m._I),
                    lb = 0.0, ub=gb.GRB.INFINITY,
                    vtype = gb.GRB.CONTINUOUS,
                    name = "x")
                )
                
                m._Ix.update(
                    m.addVars(((i,t) for i in m._I),
                    lb = 0.0, ub=gb.GRB.INFINITY,
                    vtype = gb.GRB.CONTINUOUS,
                    name = "stock_x")
                )

                m._y.update(
                    m.addVars(((i,t) for i in m._I),
                    lb = 0.0, ub=1.0,
                    vtype = gb.GRB.CONTINUOUS,
                    name = "y")
                )

        self.variables.x = m._x
        self.variables.Ix = m._Ix
        self.variables.y = m._y

        m.update()

    def _build_objective(self):
        self.model.setObjective(
            gb.quicksum(self.data.s[i]*self.variables.y[i,t] + self.data.h[i]*self.variables.Ix[i,t] for i in self.model._I for t in range(1, self.model._T[-1]+1))
            ,gb.GRB.MINIMIZE)

    def _build_constraints(self):
        m = self.model
        x = self.variables.x
        Ix = self.variables.Ix
        y = self.variables.y

        sp = self.data.sp
        d_tau = self.data.d_tau
        h = self.data.h
        b = self.data.b
        d = self.data.d

        def _u_constrs(i,t):
            if t == 1:
                return (Ix[i,t] - x[i,t] + d[i,t] == 0)
            else:
                return (Ix[i,t] - Ix[i,t-1] - x[i,t] + d[i,t] == 0)
        
        self.constraints.u_constrs = self.model.addConstrs(
            (_u_constrs(i, t) for i in self.model._I for t in self.model._T),
            name = "u_constrs"
        )
        print(f"# Original Problem | U constraints: {len(self.constraints.u_constrs)}")

        def _v_constrs(t):
            return (gb.quicksum(sp[i]*y[i,t] + b[i]*x[i,t] for i in self.model._I) <= self.data.Ct)
        
        self.constraints.v_constrs = self.model.addConstrs(
            (_v_constrs(t) for t in self.model._T),
            name = "v_constrs"
        )
        print(f"# Original Problem | V constraints: {len(self.constraints.v_constrs)}")

        def _w_constrs(i,t):
            return (x[i,t] <= minimo((self.data.Ct-self.data.sp[i])/self.data.b[i],self.data.d_tau[i])*y[i,t])

        self.constraints.w_constrs = self.model.addConstrs(
            (_w_constrs(i, t) for i in self.model._I for t in self.model._T),
            name = "w_constrs"
        )
        print(f"# Original Problem | W constraints: {len(self.constraints.w_constrs)}")

prev_m = None
bloco = 1
periodos = range(bloco,11+bloco,1*bloco)
for t in periodos:
    m = Relax_and_Fix(t, prev_m, bloco)
    m.optimize()
    prev_m = m
soma1 =0 
soma2 =0 
for i in range(1,6):
    for t in range(1,11):
        soma1 = soma1 + m.data.s[i]*m.variables.y[i,t]
        soma2 = soma2 + m.data.h[i]*m.variables.Ix[i,t].x
print("Objetivo: ", soma1+soma2, " Produção: ", soma1, " Estoque: ", soma2)