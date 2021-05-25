import gurobipy as gb

####
#   Benders decomposition via Gurobi + Python
#   Example 3.1 from Conejo et al.'s book on optimization techniques
####

##
# To Run:
# m = Benders_Master()
# m.optimize()
##


# Class which can have attributes set.
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


# Master problem
class Benders_Master:
    def __init__(self, benders_gap=0.001, max_iters=1000):
        self.max_iters = max_iters
        self.data = ProblemData()
        self.variables = expando()
        self.constraints = expando()
        self.results = expando()
        self._load_data(benders_gap=benders_gap)
        self._build_model()

    def optimize(self, simple_results=False):
        self.model.optimize()
        self.submodel = Benders_Subproblem(self)
        self.submodel.optimize()
        self._add_cut()
        self._update_bounds()
        self._save_vars()
        while self.data.ub > self.data.lb + self.data.benders_gap and len(self.data.cutlist) < self.max_iters:
            self.model.optimize()
            self.submodel = Benders_Subproblem(self)
            self.submodel.optimize()
            self._add_cut()
            self._update_bounds()
            self._save_vars()
            print(self.data.ys[len(self.data.cutlist)-1])
            print(self.data.ub, self.data.lb, len(self.data.cutlist))
            soma1 =0 
            soma2 =0 
            for i in range(1,6):
                for t in range(1,11):
                    soma1 = soma1 + m.data.s[i]*m.variables.y[i,t].x
            print(soma1, self.variables.z.x,soma1+self.variables.z.x )
        pass

    ###
    #   Loading functions
    ###

    def _load_data(self, benders_gap=0.001):
        self.data.cutlist = []
        self.data.upper_bounds = []
        self.data.lower_bounds = []
        self.data.lambdas = {}
        self.data.benders_gap = benders_gap
        self.data.ub = gb.GRB.INFINITY
        self.data.lb = -gb.GRB.INFINITY
        self.data.ys = []
        self.data.us = []
        self.data.vs = []
        self.data.ws = []
        self.data.xs = []
        self.data.zs = []

    ###
    #   Model Building
    ###
    def _build_model(self):
        self.model = gb.Model()
        self._build_variables()
        self._build_objective()
        self._build_constraints()
        self.model.update()

    def _build_variables(self):
        m = self.model

        self.model._I = range(1,6)
        self.model._T = range(1,11)
        
        # def _x_upper_bound():
        #     return {(i,t): minimo((self.data.Ct-self.data.sp[i])/self.data.b[i],self.data.d_tau[i])
        #         for i in self.model._I
        #         for t in self.model._T  
        #     }

        # m._x = \
        #     m.addVars(((i,t) for i in m._I for t in m._T),
        #     lb = 0.0, ub=_x_upper_bound(),
        #     vtype = gb.GRB.CONTINUOUS,
        #     name = "x")
        
        # m._Ix = \
        #     m.addVars(((i,t) for i in m._I for t in m._T),
        #     lb = 0.0, ub=1000,
        #     vtype = gb.GRB.CONTINUOUS,
        #     name = "stock_x")

        m._y = \
            m.addVars(((i,t) for i in m._I for t in m._T),
            lb = 0.0, ub=1.0,
            vtype = gb.GRB.BINARY,
            name = "y")

        m._z = \
            m.addVar(
            lb=-1000, ub=gb.GRB.INFINITY,
            vtype = gb.GRB.CONTINUOUS,
            name = "z")
    

        # self.variables.x = m._x
        # self.variables.Ix = m._Ix
        self.variables.y = m._y
        self.variables.z = m._z

        m.update()

    def _build_objective(self):
        self.model.setObjective(
            gb.quicksum(self.data.s[i]*self.model._y[i,t] for i in self.model._I for t in self.model._T) 
            + self.variables.z
            # + gb.quicksum(self.variables.z[t] for t in self.model._T)
            
            ,gb.GRB.MINIMIZE)

    def _build_constraints(self):
        self.constraints.cuts = {}
        pass

    ###
    # Cut adding
    ###
    def _add_cut(self):
        y = self.variables.y
        cut = len(self.data.cutlist)
        self.data.cutlist.append(cut)
        z_sub = self.submodel.model.ObjVal
        
        optimal_y = {
        (1,1):1, (1,2):1, (1,3):1, (1,4):1, (1,5):1, (1,6):1, (1,7):1, (1,8):1, (1,9):0, (1,10):1,
        (2,1):1, (2,2):1, (2,3):0, (2,4):0, (2,5):0, (2,6):0, (2,7):1, (2,8):0, (2,9):0, (2,10):1,
        (3,1):1, (3,2):1, (3,3):1, (3,4):0, (3,5):0, (3,6):1, (3,7):0, (3,8):1, (3,9):0, (3,10):1,
        (4,1):1, (4,2):0, (4,3):1, (4,4):0, (4,5):1, (4,6):1, (4,7):0, (4,8):1, (4,9):1, (4,10):0,
        (5,1):1, (5,2):0, (5,3):0, (5,4):1, (5,5):0, (5,6):0, (5,7):0, (5,8):0, (5,9):0, (5,10):0
        }

        # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        #  0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0,
        #   0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0,
        #    0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0,
        #     0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0]


        if z_sub < 100000:
            self.constraints.cuts[cut] = self.model.addConstr(
                    gb.quicksum(
                    self.submodel.variables.u[i,t].x*(-self.data.d[i,t])
                    for i in self.model._I
                    for t in self.model._T) 
                    +
                    gb.quicksum(
                    self.submodel.variables.v[t].x*(self.data.Ct - gb.quicksum(self.data.sp[i]*self.variables.y[i,t]
                    for i in self.model._I
                    ))
                    for t in self.model._T )
                    +
                    gb.quicksum(
                    self.submodel.variables.w[i,t].x*(minimo((self.data.Ct-self.data.sp[i])/self.data.b[i],self.data.d_tau[i])*self.variables.y[i,t])
                    for i in self.model._I
                    for t in self.model._T
                    )
                    
                    <=
                    self.variables.z
                )
        else:
            self.constraints.cuts[cut] = self.model.addConstr(
                    gb.quicksum(
                    (self.submodel.variables.u[i,t].x*(-self.data.d[i,t]) 
                    +
                    self.submodel.variables.v[t].x*(self.data.Ct - self.data.sp[i]*self.variables.y[i,t])#self.data.sp[i]*self.variables.y[i,t])
                    +
                    self.submodel.variables.w[i,t].x*(minimo((self.data.Ct-self.data.sp[i])/self.data.b[i],self.data.d_tau[i])*self.variables.y[i,t]))
                    for i in self.model._I
                    for t in self.model._T
                    )
                    <=
                    0
                )
        # if z_sub < 30:
        #     self.constraints.cuts["heuristica"] = self.model.addConstr(
        #             -200
        #             <=
        #             self.variables.z
        #         )


    ###
    # Update upper and lower bounds
    ###
    def _update_bounds(self):
        obj_sub = self.submodel.model.ObjVal
        obj_master = self.model.ObjVal
        self.data.ub = minimo(obj_sub,self.data.ub)

            # sum([self.variables.z[x].x for x in self.variables.z]) \
        # The best lower bound is the current bestbound,
        # This will equal z_master at optimality
        self.data.lb = obj_master
        self.data.upper_bounds.append(self.data.ub)
        self.data.lower_bounds.append(self.data.lb)

    def _save_vars(self):
        u = [self.submodel.variables.u[x].x for x in self.submodel.variables.u]
        v = [self.submodel.variables.v[x].x for x in self.submodel.variables.v]
        w = [self.submodel.variables.w[x].x for x in self.submodel.variables.w]
        y = [self.variables.y[x].x for x in self.variables.y]
        z = self.variables.z
        fix_y = [self.submodel.variables.fix_y[x] for x in self.submodel.variables.fix_y]
        self.data.ys.append(y)
        self.data.us.append(u)
        self.data.vs.append(v)
        self.data.ws.append(w)
        self.data.zs.append(z)


# Subproblem
class Benders_Subproblem:
    def __init__(self, MP):
        self.data = ProblemData()
        self.variables = expando()
        self.constraints = expando()
        self.results = expando()
        self._build_model()
        self.MP = MP
        self.update_fixed_vars()
        self._build_objective()
        self.model.update()


    def optimize(self):
        self.model.optimize()

    ###
    #   Model Building
    ###
    def _build_model(self):
        self.model = gb.Model()
        self._build_variables()
        self._build_constraints()
        self.model.update()

    def _build_variables(self):
        m = self.model

        self.model._I = range(1,6)
        self.model._T = range(1,11)
        
        m._u = \
            m.addVars(((i,t) for i in m._I for t in m._T),
            lb = -gb.GRB.INFINITY, ub=10000,
            vtype = gb.GRB.CONTINUOUS,
            name = "u")
        
        m._v = \
            m.addVars(((t) for t in m._T),
            lb = -gb.GRB.INFINITY, ub=0,
            vtype = gb.GRB.CONTINUOUS,
            name = "v")

        m._w = \
            m.addVars(((i,t) for i in m._I for t in m._T),
            lb = -gb.GRB.INFINITY, ub=0, #removi
            vtype = gb.GRB.CONTINUOUS,
            name = "w")

        
        # m._fix_y = \
        #     m.addVars(((i,t) for i in m._I for t in m._T),
        #     lb = 0.0, ub=1.0,
        #     vtype = gb.GRB.CONTINUOUS,
        #     name = "fix_y")

        self.variables.u = m._u
        self.variables.v = m._v
        self.variables.w = m._w
        # self.variables.fix_y = m._fix_y

        
        self.variables.fix_y = {
        (1,1):1, (1,2):1, (1,3):1, (1,4):1, (1,5):1, (1,6):1, (1,7):1, (1,8):1, (1,9):0, (1,10):1,
        (2,1):1, (2,2):1, (2,3):0, (2,4):0, (2,5):0, (2,6):0, (2,7):1, (2,8):0, (2,9):0, (2,10):1,
        (3,1):1, (3,2):1, (3,3):1, (3,4):0, (3,5):0, (3,6):1, (3,7):0, (3,8):1, (3,9):0, (3,10):1,
        (4,1):1, (4,2):0, (4,3):1, (4,4):0, (4,5):1, (4,6):1, (4,7):0, (4,8):1, (4,9):1, (4,10):0,
        (5,1):1, (5,2):0, (5,3):0, (5,4):1, (5,5):0, (5,6):0, (5,7):0, (5,8):0, (5,9):0, (5,10):0
        }


        m.update()

    def _build_objective(self):
        m = self.model

        self.model.setObjective(
            gb.quicksum(-self.data.d[i,t]*self.variables.u[i,t] for i in self.model._I for t in self.model._T)
            +
            gb.quicksum(self.variables.v[t]*self.data.Ct for t in self.model._T) - gb.quicksum(self.model._v[t]*gb.quicksum(self.data.sp[i]*self.variables.fix_y[i,t] for i in self.model._I) for t in self.model._T)
            +
            gb.quicksum(
                minimo((self.data.Ct-self.data.sp[i])/self.data.b[i],self.data.d_tau[i])*self.variables.fix_y[i,t]*self.variables.w[i,t]
                for i in self.model._I for t in self.model._T
            )
            , gb.GRB.MAXIMIZE)

    def _build_constraints(self):
        m = self.model
        u = self.variables.u
        v = self.variables.v
        w = self.variables.w
        y = self.variables.fix_y
        h = self.data.h
        b = self.data.b

        def _general_constrs(i,t):
            return (-u[i,t] + b[i]*v[t] + w[i,t] <= 0) #b[i]*v[t] + w[i,t] <= 0)  
        
        self.constraints.general_constrs = self.model.addConstrs(
            (_general_constrs(i, t) for i in self.model._I for t in self.model._T),
            name = "general_constrs"
        )
        print(f"# Dual Subproblem | General constraints: {len(self.constraints.general_constrs)}")

        def _u_constrs(i,t):
            if t == self.model._T[0]:
                return (-u[i,t] <= h[i])
            elif t == self.model._T[-1]:
                return (u[i,t] <= h[i])
            else:
                return True
        
        self.constraints.u_constrs = self.model.addConstrs(
            (_u_constrs(i, t) for i in self.model._I for t in [1,10]),
            name = "u_constrs"
        )
        print(f"# Dual Subproblem | U constraints: {len(self.constraints.u_constrs)}")

        def _u_constrs2(i,t):
            if t == self.model._T[0]:
                return True
            else:
                return (u[i,t-1] - u[i,t] <= h[i])
        
        self.constraints.u_constrs2 = self.model.addConstrs(
            (_u_constrs2(i, t) for i in self.model._I for t in self.model._T[1:]),
            name = "u_constrs2"
        )
        print(f"# Dual Subproblem | U constraints: {len(self.constraints.u_constrs2)}")


        # def _y_constrs(i,t):
        #     return (y[i,t] == self.variables.fix_y[i,t])

        # self.constraints.y_constrs = self.model.addConstrs(
        #     (_y_constrs(i, t) for i in self.model._I for t in self.model._T),
        #     name = "y_constrs"
        # )
        # print(f"# Dual Subproblem | Y constraints: {len(self.constraints.y_constrs)}")

    def update_fixed_vars(self, MP=None):
        if MP is None:
            MP = self.MP
        #para testes
        for i in self.model._I:
            for t in self.model._T:
                self.variables.fix_y[i,t] = MP.variables.y[i,t].x #y_optimal[i,t] 

m = Benders_Master()
m.optimize()