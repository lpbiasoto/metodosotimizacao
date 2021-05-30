import gurobipy as gb

#classe genérica que vai receber objetos do gurobi
class generic(object):
    pass

#função para cálculo do mínimo
def minimo(a, b):
    if a < b:
        return a
    else:
        return b

#classe contendo os dados do problema
class ProblemData():
    items = range(1,6)
    time = range(1,11)
    s = {1:4.5, 2:6.2, 3:4.2, 4:4.0, 5:9.5}
    h = {1:0.9, 2:0.4, 3:0.8, 4:0.5, 5:0.1}
    b = {1:0.8, 2:0.6, 3:0.7, 4:0.2, 5:0.5}
    sp = {1:9.2, 2:6.7, 3:6.7, 4:7.2, 5:4.3}
    Ct = 48.5
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


# classe do Problema Mestre Restrito, contendo todo o fluxo de execução do algoritimo
class Benders_Master:

    #definição inicial dos dados do problema, instâncias iniciais das variáveis, restrições e resultados
    def __init__(self, benders_gap=0.001, max_iters=1000):
        self.max_iters = max_iters
        self.data = ProblemData()
        self.variables = generic()
        self.constraints = generic()
        self.results = generic()
        self._load_data(benders_gap=benders_gap)
        self._build_model()
        self.iter = 1

    #fluxo de execução quando o comando m.optimize() é executado
    def optimize(self, simple_results=False):
        #otimiza o problema mestre restrito
        self.model.optimize()

        #define o subproblema com as variáveis de y resultantes da otimização do problema mestre restrito
        self.submodel = Benders_Subproblem(self)
        #otimiza o subproblema
        self.submodel.optimize()
        #adiciona os cortes no problema mestre restrito conforme resultados obtidos na otimização do subproblema
        self._add_cut()
        #atualiza os upper e lower bounds
        self._update_bounds()
        #salva as variáveis resultantes em listas
        self._save_vars()

        #inicia o loop do algoritmo de benders com os critérios de parada definidos
        while self.data.ub > self.data.lb + self.data.benders_gap and self.iter < self.max_iters:

            #otimiza o problema mestre restrito com os novos cortes
            self.model.optimize()

            #define o novo subproblema com as variáveis de y resultantes da otimização do problema mestre restrito
            self.submodel = Benders_Subproblem(self)
            
            #otimiza o subproblema
            self.submodel.optimize()
            #adiciona os cortes no problema mestre restrito conforme resultados obtidos na otimização do subproblema
            self._add_cut()
            #atualiza os upper e lower bounds
            self._update_bounds()
            #salva as variáveis resultantes em listas
            self._save_vars()

            #atualiza o número de iterações
            self.iter = self.iter + 1

            #printa na janela de comando os resultados obtidos
            print("y_otimo_benders[i,t] = ",self.data.ys[-1])
            print("Upper Bound: ",self.data.ub)
            print("Lower Bound: ",self.data.lb)
            print("Iteração: ", len(self.data.ys))
            soma1 =0 
            soma2 =0 
            for i in range(1,6):
                for t in range(1,11):
                    soma1 = soma1 + m.data.s[i]*m.variables.y[i,t].x
            print("Custo Yit: ",soma1)
            print("Obj Subproblema: ", self.submodel.model.ObjVal)
            print("Variável Z: ",sum([self.variables.z[t].x for t in self.variables.z]))
            print("Obj Problema Mestre: ",self.model.ObjVal )
        pass

    #declaração inicial dos vetores que armazenam os resultados e do upper e lower bounds
    def _load_data(self, benders_gap=0.001):
        self.data.cutlist = []
        self.data.upper_bounds = []
        self.data.lower_bounds = []
        self.data.benders_gap = benders_gap
        self.data.ub = gb.GRB.INFINITY
        self.data.lb = -gb.GRB.INFINITY
        self.data.ys = []
        self.data.us = []
        self.data.vs = []
        self.data.ws = []
        self.data.xs = []
        self.data.zs = []

    #fluxo de estruturação do problema mestre
    def _build_model(self):
        #nova instância do gurobi
        self.model = gb.Model()

        #criação das variáveis
        self._build_variables()

        #definição da função objetivo
        self._build_objective()

        #definição das restrições do problema mestre restrito
        self._build_constraints()

        self.model.update()

    #definição das variáveis do problema mestre
    def _build_variables(self):
        m = self.model

        #definição dos índices I = itens, T = períodos
        self.model._I = range(1,6)
        self.model._T = range(1,11)
        
        #problema mestre restrito possui duas variáveis: y, binária, e z[t], contínua livre
        m._y = \
            m.addVars(((i,t) for i in m._I for t in m._T),
            lb = 0.0, ub=1.0,
            vtype = gb.GRB.BINARY,
            name = "y")

        m._z = \
            m.addVars(((t) for t in m._T),
            lb=-gb.GRB.INFINITY, ub=gb.GRB.INFINITY,
            vtype = gb.GRB.CONTINUOUS,
            name = "z")
    
        self.variables.y = m._y
        self.variables.z = m._z

        m.update()

    #definição da função objetivo do problema mestre restrito
    def _build_objective(self):
        self.model.setObjective(
            gb.quicksum(self.data.s[i]*self.model._y[i,t] for i in self.model._I for t in self.model._T) 
            + gb.quicksum(self.variables.z[t] for t in self.model._T)
            ,gb.GRB.MINIMIZE)

    #não foi definida nenhuma restrição inicial para o problema mestre restrito,
    #portanto essa função faz somente a inicialização do dicionário dos cortes
    def _build_constraints(self):
        self.constraints.cuts = {}

        # def _general_constrs(i):
        #     return (gb.quicksum(self.variables.y[i,t] for t in self.model._T) >= 1)
        
        # self.constraints.general_constrs = self.model.addConstrs(
        #     (_general_constrs(i) for i in self.model._I),
        #     name = "general_constrs"
        # )
        # print(f"# Master Problem | General constraints: {len(self.constraints.general_constrs)}")

        pass

    def _add_cut(self):

        #variável y dos cortes é a variável do problema mestre restrito
        y = self.variables.y

        #cortes utilizam o resultado das variáveis v, u e w do subproblema de benders
        v = self.submodel.variables.v
        u = self.submodel.variables.u
        w = self.submodel.variables.w

        #variável z é o limitante do problema mestre restrito
        z = self.variables.z

        cut = self.iter

        #resultado da função objetivo do dual vai determinar se é um ponto extremo ou um raio extremo
        z_sub = self.submodel.model.ObjVal

        
        #cortes de benders principais (para todos os períodos e itens)
        self.data.cutlist.append(str(cut)+"_geral")
        #se obj do dual for menor que 500, então é ponto extremo, senão é raio extremo
        if z_sub < 500:
            self.constraints.cuts[str(cut)+"_geral"] = self.model.addConstr(
                    gb.quicksum(
                    u[i,t].x*(-self.data.d[i,t])
                    for i in self.model._I
                    for t in self.model._T) 
                    +
                    gb.quicksum(
                    v[t].x*(self.data.Ct - gb.quicksum(self.data.sp[i]*y[i,t]
                    for i in self.model._I
                    ))
                    for t in self.model._T)
                    +
                    gb.quicksum(
                    w[i,t].x*(minimo(((self.data.Ct-self.data.sp[i])/self.data.b[i]),self.data.d_tau[i])*y[i,t])
                    for i in self.model._I
                    for t in self.model._T
                    )
                    <=
                    gb.quicksum(z[t] for t in self.model._T)
                )
        else:
            self.constraints.cuts[str(cut)+"_geral"] = self.model.addConstr(
                    gb.quicksum(
                    self.submodel.variables.u[i,t].x*(-self.data.d[i,t])
                    for i in self.model._I
                    for t in self.model._T) 
                    +
                    gb.quicksum(self.submodel.variables.v[t].x*(self.data.Ct - gb.quicksum(self.data.sp[i]*self.variables.y[i,t]
                    for i in self.model._I
                    ))
                    for t in self.model._T)
                    +
                    gb.quicksum(
                    self.submodel.variables.w[i,t].x*(minimo(((self.data.Ct-self.data.sp[i])/self.data.b[i]),self.data.d_tau[i])*self.variables.y[i,t])
                    for i in self.model._I
                    for t in self.model._T
                    )
                    <=
                    0
                )


        #teste adicionando cortes de benders separados por períodos para tentar acelerar a convergência
        #para desabilitá-lo, basta comentar esta cadeia de for
        for T in self.model._T:
            self.data.cutlist.append(str(cut)+"_"+str(T))
            if z_sub < 500:
                self.constraints.cuts[str(cut)+"_"+str(T)] = self.model.addConstr(
                        gb.quicksum(
                        self.submodel.variables.u[i,T].x*(-self.data.d[i,T])
                        for i in self.model._I) 
                        +
                        self.submodel.variables.v[T].x*(self.data.Ct - gb.quicksum(self.data.sp[i]*self.variables.y[i,T]
                        for i in self.model._I
                        ))
                        +
                        gb.quicksum(
                        self.submodel.variables.w[i,T].x*(minimo(((self.data.Ct-self.data.sp[i])/self.data.b[i]),self.data.d_tau[i])*self.variables.y[i,T])
                        for i in self.model._I
                        )
                        
                        <=
                        self.variables.z[T]
                    )
            else:
                self.constraints.cuts[str(cut)+"_"+str(T)] = self.model.addConstr(
                        gb.quicksum(
                        self.submodel.variables.u[i,T].x*(-self.data.d[i,T])
                        for i in self.model._I) 
                        +
                        self.submodel.variables.v[T].x*(self.data.Ct - gb.quicksum(self.data.sp[i]*self.variables.y[i,T]
                        for i in self.model._I
                        ))
                        +
                        gb.quicksum(
                        self.submodel.variables.w[i,T].x*(minimo(((self.data.Ct-self.data.sp[i])/self.data.b[i]),self.data.d_tau[i])*self.variables.y[i,T])
                        for i in self.model._I
                        )
                        
                        <=
                        0
                    )

    #atualiza o lower e upper bounds
    def _update_bounds(self):
        
        custo_y = 0 
        custo_z = 0
        for t in self.model._T:
            for i in self.model._I:
                custo_y = custo_y + m.data.s[i]*m.variables.y[i,t].x
            custo_z = custo_z + m.variables.z[t].x
        
        obj_sub = self.submodel.model.ObjVal + custo_y
        obj_master = custo_y + custo_z

        #upper bound é o mínimo entre o upper bound anterior e o objetivo do subproblema dual + custo dos y's
        self.data.ub = minimo(obj_sub,self.data.ub)

        #lower bound é o custo dos y's mais a variável limitante z
        self.data.lb = obj_master

        self.data.upper_bounds.append(self.data.ub)
        self.data.lower_bounds.append(self.data.lb)

    #salva as informações do problema mestre restrito e subproblema em lista para gravar histórico
    def _save_vars(self):
        u = [self.submodel.variables.u[x].x for x in self.submodel.variables.u]
        v = [self.submodel.variables.v[x].x for x in self.submodel.variables.v]
        w = [self.submodel.variables.w[x].x for x in self.submodel.variables.w]
        y = [self.variables.y[x].x for x in self.variables.y]
        z = [self.variables.z[x].x for x in self.variables.z]
        fix_y = [self.submodel.variables.fix_y[x] for x in self.submodel.variables.fix_y]
        self.data.ys.append(y)
        self.data.us.append(u)
        self.data.vs.append(v)
        self.data.ws.append(w)
        self.data.zs.append(z)


#subproblema
class Benders_Subproblem:

    #definição inicial dos dados do problema, instâncias iniciais das variáveis, restrições e resultados
    def __init__(self, MP):
        self.data = ProblemData()
        self.variables = generic()
        self.constraints = generic()
        self.results = generic()
        self._build_model()
        self.MP = MP
        self.update_fixed_vars()
        self._build_objective()
        self.model.update()

    #otimiza o subproblema
    def optimize(self):
        self.model.optimize()

    #monta o subproblema dual
    def _build_model(self):

        #nova instância do gurobi
        self.model = gb.Model()
        #define as variáveis do subproblema
        self._build_variables()
        #define as restrições do subproblema
        self._build_constraints()

        self.model.update()

    #define as variáveis do subproblema
    def _build_variables(self):
        m = self.model

        #índices I = itens e T = períodos
        self.model._I = range(1,6)
        self.model._T = range(1,11)
        
        #variáveis do subproblema: uit, contínua e livre; vt, contínua negativa; e wit, contínua negativa
        m._u = \
            m.addVars(((i,t) for i in m._I for t in m._T),
            lb = -gb.GRB.INFINITY, ub=gb.GRB.INFINITY,
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

        self.variables.u = m._u
        self.variables.v = m._v
        self.variables.w = m._w

        #definida a versão inicial dos parâmetros de yit - solução ótima do problema original
        self.variables.fix_y = {
        (1,1):1, (1,2):1, (1,3):1, (1,4):1, (1,5):1, (1,6):1, (1,7):1, (1,8):1, (1,9):0, (1,10):1,
        (2,1):1, (2,2):1, (2,3):0, (2,4):0, (2,5):0, (2,6):0, (2,7):1, (2,8):0, (2,9):0, (2,10):1,
        (3,1):1, (3,2):1, (3,3):1, (3,4):0, (3,5):0, (3,6):1, (3,7):0, (3,8):1, (3,9):0, (3,10):1,
        (4,1):1, (4,2):0, (4,3):1, (4,4):0, (4,5):1, (4,6):1, (4,7):0, (4,8):1, (4,9):1, (4,10):0,
        (5,1):1, (5,2):0, (5,3):0, (5,4):1, (5,5):0, (5,6):0, (5,7):0, (5,8):0, (5,9):0, (5,10):0
        }

        m.update()

    #definição do objetivo do subproblema dual
    def _build_objective(self):
        m = self.model

        self.model.setObjective(
            gb.quicksum(-self.data.d[i,t]*self.variables.u[i,t] for i in self.model._I for t in self.model._T)
            +
            gb.quicksum(self.variables.v[t]*self.data.Ct for t in self.model._T) - gb.quicksum(self.variables.v[t]*gb.quicksum(self.data.sp[i]*self.variables.fix_y[i,t] for i in self.model._I) for t in self.model._T)
            +
            gb.quicksum(
                minimo((self.data.Ct-self.data.sp[i])/self.data.b[i],self.data.d_tau[i])*self.variables.fix_y[i,t]*self.variables.w[i,t]
                for i in self.model._I for t in self.model._T
            )
            , gb.GRB.MAXIMIZE)

    #restrições do subproblema dual
    def _build_constraints(self):
        m = self.model
        u = self.variables.u
        v = self.variables.v
        w = self.variables.w
        y = self.variables.fix_y
        h = self.data.h
        b = self.data.b

        #restrições de xit
        def _general_constrs(i,t):
            return (-u[i,t] + b[i]*v[t] + w[i,t] <= 0)  #sum(b.values())*v[t] + w[i,t] <= 0) #
        
        self.constraints.general_constrs = self.model.addConstrs(
            (_general_constrs(i, t) for i in self.model._I for t in self.model._T),
            name = "general_constrs"
        )
        print(f"# Dual Subproblem | General constraints: {len(self.constraints.general_constrs)}")

        #contornos para a variável uit nos períodos extremos (Iit)
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

        #limite de variação de uit entre períodos (Iit)
        def _u_constrs2(i,t):
            return (u[i,t] - u[i,t+1] <= h[i])
        
        self.constraints.u_constrs2 = self.model.addConstrs(
            (_u_constrs2(i, t) for i in self.model._I for t in range(1,10)),
            name = "u_constrs2"
        )
        print(f"# Dual Subproblem | U constraints: {len(self.constraints.u_constrs2)}")

    #atualização dos parâmetros de yit com o resultado das variáveis da otimização do problema mestre restrito
    def update_fixed_vars(self, MP=None):
        if MP is None:
            MP = self.MP

        # comentar as três linhas abaixo para testes com a solução ótima do problema original
        for i in self.model._I:
            for t in self.model._T:
                self.variables.fix_y[i,t] = MP.variables.y[i,t].x

#fluxo de execução do programa: definição do problema e otimização
m = Benders_Master()
m.optimize()