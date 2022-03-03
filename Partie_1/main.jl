using CPLEX
using JuMP
using Combinatorics

model = Model(CPLEX.Optimizer)
p = 3
g_ = [
1 0 0 1 0
0 1 0 0 0
0 1 1 0 0
1 0 0 1 0
0 0 0 1 0
1 1 0 0 0
0 1 0 0 0
0 1 1 0 0
0 0 0 1 0
1 1 1 0 0
]
K,N = size(g_)

g = [
0 0 0 1 1
0 0 0 0 1
0 0 0 1 1
1 0 1 0 1
1 1 1 1 0
]
@variable(model, x[1:N, 1:K], Bin) # Site i couvre client k
@variable(model, y[1:N], Bin)# Site i ouvert

@objective(model, Max, sum(x)) # on maximise les client servis

@constraint(model, sum(y) <= p, base_name="sites number")# avec seulement p sites

@constraint(model, [i=1:N, k=1:K], x[i,k] <= y[i]*g_[k,i], base_name=string("client cover_",i,"_",k)) # Client couvert si site ouvert et arête existe

@constraint(model, [k=1:K], sum(x[:,k]) <= 1, base_name=string("cover unicity_",k))# Client couvert une seule fois


# Contraintes d'arbre couvrant
@variable(model, z[1:N, 1:N], Bin)# arête i,j de G séléctionnée
@constraint(model, [i=1:N, j=1:N], 2*z[i,j] <= g[i,j] * (y[i] + y[j]), base_name=string("edge selection_",i,"_",j))# arête séléctionnée si existe et si sites ouverts
@constraint(model, [i=1:N, j=1:N], sum(z)/2 == sum(y) - 1, base_name=string("edge number_",i,"_",j))
for sites in powerset(1:N)
    if size(sites)==(0,)
        continue
    end
    @constraint(model, sum(z[sites,sites])/2 <= size(sites)[1]-1)
end

optimize!(model)
if has_values(model)
    print(solution_summary(model, verbose=true))
else
    print("No solution")
end