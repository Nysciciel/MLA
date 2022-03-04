#=
model_2:
- Julia version: 
- Author: Vincent
- Date: 2022-01-27
=#

using JuMP, Random, CPLEX
model = Model(CPLEX.Optimizer)

Random.seed!(1234)  # initialisation du germe

#donnees
n=1000  # nombre de points au sol
m=50  # nombre de positions de drones
Rcouv=30 #rayon de couverture des drones
Rcom =15 #rayon de communication des drones
L=20     #altitude
p = 10     # nombre de drones à placer

#Les coordonnees d'un point i sont (abs_sol, ord_sol, 0)
abs_sol=[100*rand() for i in 1:n]
ord_sol=[100*rand() for i in 1:n]
#Les coordonnees d'une position j sont (abs_alt, ord_alt, L)
abs_alt=[100*rand() for j in 1:m]
ord_alt=[100*rand() for j in 1:m]



d = [0 for i in 1:n, j in 1:m]
for i in 1:n, j in 1:m
    d[i,j]=round(sqrt( (abs_sol[i]-abs_alt[j])^2 + (ord_sol[i]-ord_alt[j])^2 + L^2 )) <= Rcouv
end
d_com = [0 for j in 1:m, j1 in 1:m]
for j in 1:m, j1 in 1:m
    d_com[j,j1]=round(sqrt( (abs_alt[j]-abs_alt[j1])^2 + (ord_alt[j]-ord_alt[j1])^2 )) <= Rcom
end


@variable(model, x[1:n], Bin) # client k est couvert
@variable(model, y[1:m], Bin)# Site i ouvert

@objective(model, Max, sum(x)) # on maximise les client servis

@constraint(model, sum(y) <= p, base_name="sites number")# avec seulement p sites
@constraint(model, [k=1:n], x[k] <= sum(y[i]*d[k,i] for i in 1:m), base_name=string("client cover_",k)) # Client couvert si site ouvert et arête existe


# Contraintes d'arbre couvrant
@variable(model, z[i=1:m, j=1:m], Bin)# arête i,j de G séléctionnée
@variable(model, l[1:m] >=0, Int) #nombre de sommets visités entre 1 et i

#@contraint(model, z[i,j] = z[j,i])
@constraint(model, [i=1:m, j=1:m], 2*z[i,j] <= d_com[i,j] * (y[i] + y[j]), base_name=string("edge selection_",i,"_",j))# arête séléctionnée si existe et si sites ouverts
@constraint(model, sum(z) == sum(y) - 1, base_name=string("edge number_"))
@constraint(model, [i=1:m, j=1:m], l[j] >= l[i] + 1 - m*(1 - z[i,j]), base_name=string("sites order_",i,"_",j))

optimize!(model)
if has_values(model)
    print(solution_summary(model, verbose=true))
else
    print("No solution")
end