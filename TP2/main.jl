#=
main:
- Julia version: 
- Author: Vincent
- Date: 2022-02-17
=#
using CPLEX
using JuMP
n = 6
f = [7, 2, 2, 7, 7, 1]
c = [66, 5, 4, 3, 2]
d = 3



function sous_pb(y)
    sous_model = Model(CPLEX.Optimizer)
    @variable(sous_model, b)
    @variable(sous_model, v[1:n-1]>=0)

    @constraint(sous_model, [i=1:n-1], b-v[i] <= c[i])

    @objective(sous_model, Max, d*b - y'*v)

    optimize!(sous_model)
    println(termination_status(sous_model))
    if termination_status(sous_model) == INFEASIBLE_OR_UNBOUNDED
        @constraint(sous_model, b+sum(v) == 1000)
        optimize!(sous_model)
        println("faire une coupe <=0")
        println(value(b), value.(v))
        return false, value(b), value.(v)
    end

    if has_values(sous_model)
        return true, value(b), value.(v)
    else
        error("NO_SOLUTION")
    end

end

model = Model(CPLEX.Optimizer)

@variable(model, y[1:n-1], Bin)
@variable(model, w >=0, Int)

@constraint(model, y[1]>=y[2])
@constraint(model, y[1]>=y[3])

@objective(model, Min, f'*[y; w])
optimize!(model)

y_val = value.(y)

i=0
while true
    global i
    println(i)
    i+=1
    global y_val
    status, b_val,v_val = sous_pb(y_val)
    if status && value(w) >= d*b_val - y_val'*v_val
       break
    elseif !status
        println("siduyfgsdigus")
        @constraint(model, d*b_val - y'*v_val <= 0)
        optimize!(model)
        y_val = value.(y)
        continue
    end
    @constraint(model, w >= d*b_val - y'*v_val)
    optimize!(model)
    y_val = value.(y)
end