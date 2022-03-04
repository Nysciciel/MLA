"""
Résoud le problème suivant:
    min f'.y + c'.x
        st  y_1 ≥ y_2, y_1 ≥ y_3
            ∑ x_i = d
            x_i ≤ y_i ∀ i
            x_i ≥ 0, y_i ∈ {0,1} ∀ i
Retourne (y*,x*) solution du pb
"""
function solve_PL(n::Int, d::Int, f::Array{Int}, c::Array{Int}, benders::Bool=true)
    model = Model(CPLEX.Optimizer)
    set_silent(model)

    x = @variable(model, x[1:n] ≥ 0)
    y = @variable(model, y[1:n], Bin)

    @objective(model, Min, f[1:n]'*y + c'*x)

    @constraint(model, y[1] ≥ y[2])
    @constraint(model, y[1] ≥ y[3])
    @constraint(model, sum(x) == d)
    @constraint(model, [i=1:n], x[i] ≤ y[i])

    if benders
        set_optimizer_attribute(model, "CPXPARAM_Benders_Strategy",3)
    end
    optimize!(model)
    
    if has_values(model)
        return value.(x), value.(y), objective_value(model), solve_time(model)
    else
        println("NO_SOLUTION")
        return nothing
    end
end