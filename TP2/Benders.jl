"""
Résoud le problème de séparation:
    max_{b,v} d.b - y'.v
        st  b - v_i ≤ c_i ∀ i
            b ∈ ℝ, v ≥ 0
Retourne 
    - true, b, v si le pb est borné
    - fale, b, v si le pb est non borné (b et v forment un rayon)
"""
function sous_pb(y::Vector{Float64}, d::Int, c::Array{Int})
    n = length(y)
    sous_model = Model(CPLEX.Optimizer)
    @variable(sous_model, b)
    @variable(sous_model, v[1:n]>=0)

    @constraint(sous_model, [i=1:n], b-v[i] <= c[i])

    @objective(sous_model, Max, d*b - y'*v)
    set_silent(sous_model)
    optimize!(sous_model)

    if termination_status(sous_model) == INFEASIBLE_OR_UNBOUNDED
        @constraint(sous_model, b+sum(v) == 1000*d)
        optimize!(sous_model)
        return false, value(b), value.(v)
    end

    if has_values(sous_model)
        return true, value(b), value.(v)
    else
        error("NO_SOLUTION")
    end

end

"""
Résoud le problème maître:
    min f'.[y;w]
        st  y_1 ≥ y_2, y_1 ≥ y_3
            w ≥ d.b* - y'.v*
            w ≥ 0, y_i ∈ {0,1} ∀ i
Retourne y* solution du pb
"""
function benders_solve(n::Int, d::Int, f::Array{Int}, c::Array{Int})
    model = Model(CPLEX.Optimizer)

    @variable(model, y[1:n], Bin)
    @variable(model, w >=0, Int)
    
    @constraint(model, y[1]>=y[2])
    @constraint(model, y[1]>=y[3])
    
    @objective(model, Min, f'*[y; w])
    set_silent(model)
    optimize!(model)
    while true
        status, b_val, v_val = sous_pb(value.(y), d, c)
        if status && value(w) >= d*b_val - value.(y)'*v_val
           break
        elseif !status
            println("rayon")
            println(b_val, "\t", v_val)
            @constraint(model, d*b_val - y'*v_val <= 0)
            optimize!(model)
            continue
        end
        println("coupe")
        @constraint(model, w >= d*b_val - y'*v_val)
        
        optimize!(model)
    end
    optimize!(model)
    println(model)
    return value.(y), value(w), objective_value(model)
end