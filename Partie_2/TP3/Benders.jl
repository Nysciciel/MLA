"""
Résoud le problème de séparation:
    max_{b,v} d.b - y'.v
        st  b - v_i ≤ c_i ∀ i
            b ∈ ℝ, v ≥ 0
Retourne 
    - true, b, v si le pb est borné
    - fale, b, v si le pb est non borné (b et v forment un rayon)
"""

function sous_pb(y::Matrix{Float64}, b::Int; bound_ray::Bool=true)
    model = Model(CPLEX.Optimizer)
    set_optimizer_attribute(model, "CPX_PARAM_PREIND", 0) #Pas de présolve
    set_optimizer_attribute(model, "CPX_PARAM_PARALLELMODE", -1) #Parallèle opportuniste
    set_silent(model)

    @variable(model, v[1:n,1:n] ≥ 0)
    @variable(model, ν[1:n] ≥ 0)
    @constraint(model, ν[1] == 0 )
    @constraint(model, [i=1:n,j=1:n;i<j], adj[i,j]*(-v[i,j] - ν[i] + ν[j]) ≤ 0)
    @constraint(model, [i=1:n,j=1:n;i<j], adj[i,j]*(-v[i,j] + ν[i] - ν[j]) ≤ 0)
    @objective(model, Max, -sum(b*adj.*y.*v) + demande'*ν)
    
    optimize!(model)

    if bound_ray && termination_status(model) == MOI.DUAL_INFEASIBLE
        @constraint(model, sum(v) + sum(ν) == 1)
        optimize!(model)
    end
    return termination_status(model), value.(v), value.(ν), objective_value(model)
end

"""
Résoud le problème maître:
    min f'.[y;w]
        st  y_1 ≥ y_2, y_1 ≥ y_3
            w ≥ d.b* - y'.v*
            w ≥ 0, y_i ∈ {0,1} ∀ i
Retourne y* solution du pb
"""
function benders_solve(b::Int;time_limit::Float64=300.0, relaxation::Bool=true, bound_ray::Bool=true)
    start = time()
    model = Model(CPLEX.Optimizer)
    set_time_limit_sec(model, time_limit)
    set_silent(model)

    @variable(model, y[1:n, 1:n] ≥ 0, Int)
    
    @objective(model, Min, sum(y))
    

    if relaxation
        unset_integer.(y)
    end
   
    nb_cuts = 0
    
    optimize!(model)
    while time() - start < time_limit
        status, v_val, ν_val, obj = sous_pb(value.(y), b, bound_ray=bound_ray)

        if obj > 0 #status == MOI.DUAL_INFEASIBLE
            @constraint(model, -sum(b*adj.*y.*v_val) + demande'*ν_val ≤ 0)
            nb_cuts += 1
            optimize!(model)
        else
            break
        end
        
    end
    if !relaxation
        return round(time()-start, digits=5), round(Int, objective_value(model)), 0, nb_cuts
    end

    #On ajoute maintenant des coupes sur les solutions entières
    set_integer.(y)
    nb_relaxed_cuts = nb_cuts
    nb_integer_cuts = 0

    optimize!(model)
    while time() - start < time_limit
        
        status, v_val, ν_val, obj = sous_pb(value.(y), b, bound_ray=bound_ray)
        if obj > 0 #status == MOI.DUAL_INFEASIBLE
            @constraint(model, -sum(b*adj.*y.*v_val) + demande'*ν_val ≤ 0)
            nb_integer_cuts += 1
            optimize!(model)
        else
            break
        end
    end
    return round(time()-start, digits=5), round(Int, objective_value(model)), nb_relaxed_cuts, nb_integer_cuts
end

