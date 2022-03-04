"""
Résoud le problème de séparation:
    max_{b,v} d.b - y'.v
        st  b - v_i ≤ c_i ∀ i
            b ∈ ℝ, v ≥ 0
Retourne 
    - true, b, v si le pb est borné
    - fale, b, v si le pb est non borné (b et v forment un rayon)
"""
function sous_pb(y::Vector{Float64}, d::Int, c::Array{Int}, resDirecte::Bool=true)

    if resDirecte && sum(y) == d #Résolution directe
        b = maximum(c)
        return true, b, b .- c
    end

    n = length(y)
    sous_model = Model(CPLEX.Optimizer)
    set_silent(sous_model)

    @variable(sous_model, b)
    @variable(sous_model, v[1:n]>=0)
    @constraint(sous_model, [i=1:n], b-v[i] <= c[i])
    @objective(sous_model, Max, d*b - y'*v)
    
    optimize!(sous_model)
    println(raw_status(sous_model))
    if termination_status(sous_model) == MOI.INFEASIBLE_OR_UNBOUNDED
        
        println("infeasible_or_unbounded")

        @constraint(sous_model, b+sum(v) <= 2000*d)
        optimize!(sous_model)
        point1b = value(b)
        point1v = value.(v)
        
        @constraint(sous_model, b+sum(v) <= 1000*d)
        optimize!(sous_model)
        point2b = value(b)
        point2v = value.(v)
        
        return false, point1b-point2b, point1v-point2v
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
function benders_solve(n::Int, d::Int, f::Array{Int}, c::Array{Int}, resDirecte::Bool=true)
    
    start = time()
    model = Model(CPLEX.Optimizer)
    MOI.set(model, MOI.NumberOfThreads(), 1)

    @variable(model, y[1:n], Bin)
    @variable(model, w >=0, Int)
    
    @constraint(model, y[1]>=y[2])
    @constraint(model, y[1]>=y[3])
    
    @objective(model, Min, f'*[y; w])
    set_silent(model)
    
    while true
        optimize!(model)
        status, b_val, v_val = sous_pb(value.(y), d, c, resDirecte)

        if status && value(w) >= d*b_val - value.(y)'*v_val - eps
            break
        elseif !status
            @constraint(model, d*b_val - y'*v_val <= 0)
            optimize!(model)
            continue
        end
        
        @constraint(model, w >= d*b_val - y'*v_val)
    end
    optimize!(model)

    return value.(y), value(w), objective_value(model), time()-start
end

function benders_solve_callback(n::Int, d::Int, f::Array{Int}, c::Array{Int}, resDirecte::Bool=true)
    
    start = time()
    model = Model(CPLEX.Optimizer)
    MOI.set(model, MOI.NumberOfThreads(), 1)

    @variable(model, y[1:n], Bin)
    @variable(model, w >=0, Int)
    
    @constraint(model, y[1]>=y[2])
    @constraint(model, y[1]>=y[3])
    
    @objective(model, Min, f'*[y; w])
    set_silent(model)

    function my_cb_function(cb_data::CPLEX.CallbackContext, context_id::Clong)
        
        if context_id == CPX_CALLBACKCONTEXT_CANDIDATE
            CPLEX.load_callback_variable_primal(cb_data, context_id)
        
            y_val = callback_value.(cb_data, y)
        
            bounded, b_val, v_val = sous_pb(y_val,d,c, resDirecte)

            if bounded && callback_value(cb_data,w) >= d*b_val -y_val'*v_val - eps
                println("end")
            elseif !bounded
                con = @build_constraint(d*b_val - (y'*v_val) <= 0)
                println( )
                MOI.submit(model, MOI.LazyConstraint(cb_data), con)
            else # callback_value(cb_data,w) < d*b_val - (y_val'*v_val)
                con = @build_constraint( w >= d*b_val - (y'*v_val) )
                MOI.submit(model, MOI.LazyConstraint(cb_data), con)
            end 
        end
        
    end 
    MOI.set(model, CPLEX.CallbackFunction(), my_cb_function)
    optimize!(model)
    if termination_status(model) == MOI.INFEASIBLE
        println(termination_status(model)) #INFEASIBLE
        return nothing, nothing, +Inf, time()-start
    end
    return value.(y), value(w), objective_value(model), time()-start
end