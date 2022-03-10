using CPLEX
using JuMP

n=3000

#opt = optimizer_with_attributes(CPLEX.Optimizer, "Presolve" => 0)()
opt = CPLEX.Optimizer()
m = Model(() -> opt)
set_optimizer_attribute(m, "CPX_PARAM_PREIND", 0)
set_optimizer_attribute(m, "CPX_PARAM_THREADS", 1)
#set_silent(m)


@variable(m, x[1:n]>=0)
@variable(m, y>=0)
@objective(m,Max,y)
@constraint(m, y <= sum(x))

optimize!(m)
println(has_values(m))
println(raw_status(m))
println(termination_status(m))
