using CPLEX
using JuMP

include("PL.jl")
include("Benders.jl")


global const eps = 1e-5

function genere_instance(n::Int)
    d = div(n,2)
    f = zeros(Int, n+1)
    c = zeros(Int, n)
    f[1] = 7
    c[1] = 8
    for i in 2:n
        f[i] = rem(f[i-1]*f[1],159)
        c[i] = rem(c[i-1]*c[1],61)
    end
    f[n+1] = 1

    return d, f, c
end

const n = 50000
const d, f, c = genere_instance(n)
# println("n = ", n, "\td = ", d)
# println("f = ", f)
# println("c = ", c)

println(" ----- BENDERS MANUELLE -----")
y, w, obj, time_ = benders_solve(n,d,f,c, false)
println("Time \t Value\n", round(time_,digits=5), "\t ", round(obj,digits=5))

println("\n ----- SANS BENDERS -----")
x, y, obj, time_ = solve_PL(n,d,f,c, false)
println("Time \t Value\n", round(time_,digits=5), "\t ", round(obj,digits=5))

println("\n ----- AUTOMATIC BENDERS -----")
x, y, obj, time_ = solve_PL(n,d,f,c)
println("Time \t Value\n", round(time_,digits=5), "\t ", round(obj,digits=5))

println(" ----- BENDERS MANUELLE SANS PL -----")
y, w, obj, time_ = benders_solve(n,d,f,c)
println("Time \t Value\n", round(time_,digits=5), "\t ", round(obj,digits=5))

println(" ----- BENDERS CALLBACK -----")
y, w, obj, time_ = benders_solve_callback(n,d,f,c)
println("Time \t Value\n", round(time_,digits=5), "\t ", round(obj,digits=5))


