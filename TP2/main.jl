using CPLEX
using JuMP

include("PL.jl")
include("Benders.jl")

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
    return n, d, f, c
end

n, d, f, c = genere_instance(5)
println("n = ", n, "\td = ", d)
println("f = ", f)
println("c = ", c)

println(" ----- BENDERS -----")
y, w, obj = benders_solve(n,d,f,c)
println(y, " of value ", obj, "\n(w=",w,")")


println("\n ----- PL -----")
x, y, obj = solve_PL(n,d,f,c)
println(y, " of value ", obj, "\n(x=",x,")")