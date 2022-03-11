using CPLEX
using JuMP
using Graphs

include("exemple-lecture-graphe.jl")
include("Benders.jl")

"""
INPUT:
    - filename : filename of the instance
"""
function LAN()
    start = time()
    
    ds = dijkstra_shortest_paths(SimpleGraph(adj), 1, allpaths=true)
    paths = enumerate_paths(ds)

    y = zeros(Int, n, n)
    for k in 1:n
        for i in 1:length(paths[k])-1
            y[paths[k][i],paths[k][i+1]] += demande[k]
        end
    end

    return round(time()-start, digits=5), sum(y)
end

function run(time_limit::Float64=300.0)
    println("TIME_LIMIT = ", time_limit)
    instances = ["benders-graphe-hexagone.txt", "benders1.txt", "benders2.txt", "benders3.txt", "benders4.txt"]

    header = raw"""
                    NO RELAXATION    CUTS   |   WITH RELAXATION CUTS        |   NO BOUND_RAY    CUTS        |   DIJKSTRA
                    TIME    VALUE  RLXD/INT |   TIME    VALUE  RLXD/INT     |   TIME    VALUE  RLXD/INT     |   TIME    VALUE
    ========================================================================================================================="""

    for b in [1,3]
        println("\n=========================================================================================================================")
        println("b = ", b)
        println(header)
        for instance in instances
            readGraph(instance)
            if instance == "benders-graphe-hexagone.txt"
                print("hexagone")
            else
                print(instance)
            end
      
            time_, obj, rlxd, int = benders_solve(b, time_limit = time_limit, relaxation = false)
            print("\t",join((time_, obj, string(rlxd)*"/"*string(int)), "\t"), "\t|")

            time_, obj, rlxd, int = benders_solve(b, time_limit = time_limit)
            print("   ", join((time_, obj, string(rlxd)*"/"*string(int)), "\t"), "\t|")

            time_, obj, rlxd, int = benders_solve(b, time_limit = time_limit, bound_ray = false)
            print("   ", join((time_, obj, string(rlxd)*"/"*string(int)), "\t"), "\t|" )

            if b == 1
                println("   ",join(LAN(), "\t   "))
            else
                println()
            end
        end
    end
end

run()