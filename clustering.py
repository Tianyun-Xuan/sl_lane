from math import *
 
_inf = float('inf')
_ldist_p = 1.1
 
def riemann_dist(Xi, Xj) :
    total = 0.0
    for i in range(len(Xi)) :
        total += (Xi[i] - Xj[i]) ** 2
    dist = sqrt(total)
 
    global _ldist_p
    return pow(_ldist_p, dist) - 1
 
def euclid_dist(Xi, Xj) :
    total = 0.0
    for i in range(len(Xi)) :
        total += (Xi[i] - Xj[i]) ** 2
    return sqrt(total)
 
def matrix(vexlist, dist_func) :
    array = []
    size = len(vexlist)
    for i in range(size) :
        vexi = vexlist[i]
        array.append([])
        for j in range(size) :
            if i == j :
                array[i].append(0)
            else :
                dist = dist_func(vexlist[i], vexlist[j])
                array[i].append(dist)
    return array
                
 
def dijkstra(start, end, matrix) :        
    path = [0]
    size = len(matrix)
 
    dist = [k for k in matrix[start]]
    
    while True :
        if len(path) > 1 :
            idx = path[-1]
            row = matrix[idx]
            val = dist[idx]
            
            for i in range(size) :
                if i in path :
                    continue
                orig = dist[i]
                caculate = val + row[i]
                if orig > caculate :
                    dist[i] = caculate
 
        curdist = []
        for i in range(size) :
            if i not in path :
                curdist.append(dist[i])
                
        cur = dist.index(min(curdist))
        path.append(cur)
 
        if cur == end or len(path) == size - 1 :
            break
 
    return dist[end]
 
def manifold_dist(vexlist, start, end) :
    m = matrix(vexlist, riemann_dist)
    return dijkstra(start, end, m)
 
'''---------------------------main---------------------------'''
if __name__ == '__main__':  
    print (euclid_dist((0, 3), (4, 0)))
    print (riemann_dist((0, 3), (4, 0)))
 
    vexlist = [(0, 0), (1, 1), (2, 2)]
    print (matrix(vexlist, riemann_dist))
 
    m = [[0, _inf, 10, _inf, 30, 100],
         [_inf, 0, 5, _inf, _inf, _inf],
         [10, 5, 0, _inf, 50, _inf, _inf],
         [_inf, _inf, 50, 0, 20, 10],
         [30, _inf, _inf, 20, 0, 60],
         [100, _inf, _inf, 10, 60, 0]
         ]
    print (dijkstra(0, 3, m))
    print (manifold_dist(vexlist, 0, 2))