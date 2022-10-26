from dataclasses import dataclass
import queue
from queue import PriorityQueue

# write a hasing function
def getIndex(x,y,xsize,ysize):
    ind= (y-1)*xsize + x
    
    return ind


if __name__=="main":
    @dataclass
    class node:
        index: int
        g:int 
        parent: int

    startX=12
    startY=12

    goalX=122
    goalY=212

    dX = [-1, -1, -1,  0,  0,  1, 1, 1]
    dY = [-1,  0,  1, -1,  1, -1, 0, 1]


    IndStart=getIndex(startX,startY)

    node(IndStart,0,0)

    # open["ind"]=queue.insert(map(0,IndStart))

    while (len(open)!=0):

        open.pop(open)
        remove s with the smallest [f(s) = g(s)+h(s)] from OPEN;
        insert s into CLOSED;
        for every successor s’ of s such that s’ not in CLOSED
        if g(s’) > g(s) + c(s,s’) g(s’) = g(s) + c(s,s’); insert s’ into OPEN;
