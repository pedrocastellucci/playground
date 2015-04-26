'''
This is an implementation of the Segmented Least Square algorithm
as described in the book "Algorithm design". Chapter 6 - Dynamic Programming.
Book author's: Jon Kleinberg, Eva Tardos.

I do not ensure this works properly.
'''

import numpy as np
import matplotlib.pyplot as plt
from math import sin


def getLeastSquareError(points):

    if len(points) <= 2:
        return 0.0

    X = []
    Y = []
    for p in points:
        X.append(p[0])
        Y.append(p[1])

    ret = np.polyfit(X, Y, deg=1, full=True)
    return ret[1][0]


def findSegments(e, M, n, cost):
    if n == 0:
        return []

    minIndex = np.argmin([e[i, n-1] + cost + M[i-1] for i in range(n)])

    return [range(minIndex, n)] + findSegments(e, M, minIndex, cost)


def segmentedLeastSquares(points, cost):
    '''
    Algorithm Design - Chapter 6 - Dynamic Programming.
    Jon Kleinberg, Eva Tardos
    @param points - a list [(x0, y0), (x1, y1), ..., (xn, yn)] with the points
    that will be approximated by linear segments. It is assumed that
    x0 < x1 < ... < xn.
    @param cost - cost of adding a new segment to the approximation.
    @return - a list [[a00, a01, ..., a0r], [a10, a11, ..., a0p], ...] in which
    each element of the list is itself a list with the indices of the items in
    'points' list that are approximated by one linear segment.
    '''

    n = len(points)
    e = {}

    M = {}
    M[-1] = 0

    for j in range(n):
        for i in range(j+1):
            e[i, j] = getLeastSquareError(points[i:j+1])

    for j in range(n):
        minCandidates = []
        for i in range(j+1):
            minCandidates.append(e[i, j] + cost + M[i-1])

            if len(minCandidates) > 0:
                M[j] = min(minCandidates)

    lines = findSegments(e, M, n, cost)
    lines.sort(key=lambda line: line[0])

    return lines


# Here's a small example:
xAxis = np.arange(0, 2*3.1416, 0.1)
points = [(x, sin(x))
          for i, x in enumerate(xAxis)]

lines = segmentedLeastSquares(points, 1)

linesToConnect = [[lines[i-1][-1], lines[i][0]] for i in range(1, len(lines))]

for line in lines + linesToConnect:
    x = [xAxis[line[0]], xAxis[line[-1]]]
    y = [points[line[0]][1], points[line[-1]][1]]
    print x
    plt.plot(x, y, linewidth=2, color='r')

plt.scatter([z[0] for z in points], [z[1] for z in points])
plt.tick_params(axis='both', which='major', labelsize=16)
plt.show()
