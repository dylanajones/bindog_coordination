#this one is for you, dylan

import math

def euclideanDist(p1, p2):
  if p1 == p2:
    return 0

  res = 0
  for pair in zip(p1, p2):
    res += (pair[1] - pair[0])**2
  return math.sqrt(res)


def manhattanDist(p1, p2):
  #This too
  res = 0
  for pair in zip(p1, p2):
    res += abs(pair[1] - pair[0])

  return res

def safeMax(l):
  if len(l) == 0:
    return 0
  else:
    return max(l)
