import numpy as np
import math
from scipy.optimize import curve_fit

class Predictor:
    def __init__(self):
        self.objects = dict()

    '''t: time
       o: ObjectStateEstimate
       no return'''
    def addPoint(self, t, o):
        if not o.name in self.objects:
            self.objects[o.name] = []
        self.objects[o.name].append((t, o.meanPosition(), o.meanVelocity()))

    '''name: string name of object
       returns (t list, position list, velocity list)'''
    def getHistory(self, name):
        if not name in self.objects:
            return None

        tlist = []
        positionlist = []
        velocitylist = []
        tuples = self.objects[name]
        for tu in tuples:
            tlist.append(tu[0])
            positionlist.append(tu[1])
            velocitylist.append(tu[2])

        return (tlist, positionlist, velocitylist)

class LinearPredictor(Predictor):
    '''t: time to predict at
       name: string name of object
       returns (x, y, z) of predicted state'''
    def predict(self, t, name):
        if not name in self.objects:
            return None
        tuples = self.objects[name]
        lastT, lastPosition, lastVelocity = tuples[-1]
        deltaT = t - lastT
        return (lastPosition[0] + deltaT * lastVelocity[0],
                lastPosition[1] + deltaT * lastVelocity[1],
                lastPosition[2] + deltaT * lastVelocity[2])

class YSinePredictor(Predictor):
    '''t: time to predict at
       name: string name of object
       returns (x, y, z) of predicted state'''
    def predict(self, t, name):
        def f(x, a, b, c):
            return a * np.sin(b * x + c)

        if not name in self.objects:
            return None
        tuples = self.objects[name]
        tlist = map(lambda x: x[0], tuples)
        ylist = map(lambda x: x[1][1], tuples)
        popt, pcov = curve_fit(f, tlist, ylist)

        return (0, f(t, popt[0], popt[1], popt[2]), 0)
