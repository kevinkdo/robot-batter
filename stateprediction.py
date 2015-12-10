import numpy as np
import math
import sys
from scipy.optimize import curve_fit
import matplotlib.pyplot as plot

MAX_HISTORY = 300

class Predictor:
    def __init__(self):
        self.objects = dict()
        self.clean = dict()
        self.parameter_cache = dict()

    '''t: time
       o: ObjectStateEstimate
       no return'''
    def addPoint(self, t, o):
        self.clean[o.name] = False
        if not o.name in self.objects:
            self.objects[o.name] = []
            self.parameter_cache[o.name] = [1, .6, 1, 0]
        self.objects[o.name].append((t, o.meanPosition(), o.meanVelocity()))
        if len(self.objects[o.name]) > MAX_HISTORY:
            self.objects[o.name].pop(0)

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
        def f(x, a, b, c, d):
            return a * np.sin(b * x + c) + d

        if not name in self.objects:
            return None
        tuples = self.objects[name]
        tlist = np.array(map(lambda x: x[0], tuples))
        ylist = np.array(map(lambda x: x[1][1], tuples))
        try:
            if not self.clean[name]:
                self.parameter_cache[name], _ = curve_fit(f, tlist, ylist, p0=self.parameter_cache[name])
                self.clean[name] = True
        except RuntimeError as e:
            print "Warning: ", e
            sys.stdout.flush()
            return None

        return (0,
                f(t, self.parameter_cache[name][0],
                     self.parameter_cache[name][1],
                     self.parameter_cache[name][2],
                     self.parameter_cache[name][3]),
                0)
