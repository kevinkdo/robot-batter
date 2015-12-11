from sensor import *
import math
import numpy as np
from kalmanfilter import *

class ObjectStateEstimate:
    """Attributes:
    - name: an identifier of the object
    - x: mean state (position / velocity) estimate, a 6-D vector
    - cov: state (position / velocity) covariance, a 6x6 numpy array
    """
    def __init__(self,name,x,cov=0):
        self.name = name
        self.x = x
        self.cov = cov
        if isinstance(cov,(int,float)):
            self.cov = np.eye(6)*cov
    def meanPosition(self):
        return self.x[0:3]
    def meanVelocity(self):
        return self.x[3:6]

class MultiObjectStateEstimate:
    """A list of ObjectStateEstimates.
    
    Attributes:
        - objects: a list of ObjectStateEstimates, corresponding to all
          of the objects currently tracked by the state estimator.
    """
    def __init__(self,objectEstimates):
        self.objects = objectEstimates[:]
        
    def get(self,name):
        """Retrieves an object's state estimate by name"""
        for o in self.objects:
            if o.name==name:
                return o
        return None

class OmniscientStateEstimator:
    """A hack state estimator that gives perfect state information from
    OmniscientObjectOutput readings."""
    def __init__(self):
        self.reset()
        return
    def reset(self):
        pass
    def update(self,o):
        """Produces an updated MultiObjectStateEstimate given an OmniscientObjectOutput
        sensor reading."""
        assert isinstance(o,OmniscientObjectOutput),"OmniscientStateEstimator only works with an omniscient sensor"
        estimates = [ObjectStateEstimate(n,p+v) for n,p,v in zip(o.names,o.positions,o.velocities)]
        return MultiObjectStateEstimate(estimates)

class MyObjectStateEstimator:
    """Your own state estimator that will provide a state estimate given
    CameraColorDetectorOutput readings."""
    def __init__(self):
        self.reset()
        #TODO: fill this in with your own camera model
        self.Tsensor = None
        self.fov = 90
        self.w,self.h = 320,240
        self.dmax = 5
        self.dt = 0.02
        return
    def reset(self):
        pass
    def update(self,o):
        """Produces an updated MultiObjectStateEstimate given a CameraColorDetectorOutput
        sensor reading."""
        assert isinstance(o,CameraColorDetectorOutput),"MyObjectStateEstimator only works with CameraColorDetectorOutput"

        estimates = []
        goalie_name = lambda x: (x[0], x[1], x[2], 1)
        goalie_x = {(1, 0.5, 0): 2.0, (1, 1, 0): 2.5, (0.5, 1, 0): 3.0}
        for blob in o.blobs:
            if blob.color != (1, 0, 0):
                alpha = (goalie_x[blob.color] + 1.5) / 160.0
                p_y = -.5 + alpha * (160 - blob.x)
                p = [goalie_x[blob.color], p_y, 0]
            else:
                if blob.x > 150 and blob.y > 165 and blob.x < 170 and blob.y < 185 and blob.w > 45 and blob.w < 54 and blob.h > 43 and blob.h < 53:
                    p = [-1, -.5, .077]
                else:
                    p = [-2, -.5, .077]

            v = [0, 0, 0]
            estimate = ObjectStateEstimate(goalie_name(blob.color), p+v)
            estimates.append(estimate)
        return MultiObjectStateEstimate(estimates)
