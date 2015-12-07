from klampt import *
from klampt import gldraw
from stateestimation import *
from stateprediction import *
from OpenGL.GL import *
import sys
import time
import matplotlib.pyplot as plot

LOG_BALL_PATH = False
NUM_TRIALS = 10
TRIAL_LENGTH = 150

PREP_RECOVER = np.array([0.0, 2.0, -.98, .8, math.pi/2, 0.0, 0.0])
POST_RECOVER = np.array([0.0, 1.0, -.98, 1.32, math.pi/2, 0.0, 0.0])

STROKES = np.array([[0.0, 1.0, -1.08, 1.187, math.pi/2, 0.0, 0.0],#PREP_L
                    [0.0, 2.0, -1.08, 1.187, math.pi/2, 0.0, 0.0],#POST_L
                    [0.0, 1.0, -0.98 , 1.32, math.pi/2, 0.0, 0.0],#PREP_C
                    [0.0, 2.0, -0.98 , 1.32, math.pi/2, 0.0, 0.0],#POST_C
                    [0.0, 1.0, -0.98 , 1.40, 1.67     , 0.0, 1.6],#PREP_R
                    [0.0, 2.0, -0.98 , 1.40, 1.67     , 0.0, 1.6]])#POST_R

TRAVELTIMES = [[1.986, 2.293, 2.606],
               [1.973, 2.273, 2.573],
               [2.053, 2.366, 2.680]]

INTERSECTIONS = [[+.30, +.45, +.60],
                 [-.10, -.05, +0.0],
                 [-.60, -.61, -.63]]

BALL = (1, 0, 0, 1)
GOALIES = [(1, 0.5, 0, 1), (1, 1, 0, 1), (0.5, 1, 0, 1)]

SETUP_TIMES = [0.23, -.07, 0.43]
STILL_LIMIT = .01

class MyController:
    """Attributes:
    - world: the WorldModel instance used for planning.
    - objectStateEstimator: a StateEstimator instance, which you may set up.
    - state: a string indicating the state of the state machine. TODO:
      decide what states you want in your state machine and how you want
      them to be named.  By default, this will go into the 'waiting' state
      on startup, and will go into the 'user' state when 'u' is pressed.

    By default, if 'u' is pressed, then this switches to 'user' mode,
    and uses the keys.
     - 1,2,3,4,5,6 to increase the robot's joint angles and
     - q,w,e,r,t,y to decrease the robot's joint angles.
    If 'u' is pressed again, it switches back to 'waiting' mode
    """
    def __init__(self,world,robotController):
        self.world = world
        self.objectStateEstimator = None
        self.ballPredictor = None
        self.goaliePredictor = None
        self.state = None
        self.robotController = robotController
        self.reset(robotController)
        
    def reset(self,robotController):
        """Called on initialization, and when the simulator is reset."""
        self.objectStateEstimator = MyObjectStateEstimator()
        self.ballPredictor = LinearPredictor()
        self.goaliePredictor = YSinePredictor()
        self.objectEstimates = None
        self.state = 'precycle0'
        self.qdes = robotController.getCommandedConfig()
        self.t = 0
        self.substate = 0

        if LOG_BALL_PATH:
            self.xplot = [[] for i in range(NUM_TRIALS)]
            self.yplot = [[] for i in range(NUM_TRIALS)]
            self.trial = -1

    '''Returns whether q1 is close to q2'''
    def close(self, q1, q2):
        if len(q1) != len(q2):
            print "self.close requires arguments of same length"
            sys.stdout.flush()
            sys.exit()
            return False
        sum = 0
        for i in range(len(q1)):
            sum += abs(q1[i] - q2[i])

        return sum < 1e-1

    '''Returns whether the ball is waiting to be struck'''
    def ballWaiting(self, ballState):
        p = ballState.meanPosition()
        v = ballState.meanVelocity()
        return (-1.5 < p[0] and p[0] < -0.5 and
                -1.0 < p[1] and p[1] < +0.0 and
                .072 < p[2] and p[2] < .082 and
                -0.5 < v[0] and v[0] < +0.5 and
                -0.5 < v[1] and v[1] < +0.5 and
                -.01 < v[2] and v[2] < +.01)

    '''Returns index of best stroke (0, 1, 2) or -1 to indicate no valid path'''
    def best_stroke(self):
        answer = -1
        best_clearance = .6
        for i in range(len(TRAVELTIMES)):
            clearance = 50
            for j in range(len(GOALIES)):
                yhat = self.goaliePredictor.predict(self.t + TRAVELTIMES[i][j] + SETUP_TIMES[i], GOALIES[j])[1]
                sys.stdout.flush()
                clearance = min(clearance, abs(yhat - INTERSECTIONS[i][j]))
            if clearance > best_clearance:
                answer = i
                best_clearance = clearance
        return answer

    def myPlayerLogic(self,
                      dt,
                      sensorReadings,
                      objectStateEstimate,
                      robotController):
        """
        TODO: fill this out to updates the robot's low level controller
        in response to a new time step.  This is allowed to set any
        attributes of MyController that you wish, such as self.state.
        
        Arguments:
        - dt: the simulation time elapsed since the last call
        - sensorReadings: the sensor readings given on the current time step.
          this will be a dictionary mapping sensor names to sensor data.
          The name "blobdetector" indicates a sensor reading coming from the
          blob detector.  The name "omniscient" indicates a sensor reading
          coming from the omniscient object sensor.  You will not need to
          use raw sensor data directly, if you have a working state estimator.
        - objectStateEstimate: a MultiObjectStateEstimate class (see
          stateestimation.py) produced by the state estimator.
        - robotController: a SimRobotController instance giving access
          to the robot's low-level controller.  You can call any of the
          methods.  At the end of this call, you can either compute some
          PID command via robotController.setPIDCommand(), or compute a
          trajectory to execute via robotController.set/addMilestone().
          (if you are into masochism you can use robotController.setTorque())
        """
        self.t += dt
        qcmd = robotController.getCommandedConfig()
        vcmd = robotController.getCommandedVelocity()
        qsns = robotController.getSensedConfig()
        vsns = robotController.getSensedVelocity()

        if self.t > dt:
            self.ballPredictor.addPoint(self.t, objectStateEstimate.get(BALL))
            for goalie in GOALIES:
                self.goaliePredictor.addPoint(self.t, objectStateEstimate.get(goalie))

        ''' Confusing function that plans path on first call, maintains state between calls
            and keeps on going on the pre-planned path'''
        def moveAndGoToState(qdes, next_state, frames=1):
            if self.substate == 0:
                self.qdes = qdes
                self.start = qsns
            if self.substate < frames:
                self.substate += 1
            next_goal = self.start + (1.0 * self.substate / frames) * (self.qdes - self.start)
            robotController.setPIDCommand(next_goal, [0.0]*7)
            if self.close(qsns, self.qdes) and np.linalg.norm(vsns) < STILL_LIMIT:
                self.state = next_state
                self.substate = 0

        if self.state == 'precycle0':
            moveAndGoToState(POST_RECOVER, 'waiting', 22)
        if self.state == 'post_recover':
            moveAndGoToState(POST_RECOVER, 'waiting', 10)
        if self.state == 'waiting':
            if self.ballWaiting(objectStateEstimate.get(BALL)):
                best_stroke = self.best_stroke()
                if best_stroke != -1:
                    self.state = 'pre_stroke' + str(best_stroke)
        if self.state[:10] == 'pre_stroke':
            stroke_index = int(self.state[10])
            moveAndGoToState(STROKES[2 * stroke_index], 'stroke' + str(stroke_index), 1)
        if self.state[:6] == 'stroke':
            if LOG_BALL_PATH and self.substate == 0:
                self.trial += 1

            stroke_index = int(self.state[6])
            moveAndGoToState(STROKES[2 * stroke_index + 1], 'pre_recover', 22)
        if self.state == 'pre_recover':
            moveAndGoToState(PREP_RECOVER, 'recover', 10)
        if self.state == 'recover':
            moveAndGoToState(POST_RECOVER, 'post_recover', 25)
        if self.state == 'user':
            robotController.setPIDCommand(self.qdes,[0.0]*7)

        if LOG_BALL_PATH:
            if self.trial < NUM_TRIALS:
                if len(self.xplot[self.trial]) < TRIAL_LENGTH:
                    ballstate = objectStateEstimate.get(BALL)
                    self.xplot[self.trial].append(ballstate.meanPosition()[0])
                    self.yplot[self.trial].append(ballstate.meanPosition()[1])
            else:
                print np.array(self.yplot).shape
                np.save('x.npy', np.array(self.xplot))
                np.save('y.npy', np.array(self.yplot))
                sys.exit()

        #print self.t
        #print self.state
        sys.stdout.flush()
        return
        
    def loop(self,dt,robotController,sensorReadings):
        """Called every control loop (every dt seconds).
        Input:
        - dt: the simulation time elapsed since the last call
        - robotController: a SimRobotController instance. Use this to get
          sensor data, like the commanded and sensed configurations.
        - sensorReadings: a dictionary mapping sensor names to sensor data.
          The name "blobdetector" indicates a sensor reading coming from the
          blob detector.  The name "omniscient" indicates a sensor reading coming
          from the omniscient object sensor.
        Output: None.  However, you should produce a command sent to
          robotController, e.g., robotController.setPIDCommand(qdesired).

        """
        multiObjectStateEstimate = None
        if self.objectStateEstimator and 'blobdetector' in sensorReadings:
            multiObjectStateEstimate = self.objectStateEstimator.update(sensorReadings['blobdetector'])
            self.objectEstimates = multiObjectStateEstimate
        if 'omniscient' in sensorReadings:
            omniscientObjectState = OmniscientStateEstimator().update(sensorReadings['omniscient'])
            #TODO: Comment out the following line when you are ready to test your state estimator
            multiObjectStateEstimate  = omniscientObjectState

        self.myPlayerLogic(dt,
                           sensorReadings,multiObjectStateEstimate,
                           robotController)
        return

    def keypress(self,key):
        """If you want to implement some interactivity while debugging,
        you can do it here. By default, it uses 1,2,3,4,5,6 to increase
        the robot's joint angles and q,w,e,r,t,y to to decrease them.
        """
        if key == 'u':
            if self.state == 'user':
                print "Switching out of user mode..."
                self.state = 'waiting'
            else:
                print "Switching into user mode..."
                self.state = 'user'
                self.qdes = self.robotController.getCommandedConfig()
        if self.state == 'user':
            #note: joint 0 is a dummy joint
            upkeys = {'1':1,'2':2,'3':3,'4':4,'5':5,'6':6}
            downkeys = {'\'':1,',':2,'.':3,'p':4,'y':5,'f':6}
            if key in upkeys:
                self.qdes[upkeys[key]] += 0.1
            elif key in downkeys:
                self.qdes[downkeys[key]] -= 0.1
            print self.qdes
            sys.stdout.flush()
    
    def drawGL(self):
        """This gets called every time an OpenGL rendering loop is called.
        TODO: You may consider visually debugging some of your code here.

        For example, to draw the robot at a given configuration q, you can call:
          self.world.robot(0).setConfig(q)
          self.world.robot(0).drawGL()

        To draw a point with size s, color (r,g,b), and world position (x,y,z)
        you can call:
          glDisable(GL_LIGHTING)
          glColor3f(r,g,b)
          glPointSize(s)
          gldraw.point([x,y,z])

        The current code draws gravity-inflenced arcs leading from all the
        object position / velocity estimates from your state estimator.  Event C
        folks should set gravity=0 in the following code.
        """
        if self.objectEstimates:
            for o in self.objectEstimates.objects:
                glDisable(GL_LIGHTING)
                glColor3f(o.name[0],o.name[1],o.name[2])
                #draw a point
                glPointSize(5.0)
                gldraw.point([o.x[0],o.x[1],o.x[2]])
                #draw an arc
                glBegin(GL_LINE_STRIP)
                x = [o.x[0],o.x[1],o.x[2]]
                v = [o.x[3],o.x[4],o.x[5]]
                #TODO: are you doing event C? If so, you should
                #set gravity=0 to get more useful visual feedback
                #about your state estimates.
                gravity = 0
                #gravity = 9.8
                for i in range(20):
                    t = i*0.05
                    glVertex3f(*vectorops.sub(vectorops.madd(x,v,t),[0,0,0.5*gravity*t*t]))
                glEnd()
