from klampt import *
from klampt.glprogram import *
from klampt import gldraw
from controller import *
from sensor import *
import time
import random
import traceback

#TODO: change this to False when you are ready to test your state estimator
omniscientSensor = True

#TODO: change this to random.seed() when you are ready to stress-test your
#controller
random.seed(12345)
#random.seed()

class EventA:
    """This class does the event logic"""
    def __init__(self,sim,difficulty=None):
        if difficulty == None:
            difficulty = 'easy'
        
        self.difficulty = difficulty
        self.score = 140
        self.forfeit = False
        
        self.lastpenaltytime = 0
        self.tcontroller = 0

        self.ball = 0
        if difficulty == 'easy':
            self.balltimes = [2+i*2 for i in range(14)]
        elif difficulty == 'medium':
            self.balltimes = [1+i*1.5 for i in range(14)]
        elif difficulty == 'hard':
            self.balltimes = [1+i*1 for i in range(14)]
        else:
            raise ValueError("Invalid difficulty value "+str(difficulty))
        self.activeBalls = [False]*14
        self.endtime = self.balltimes[-1] + 3.0

        #activate collision feedback
        robot = sim.world.robot(0)
        for i in range(robot.numLinks()):
            for j in range(sim.world.numTerrains()):
                sim.enableContactFeedback(robot.getLink(i).getID(),sim.world.terrain(j).getID())
        
    def mark_controller_time(self,tcontroller):
        self.tcontroller += tcontroller
        
    def update(self,dt,sim):
        if self.forfeit: return
        t = sim.getTime()
        if t > self.lastpenaltytime + 1.0:
            if self.tcontroller > 5:
                print "Event supervisor: Took too long to compute controls"
                print "  Result: forfeit"
                self.score -= 5
                self.forfeit = True
            elif self.tcontroller > 1:
                print "Event supervisor: Took too long to compute controls"
                print "  Time",self.tcontroller,"over the last 1s"
                print "  Penalty: 1"
                self.score -= 1
            self.tcontroller = 0
            self.lastpenaltytime = t
        #check joint limits, velocity limits, and torque limits
        qrobot = sim.getActualConfig(0)
        vrobot = sim.getActualVelocity(0)
        trobot = sim.getActualTorques(0)
        qmin,qmax = sim.world.robot(0).getJointLimits()
        vmax = sim.world.robot(0).getVelocityLimits()
        tmax = sim.world.robot(0).getTorqueLimits()
        for i in range(7):
            if qrobot[i] < qmin[i] or qrobot[i] > qmax[i]:
                print "Event supervisor: Out of joint limits"
                self.score -= dt*10
                break
        for i in range(1,7):
            if abs(vrobot[i]) > vmax[i]:
                print "Event supervisor: Out of velocity limits"
                print vrobot,vmax
                self.score -= dt*10
                break
        for i in range(6):
            if abs(trobot[i]) > tmax[i+1]:
                print "Event supervisor: Out of torque limits"
                print trobot,tmax
                self.score -= dt*10
                break
        #check collisions between robot and terrain
        if self.inContact(sim):
            print "Event supervisor: in contact with terrain"
            self.score -= dt*30
            
        #do ball kicking logic
        self.doGameLogic(sim)
        return
    
    def doGameLogic(self,sim):
        t = sim.getTime()
        goalcenter = (-2.7,0,0.5)
        goaldims = (0.2,2.2,1.0)
        goalmin = vectorops.madd(goalcenter,goaldims,-0.5)
        goalmax = vectorops.madd(goalcenter,goaldims,0.5)
        if self.ball < len(self.balltimes) and t > self.balltimes[self.ball]:
            obj = sim.world.rigidObject(self.ball)
            ballbody = sim.getBody(obj)
            Tb = ballbody.getTransform()
            ballrad = 0.1
            cmin= vectorops.add(goalmin,[ballrad]*3)
            cmax= vectorops.add(goalmax,[-ballrad]*3)
            tgty = random.uniform(cmin[1],cmax[1])
            #don't shoot in middle
            while abs(tgty) < 0.2:
                tgty = random.uniform(cmin[1],cmax[1])
            tgtz = random.uniform(cmin[2],cmax[2])
            target = (goalcenter[0],
                      tgty,
                      tgtz)
            vel = vectorops.sub(target,Tb[1])
            t = 1
            if self.difficulty == "easy":
                t = 1
            elif self.difficulty == "medium":
                t = random.uniform(0.8,1.2)
            elif self.difficulty == "hard":
                t = random.uniform(0.5,1.5)
            vel = vectorops.mul(vel,t)
            vel[2] = random.uniform(3.0/t,5.4/t)
            print "Event supervisor: Kicking ball",self.ball,"target",target,"velocity",vel
            self.activeBalls[self.ball] = True
            ballbody.setVelocity([0,0,0],vel)
            ballbody.enable(True)
            
            self.ball += 1
        
        #determine if any scored
        for i in range(self.ball):
            if self.activeBalls[i]:
                obj = sim.world.rigidObject(i)
                Tb = sim.getBody(obj).getTransform()
                Rb,tb = Tb
                if all(tb[i] > goalmin[i] and tb[i] < goalmax[i] for i in range(3)):
                    print "Event supervisor: Ball",i,"scored, deducting 10 points"
                    self.score -= 10
                    self.activeBalls[i] = False
                if tb[2] < -2.0:
                    #fallen
                    sim.getBody(obj).enable(False)
                    self.activeBalls[i] = False
        return
    
    def inContact(self,sim):
        """Returns true if the robot touches the environment"""
        robot = sim.world.robot(0)
        for i in range(robot.numLinks()):
            for j in range(sim.world.numTerrains()):
                if sim.hadContact(robot.getLink(i).getID(),sim.world.terrain(j).getID()):
                    return True
        return False

class GLTest(GLRealtimeProgram):
    def __init__(self,simWorld,planningWorld,difficulty=None):
        GLRealtimeProgram.__init__(self,"Final, group A")
        self.simWorld = simWorld
        self.planningWorld = planningWorld
        self.sim = Simulator(self.simWorld)
        self.event = EventA(self.sim,difficulty)
        #set up sensors
        self.sensors = dict()
        global omniscientSensor
        if omniscientSensor:
            self.sensors['omniscient'] = OmniscientObjectSensor()
        self.sensors['blobdetector'] = CameraColorDetectorSensor()
        cameraRot = [0,-1,0,0,0,-1,1,0,0]
        #at goal post, pointing a bit up and to the left
        Tsensor = (so3.mul(so3.rotation([0,0,1],0.20),so3.mul(so3.rotation([0,-1,0],0.25),cameraRot)),[-2.55,-1.1,0.25])
        self.sensors['blobdetector'].Tsensor = Tsensor
        self.controller = MyController(self.planningWorld,self.sim.getController(0))
        
        #set up camera to get a better vantage point
        self.camera.dist = 12
        self.camera.tgt[2] = -1
        
        self.sim.simulate(0)
        self.simulate = True
        self.continueSimAfterForfeit = False

    def display(self):
        self.sim.updateWorld()
        self.simWorld.drawGL()
        self.controller.drawGL()
        glEnable(GL_LIGHTING)
        gldraw.xform_widget(self.sensors['blobdetector'].Tsensor,0.1,0.01,fancy=True)

    def display_screen(self):
        glDisable(GL_LIGHTING)
        glColor3f(0,0,0)
        glRasterPos2i(20,30)
        gldraw.glutBitmapString(GLUT_BITMAP_HELVETICA_12,"Time: "+str(self.sim.getTime()))
        glRasterPos2i(20,50)
        gldraw.glutBitmapString(GLUT_BITMAP_HELVETICA_12,"Score: "+str(self.event.score))

    def control_loop(self):
        readings = dict()
        for n,s in self.sensors.iteritems():
            readings[n] = s.emulate(self.sim)
        try:
            self.controller.loop(self.dt,self.sim.getController(0),readings)
        except Exception as e:
            print "Exception called during controller.loop:"
            traceback.print_exc()
            exit(0)

    def step(self):
        t0 = time.time()
        self.control_loop()
        tcontroller = time.time()-t0
        self.event.mark_controller_time(tcontroller)
        
        self.sim.simulate(self.dt)
        self.event.update(self.dt,self.sim)
        if self.event.forfeit and not self.continueSimAfterForfeit:
            if raw_input("Quit? (y/n) > ")=='y':
                exit(0)
            else:
                self.continueSimAfterForfeit = True
        if self.sim.getTime() >= self.event.endtime:
            print "Final score:",self.event.score
        self.refresh()
    
    def keyboardfunc(self,c,x,y):
        if c=='s':
            self.simulate = not self.simulate
        elif c==' ':
            self.step()
            self.refresh()
        else:
            self.controller.keypress(c)
            self.refresh()

    def idle(self):
        if self.simulate:
            self.step()


if __name__ == "__main__":
    global objectColors
    world = WorldModel()
    world2 = WorldModel()
    fn = "data/finalA.xml"
    res = world.readFile(fn)
    if not res:
        raise RuntimeError("Unable to load world "+fn)
    res = world2.readFile(fn)
    for i in range(world.numRigidObjects()):
        world.rigidObject(i).appearance().setColor(*objectColors[i%len(objectColors)])
        world2.rigidObject(i).appearance().setColor(*objectColors[i%len(objectColors)])
    difficulty = None
    if len(sys.argv) >= 2:
        difficulty = sys.argv[1]
    GLTest(world,world2,difficulty).run()
