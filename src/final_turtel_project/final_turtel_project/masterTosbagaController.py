import rclpy
import math
from functools import partial
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist 
from custom_interfaces.msg import Turtle, TurtleArray
from custom_interfaces.srv import CatchTurtle # add all these to dependencies in package.xml

class MasterTurtleNode(Node):
    def __init__(self):
        super().__init__("main_turtle")

        self.declare_parameter("catch_closest_param", True)
        self.isCatchingFirst = self.get_parameter("catch_closest_param").value
        self.pos = None # blank position initially
        self.targetTurtleObj = None
        self.movement_pubber = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.pos_subber = self.create_subscription(Pose, "turtle1/pose", self.subberPosition, 10)
        self.locate_subber = self.create_subscription(TurtleArray, "remaining_turtles", self.subberTargetSetter, 10)
        self.tmrObj = self.create_timer(0.01, self.controllerLoop)
        # ALSO NOTE: We don't have to create a service for controller node too,
        # because we already created one in spawner code. A service can run only
        # at a single instance at a given time

    def subberPosition(self, msg):
        self.pos = msg

    def subberTargetSetter(self, msg):
        if len(msg.turtles) > 0:
            # CONTROL BLOCK FOR CATCHING CLOSEST FIRST
            if self.isCatchingFirst:
                tempClosestTurtle = None
                tempClosestTurtleDistance = None

                for turtul in msg.turtles:
                    distX = turtul.x - self.pos.x
                    distY = turtul.y - self.pos.y
                    distDelta = math.sqrt(distX*distX + distY*distY)
                    if tempClosestTurtle == None or distDelta < tempClosestTurtleDistance:
                        tempClosestTurtle = turtul
                        tempClosestTurtleDistance = distDelta
                self.targetTurtleObj = tempClosestTurtle

            else:
                self.targetTurtleObj = msg.turtles[0]  # assign the first turtle to target

    def catchServiceClient(self, tortName):
        clientObj = self.create_client(CatchTurtle, "catch")
        while not clientObj.wait_for_service(1.0):
            self.get_logger().warn("WAITING FOR BAIT CATCHER SERVICE...")

        req = CatchTurtle.Request()
        req.name = tortName
        fudurObj = clientObj.call_async(req)
        fudurObj.add_done_callback(partial(self.catchServiceServerReturn, tortilname=tortName))

    def catchServiceServerReturn(self, futur, tortilname):
        try:
            respObj = futur.result()
            if not respObj.success:
                self.get_logger().error("Turtle" + str(tortilname) + "could not be caught.")

        except Exception as ex:
            self.get_logger().error("CATCH SERVICE FAILED!! %r" % (ex,))

    def controllerLoop(self):
        if self.pos == None or self.targetTurtleObj == None:  # return void if main turtle hasn't started 
            return

        deltaX = self.targetTurtleObj.x - self.pos.x
        deltaY = self.targetTurtleObj.y - self.pos.y
        dist = math.sqrt(pow(deltaX, 2) + pow(deltaY, 2))  # calculating the distance between main turtle and target

        messag = Twist()

        if dist > 0.5:
            # 2*dist is calculated for P controller, may adjust accordingly
            messag.linear.x = 2*dist
            targetAngle = math.atan2(deltaY, deltaX) # tana = y/x
            deltaAngle = targetAngle - self.pos.theta  # calculating the angle between main turtle and target

            # normalizing the angle if its bigger than 2pi or smaller than -pi
            if deltaAngle > math.pi:
                deltaAngle -= 2*math.pi

            elif deltaAngle < -math.pi:
                deltaAngle += 2*math.pi

            messag.angular.z = 6*deltaAngle
        
        # below case is where the target is reached
        else:
            messag.linear.x = 0.0
            messag.angular.z = 0.0
            self.catchServiceClient(self.targetTurtleObj.name)
            self.targetTurtleObj = None  # this is for preventing repetitive service call

        self.movement_pubber.publish(messag)
        

def main(args=None):
    rclpy.init(args=args)
    node = MasterTurtleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()