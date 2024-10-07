import random, math, rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from custom_interfaces.msg import Turtle, TurtleArray
from custom_interfaces.srv import CatchTurtle
from functools import partial

class Spawner(Node):
    def __init__(self):
        super().__init__("spawner_node")
        self.declare_parameter("spawn_freq_param", 2.5)
        self.declare_parameter("turtlename_prefix_param", "BaitTurtle")
        self.remainingTurtlePubber = self.create_publisher(TurtleArray, "remaining_turtles", 10)
        # PUBLSIHING IS DONE WHEN SERVICE CALLED!! 
        self.spawnFreq = self.get_parameter("spawn_freq_param").value
        self.spawnTmrObject = self.create_timer(self.spawnFreq / 1.0, self.turtleSpawner)
        self.namePrefix = self.get_parameter("turtlename_prefix_param").value
        
        self.turtleCounter = 1
        self.liveBaits = []
        self.catcherSRVObject = self.create_service(CatchTurtle, "catch", self.killServiceBuild)
        
    def killServiceBuild(self, request, response):
        self.killServiceClient(request.name)
        response.success = True
        return response

    def remainingTurtlePubberCallback(self):
        tempMsg = TurtleArray()
        tempMsg.turtles = self.liveBaits
        self.remainingTurtlePubber.publish(tempMsg)

    def turtleSpawner(self):
        nameTemp = self.namePrefix + str(self.turtleCounter)
        self.turtleCounter += 1
        xRand = random.uniform(0.0, 11.0)
        yRand = random.uniform(0.0, 11.0)
        angleRand = math.pi*(random.uniform(0.0, 2.0))
        self.spawnServiceClient(nameTemp, xRand, yRand, angleRand)

    def killServiceClient(self, turtleToKill):
        clientObj = self.create_client(Kill, "kill")
        while not clientObj.wait_for_service(1.0):
            self.get_logger().warn("WAITING FOR BAIT KILLER SERVICE...")
        
        req = Kill.Request()
        req.name = turtleToKill
        future = clientObj.call_async(req)
        future.add_done_callback(partial(self.killServiceServerReturn, tortelname=turtleToKill))

    def killServiceServerReturn(self, futur, tortelname):
        try:
            futur.result()
            # /kill service has an empty response, but calling it anyway to deal with exceptions
            for (i, TORT) in enumerate(self.liveBaits):
                if TORT.name == tortelname:
                    del self.liveBaits[i]
                    self.remainingTurtlePubberCallback()
                    break
        except Exception as e:
            self.get_logger().error("KILL SERVICE FAILED! %r" %(e,))

    def spawnServiceClient(self, turtleName, xParam, yParam, thetaParam):
        # BELOW SERVICE NAME MUST BE SAME AS ORIGINAL TURTLESIM SERVICE "SPAWN" !!!
        clientObj = self.create_client(Spawn, "spawn")
        while not clientObj.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server become available...")

        reqObj = Spawn.Request()
        reqObj.name = turtleName
        reqObj.x = xParam
        reqObj.y = yParam
        reqObj.theta = thetaParam

        futureObj = clientObj.call_async(reqObj)
        futureObj.add_done_callback(
            partial(self.spawnServiceServerReturn, tname=turtleName, xPassed=xParam, yPassed=xParam, thetaPassed=thetaParam))
            # FOR USE OF "PARTIAL"!!!
            # syntax for args: (assume partial is used under function "func1")
            # partial(self.calledFunc, calledFunc.param1 = func1.param1, calledFunc.param2 = func1.param2 .... and so)

    def spawnServiceServerReturn(self, futur, tname, xPassed, yPassed, thetaPassed):
        try:
            respObj = futur.result()
            if respObj.name != "":
                self.get_logger().info(respObj.name +" spawned.")
                spwnedTurtle = Turtle()
                spwnedTurtle.name = respObj.name
                spwnedTurtle.x = xPassed
                spwnedTurtle.y = yPassed
                spwnedTurtle.theta = thetaPassed
                self.liveBaits.append(spwnedTurtle)
                self.remainingTurtlePubberCallback()

        except Exception as e:
            self.get_logger().error(" SPAWN SERVICE FAILED! %r" %(e,))


def main(args=None):
    rclpy.init(args=args)
    node = Spawner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()