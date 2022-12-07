import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist, Point, Pose, Quaternion
from nav_msgs.msg import Odometry
from math import pow, sqrt, atan2, sin, cos, pi
from time import perf_counter


MAX_LIN_VEL = 1.0
MAX_ANG_VEL = 1.5

# "GoForward" class inherits from the base class "Node"
class GoForward(Node):

    counter = 0
    def __init__(self):
        
        # Initialize the node
        super().__init__('GoforwardCmd_publisher')
        
        # Initialize the publisher
        self.velocity_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds

        self.error_publisher = self.create_publisher(Float32, '/error',10)

        # Odometry subscriber
        self.odometry_subscriber_ = self.create_subscription(
            Odometry,
            '/odom',
            self.update_pose,
            10)
        
        # Initialize a timer that excutes call back function every 0.5 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize goal point
        self.goal_pose = None
        
        # Initialize current point
        self.current_pose = None

        # Initialize tolerance for goal positions
        self.tol = 0.1

        # Initialzie angle tolerance (radians)
        self.angle_tol = pi / 18

        # Initialize save path variables
        self.path = []
        self.save_path = False
        self.time = None

        # Initialize to true to ask the input
        self.goal_reached = True

        self.angle_req = False

    def prompt_user(self):

        msg = ("Choose what you want to do:\n"
        "1. Give position (x and y coordinates) of the goal.\n"
        "2. Give position and the orientation of the goal x,y,theta.\n"
        "3. Give a path for robot to follow using teleop\n"
        "Choose (1/2/3): ")
        
        while True:
            print(msg,end="")
            try:
                option = int(input(""))
                print("\n")
                if option not in [1,2,3]:
                    print("Give a valid integer (1/2/3)")
                    continue
                
                if option == 1:
                    self.read_goal()
                    self.angle_req = False
                    break
                elif option == 2:
                    self.read_goal(need_theta=True)
                    self.angle_req = True
                    break
                elif option == 3:
                    self.save_path = True
                    self.angle_req = False
                    break
            except ValueError:
                print("The input has to be an integer!\n")
                continue
        
        return True

     
    def read_goal(self,need_theta = False):

        pose = Pose()
        
        x = float(input("Give x-coordinate of the new goal: "))
        y = float(input("Give y-coordinate of the new goal: "))
        
        if need_theta:
            theta = float(input("Give theta in degrees: "))
            theta = pi * theta / 180
            q = self.to_quaternion(0,0,theta)

            pose.orientation = q

        pose.position.x = x
        pose.position.y = y
        
        self.goal_pose = pose
        self.get_logger().info(f"Received a new goal: x={x}, y={y}")


    def distance(self,pos1,pos2):

        # Only use x and y coordinates of the positions
        d = sqrt(pow(pos1.x-pos2.x,2) + pow(pos1.y-pos2.y,2))
        return d

    def update_pose(self,msg):

        self.current_pose = msg.pose.pose

        # Store the pose if we are recording a path.
        if self.save_path:
            # Store starting time
            if not self.time:
                self.time = perf_counter()

            self.path.append(self.current_pose)
            
            # Record path at least 5 seconds.
            time_elapsed = perf_counter() - self.time
            if time_elapsed > 5:
                pose1 = self.path[-1]
                pose2 = self.path[-2]
                
                # Stop recording path if the position of the robot 
                # has not changed.
                d =  self.distance(pose1.position, pose2.position)
                if d < 0.001:
                    self.save_path = False
                    # We dont all poses in order to create a path
                    self.path = self.path[0::40]

    def linear_vel(self, goal_pose, constant=1):
        return constant * self.distance(goal_pose,self.current_pose.position)
        
    def steering_angle(self, goal_pose):
        goal_pos = self.goal_pose.position
        current_pos = self.current_pose.position
        return atan2(goal_pos.y - current_pos.y, goal_pos.x - current_pos.x)
    
    def angular_vel(self, goal_pose, constant=1.5):
        # Get rotation around z-axis from quaternion.
        theta = self.yaw_from_quaternion(self.current_pose.orientation)
        angle_dif = self.steering_angle(goal_pose) - theta
        return constant * (angle_dif)


    def to_quaternion(self,roll, pitch, yaw):
        
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5) 
        cy = cos(yaw * 0.5) 
        sy = sin(yaw * 0.5)
        
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        return q

    def yaw_from_quaternion(self,q):

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = atan2(siny_cosp, cosy_cosp)
        
        return yaw

    def constrain_between(self,value,min,max):

        if value < min:
            value = min
        elif value > max:
            value = max
        
        return value

        
    def timer_callback(self):

        # Just in case if odometry subscription hasn't updated the pose yet.
        if self.current_pose is None:
            return

        # If turtlebot is moving because of teleop, skip the function.
        if self.save_path:
            return

        if self.goal_reached and len(self.path) > 0:
            # Read goals from the recorded path.
            
            self.goal_pose = self.path[0]
            del self.path[0]
            self.goal_reached = False

        elif self.goal_reached:
            # Goal reached, stop the robot for now and for a new goal.
            
            if self.goal_pose is not None:
                msg = (f"The goal reached. Current position: x={round(self.current_pose.position.x,2)}, y={round(self.current_pose.position.y,2)}. "
                    f"Error: {round(self.distance(self.goal_pose.position,self.current_pose.position),4)}.")
                self.get_logger().info(msg)
            
            # Prompt for a new goal
            self.prompt_user()

            # if option 3 was chosen, skip the rest of the function for now.
            if self.save_path:
                return

            self.goal_reached = False

        
        velocity_msg = Twist()
        error_msg = Float32()

        current_error = self.distance(self.goal_pose.position, self.current_pose.position)
        error_msg.data = current_error

        if self.angle_req:
            angle_error = self.yaw_from_quaternion(self.goal_pose.orientation)  - self.yaw_from_quaternion(self.current_pose.orientation)
        
        self.error_publisher.publish(error_msg)

        if  current_error > self.tol:

            current_angle = self.yaw_from_quaternion(self.current_pose.orientation)
            steer_angle = self.steering_angle(self.goal_pose) - current_angle

            too_big_angle = abs(steer_angle) > pi/2

            if too_big_angle:
                lin_vel = 0.1
                ang_vel = 0.5
            else:
                lin_vel = self.linear_vel(self.goal_pose.position)
                ang_vel = self.angular_vel(self.goal_pose)

            
            velocity_msg.linear.x = self.constrain_between(lin_vel,-MAX_LIN_VEL,MAX_LIN_VEL)
            velocity_msg.angular.z = self.constrain_between(ang_vel,-MAX_ANG_VEL,MAX_ANG_VEL)
            
            # Publishing velocity msg.
            self.velocity_publisher_.publish(velocity_msg)
        
        elif self.angle_req and abs(angle_error) > self.angle_tol:
            # The position is already correct but we are given also a requirement for the orientation
            # Keep turning until we are close enough.
            velocity_msg.linear.x = 0.0
            velocity_msg.angular.z = 0.2

            # Publishing velocity msg.
            self.velocity_publisher_.publish(velocity_msg)
        else:
            # The goal reached. Stop the robot.
            self.velocity_publisher_.publish(Twist())
            self.goal_reached = True


    def stop_turtlebot(self):
        # define what happens when program is interrupted
        # log that turtlebot is being stopped
        self.get_logger().info('stopping turtlebot')
        # publishing an empty Twist() command sets all velocity components to zero
        # Otherwise turtlebot keeps moving even if command is stopped
        self.velocity_publisher_.publish(Twist())
        
    
def main(args=None):
    rclpy.init(args=args)
    
    # we are using try-except tools to  catch keyboard interrupt
    try:
        # create an object for GoForward class
        cmd_publisher = GoForward()
        # continue untill interrupted
        rclpy.spin(cmd_publisher)
        
    except KeyboardInterrupt:
        # execute shutdown function
        cmd_publisher.stop_turtlebot()
        # clear the node
        cmd_publisher.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
