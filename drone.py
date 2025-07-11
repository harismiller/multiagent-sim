import numpy as np

class Drone:
    def __init__(self, name: str, 
                 path: str,
                 agent_type: str, 
                 PID: list = [1,0,0], 
                 zPID: list = [1,0,0],
                 scale: float = 1.0,
                 offset: float = 0.0,
                 state: int = 0) -> None:
                 
        self.name = name
        self.path = path
        self.scale = scale
        self.offset = offset
        self.agent_type = agent_type

        # PID
        self.PID = PID
        self.zPID = zPID

        # PID controller state
        self.integral_error = np.zeros(3)  # Integral error for x, y, z
        self.prev_error = np.zeros(3)      # Previous error for x, y, z

        # Stored poses
        self.pose_prev = [0,0,0]
        self.del_prev = [0,0,0]
        self.pose = [0,0,0]

        self.xgoal = 0
        self.ygoal = 0 
        self.zgoal = 0
        self.flag = 'i'
        self.angle = 0

        self.dind = 0

        # Path planning input type
        self.state = state
        self.dind = 0
    
    def setNext(self, x:float, y:float, z:float, flag:str) -> None:
        self.xgoal = (x-self.offset)*self.scale
        self.ygoal = (y-self.offset)*self.scale
        self.zgoal = 0.0 if self.agent_type == "turtle" else z
        self.flag = flag

    def getNext(self) -> list:
        return [self.xgoal, self.ygoal, self.zgoal, self.flag]

    def setPID(self, PID: list, zPID: list) -> None:
        self.PID = PID
        self.zPID = zPID

    # State update function
    def update_drone_state(self, state, control_input, dt):
        # Unpack state variables
        x, y, z, vx, vy, vz = state
        ax, ay, az = control_input

        # Update position
        x_new = x + vx * dt
        y_new = y + vy * dt
        z_new = z + vz * dt

        # Update velocity
        vx_new = vx + ax * dt
        vy_new = vy + ay * dt
        vz_new = vz + az * dt

        # New state
        return np.array([x_new, y_new, z_new, vx_new, vy_new, vz_new])

    # PID control function
    def pid_control(self, goal, dt):
        # PIDs
        Kp = [self.PID[0], self.PID[0], self.zPID[0]]
        Ki = [self.PID[1], self.PID[1], self.zPID[1]]
        Kd = [self.PID[2], self.PID[2], self.zPID[2]]

        # Compute position error
        position = self.pose  # x, y, z
        error = goal - position

        # Update integral and derivative errors
        self.integral_error += error * dt
        derivative_error = (error - self.prev_error) / dt

        # PID control law
        control_input = Kp * error + Ki * self.integral_error + Kd * derivative_error

        # Update previous error
        self.prev_error = error

        return control_input