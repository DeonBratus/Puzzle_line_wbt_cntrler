from controller import Robot, DistanceSensor, PositionSensor
import time
from colorama import Fore, Style


class Direction:
    NORTH = 'north'
    EAST = 'east'
    SOUTH = 'south'
    WEST = 'west'


class LineType:
    class Straight:
        SHORT = 0
        LONG = 4
        EMPTY = -99

    class Rotations:
        SMOOTH = 10
        STRAIGHT = 20

    class Crossroads:
        T_TYPE = 30
        X_TYPE = 40

    def __init__(self, side, type):
        self.SIDE = side
        self.TYPE = type
        self.TYPESIDE = self.Straight.EMPTY
        self.adding_x_cord = 0
        self.adding_y_cord = 0
        self.chooseSideType()

    def chooseSideType(self):

        if self.TYPE == self.Straight.SHORT or self.TYPE == self.Straight.LONG:
            if self.SIDE == Direction.NORTH:
                self.TYPESIDE = self.TYPE + 0
                self.adding_x_cord = -1
                self.adding_y_cord = 0
            elif self.SIDE == Direction.SOUTH:
                self.TYPESIDE = self.TYPE + 1
                self.adding_x_cord = 1
                self.adding_y_cord = 0
            elif self.SIDE == Direction.WEST:
                self.TYPESIDE = self.TYPE + 2
                self.adding_x_cord = 0
                self.adding_y_cord = 1
            elif self.SIDE == Direction.EAST:
                self.TYPESIDE = self.TYPE + 3
                self.adding_x_cord = 0
                self.adding_y_cord = -1

        elif self.TYPE == self.Rotations.SMOOTH or self.TYPE == self.Rotations.STRAIGHT:
            if self.SIDE == f'{Direction.NORTH}-{Direction.WEST}':
                self.TYPESIDE = self.TYPE + 1
                self.adding_x_cord = 0
                self.adding_y_cord = 1
            elif self.SIDE == f'{Direction.WEST}-{Direction.SOUTH}':
                self.TYPESIDE = self.TYPE + 2
                self.adding_x_cord = 1
                self.adding_y_cord = 0
            elif self.SIDE == f'{Direction.SOUTH}-{Direction.EAST}':
                self.TYPESIDE = self.TYPE + 3
                self.adding_x_cord = 0
                self.adding_y_cord = -1
            elif self.SIDE == f'{Direction.EAST}-{Direction.NORTH}':
                self.TYPESIDE = self.TYPE + 4
                self.adding_x_cord = -1
                self.adding_y_cord = 0
            elif self.SIDE == f'{Direction.NORTH}-{Direction.EAST}':
                self.TYPESIDE = self.TYPE + 5
                self.adding_x_cord = 0
                self.adding_y_cord = -1
            elif self.SIDE == f'{Direction.EAST}-{Direction.SOUTH}':
                self.TYPESIDE = self.TYPE + 6
                self.adding_x_cord = 1
                self.adding_y_cord = 0
            elif self.SIDE == f'{Direction.SOUTH}-{Direction.WEST}':
                self.TYPESIDE = self.TYPE + 7
                self.adding_x_cord = 0
                self.adding_y_cord = 1
            elif self.SIDE == f'{Direction.WEST}-{Direction.NORTH}':
                self.TYPESIDE = self.TYPE + 8
                self.adding_x_cord = -1
                self.adding_y_cord = 0
        elif self.TYPE == self.Crossroads.T_TYPE or self.TYPE == self.Crossroads.X_TYPE:
            if self.SIDE == f'{Direction.NORTH}-left':
                self.TYPESIDE = self.TYPE + 1
                self.adding_x_cord = -1
                self.adding_y_cord = 0
            elif self.SIDE == f'{Direction.NORTH}-right':
                self.TYPESIDE = self.TYPE + 2
                self.adding_x_cord = -1
                self.adding_y_cord = 0
            elif self.SIDE == f'{Direction.SOUTH}-left':
                self.TYPESIDE = self.TYPE + 3
                self.adding_x_cord = 1
                self.adding_y_cord = 0
            elif self.SIDE == f'{Direction.SOUTH}-right':
                self.TYPESIDE = self.TYPE + 4
                self.adding_x_cord = 0
                self.adding_y_cord = 0
            elif self.SIDE == f'{Direction.WEST}-left':
                self.TYPESIDE = self.TYPE + 5
                self.adding_x_cord = 0
                self.adding_y_cord = 1
            elif self.SIDE == f'{Direction.WEST}-right':
                self.TYPESIDE = self.TYPE + 6
                self.adding_x_cord = 0
                self.adding_y_cord = 1
            elif self.SIDE == f'{Direction.EAST}-left':
                self.TYPESIDE = self.TYPE + 7
                self.adding_x_cord = 0
                self.adding_y_cord = -1
            elif self.SIDE == f'{Direction.EAST}-right':
                self.TYPESIDE = self.TYPE + 8
                self.adding_x_cord = 0
                self.adding_y_cord = 0


class RobotController:
    def __init__(self):
        self.robot = Robot()
        self.ir_sensors = [
            self.robot.getDevice('LELIR'),
            self.robot.getDevice('LIR'),
            self.robot.getDevice('CEIR'),
            self.robot.getDevice('RIR'),
            self.robot.getDevice('RIRIR')
        ]
        self.enc_sensors = [
            self.robot.getDevice('left wheel sensor'),
            self.robot.getDevice('right wheel sensor')
        ]
        self.l_motor = self.robot.getDevice('left wheel motor')
        self.r_motor = self.robot.getDevice('right wheel motor')
        self.imu = self.robot.getDevice('inertial unit')
        self.max_speed = 6.28
        self.old_enc_sensors_data = [0, 0]

        self.global_x_cord = 4
        self.global_y_cord = 5

        self.is_cross = -1
        self.old_is_cross = -1
        self.cross_t_side = -1
        # You may want to initialize old_enc_data here or in another appropriate place

    def enable_devices(self):
        devices = self.ir_sensors + self.enc_sensors + [self.imu]
        for device in devices:
            device.enable(32)

    def set_wheel_motors_infinite(self):
        self.l_motor.setPosition(float('inf'))
        self.r_motor.setPosition(float('inf'))

    def get_ir_data(self):
        ir_sens_data = [ir_sens.getValue() for ir_sens in self.ir_sensors]
        for i in range(len(ir_sens_data)):
            if ir_sens_data[i] >= 1000:
                ir_sens_data[i] = 0
        return ir_sens_data

    def get_enc_data(self):
        encs_data = []
        for i in range(len(self.enc_sensors)):
            encs_data.append(self.enc_sensors[i].getValue() - self.old_enc_sensors_data[i])
        delta = encs_data[0] - encs_data[1]
        return encs_data, delta

    def get_imu_data(self):
        side = (Direction.NORTH, Direction.EAST, Direction.SOUTH, Direction.WEST)
        curr_side = ''
        roll, pitch, yaw = self.imu.getRollPitchYaw()
        if abs(yaw) > 2.4:
            curr_side = side[0]
        elif abs(yaw) < 0.75:
            curr_side = side[2]
        elif 0.75 < abs(yaw) < 2.4:
            curr_side = side[1] if yaw / abs(yaw) > 0 else side[3]
        return yaw, curr_side


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0

        self.proportional = None
        self.integral = 0
        self.derivative = None

    def calculate_control_signal(self, error):
        self.proportional = self.kp * error
        self.integral += self.ki * error
        self.derivative = self.kd * (error - self.prev_error)
        self.prev_error = error
        control_signal = self.proportional + self.integral + self.derivative
        return control_signal


def follow_line(controller, pid, ir_data):
    control = pid.calculate_control_signal(ir_data[1] - ir_data[3])
    controller.l_motor.setVelocity(0.3 * robot_controller.max_speed - control)
    controller.r_motor.setVelocity(0.3 * robot_controller.max_speed + control)
    return control


def zeroing_encoders_data(controller: RobotController, enc_data: list):
    controller.old_enc_sensors_data[0] += enc_data[0]
    controller.old_enc_sensors_data[1] += enc_data[1]


def check_line_type(controller: RobotController,
                    ir_data: list, enc_data: list,
                    current_side: str,
                    delta: float,
                    map: list,
                    cords: list):
    # straight lines - short, long
    global old_side, typeRotLine
    long_dist, short_dist = 16, 8

    if enc_data[0] >= long_dist and enc_data[1] >= long_dist and abs(delta) <= 2.8:
        lenLine = LineType.Straight.LONG
        typeLine = LineType(current_side, lenLine)
        if typeLine.TYPESIDE != typeLine.Straight.EMPTY:
            controller.global_x_cord += typeLine.adding_x_cord
            controller.global_y_cord += typeLine.adding_y_cord
            cords.append([controller.global_x_cord, controller.global_y_cord])
            map.append([typeLine.TYPESIDE, [cords[-2][0], cords[-2][1]]])
        zeroing_encoders_data(robot_controller, enc_data)
        return typeLine

    # simple rotation - smooth rotation, straight rotation
    start_rot_delta = 0.2
    end_rot_delta = 2.5

    if abs(delta) <= start_rot_delta:
        old_side = current_side

        if abs(ir_data[0] - ir_data[-1]) > 2:
            typeRotLine = LineType.Rotations.STRAIGHT
        else:
            typeRotLine = LineType.Rotations.SMOOTH

    elif abs(delta) >= end_rot_delta:
        typeLine = LineType(f'{old_side}-{current_side}', typeRotLine)

        if typeLine.TYPESIDE != typeLine.Straight.EMPTY:
            controller.global_x_cord += typeLine.adding_x_cord
            controller.global_y_cord += typeLine.adding_y_cord
            cords.append([controller.global_x_cord, controller.global_y_cord])
            map.append([typeLine.TYPESIDE, [cords[-2][0], cords[-2][1]]])

        zeroing_encoders_data(robot_controller, enc_data)
        return typeLine

    # crossroads - T-type crossroads, X-crossroads

    # TypeCrossLine = LineType(None, LineType.Straight.EMPTY)
    # if ir_data[0] > 4 or ir_data[-1] > 4:
    #     controller.is_cross = 1
    #     controller.old_is_cross = controller.is_cross
    #
    # else:
    #     controller.is_cross = 0
    #
    # if controller.is_cross == 1 and controller.old_is_cross == 1:
    #     if ir_data[0] > 4:
    #         controller.cross_t_side = f'{c_side}-left'
    #     elif ir_data[-1] > 4:
    #         controller.cross_t_side = f'{c_side}-right'
    # elif controller.is_cross == 0 and controller.old_is_cross == 1:
    #     controller.old_is_cross = -1
    #
    #     TypeCrossLine = LineType(controller.cross_t_side, LineType.Crossroads.T_TYPE)
    #     controller.global_x_cord += TypeCrossLine.adding_x_cord
    #     controller.global_y_cord += TypeCrossLine.adding_y_cord
    #     # print(TypeCrossLine.TYPE, TypeCrossLine.SIDE, TypeCrossLine.TYPESIDE)
    #     #if TypeCrossLine.TYPESIDE != LineType.Straight.EMPTY:
    #     cords.append([controller.global_x_cord, controller.global_y_cord])
    #
    #     map.append([TypeCrossLine.TYPESIDE, [cords[-2][0], cords[-2][1]]])
    #     zeroing_encoders_data(robot_controller, enc_data)
    #
    #     # print('Cross_is_end_really')
    #     #print(map)
    #     return TypeCrossLine
    #print(c_side)
    #print(cords)


class LineMap:
    global_map = []

    def __init__(self, cord_data: list, point_data: list, x, y, tar_cord):
        self.cord_list = cord_data
        self.point_list = point_data
        self.target_cord = tar_cord
        self.current_cord_x = x
        self.current_cord_y = y
        self.global_map = []

        self.set_global_map()

    def set_global_map(self):
        for i in range(11):
            x_line_map = []
            for j in range(11):
                x_line_map.append(f'??')
            self.global_map.append(x_line_map)

    def set_data_to_global_map(self):

        points = self.point_list
        pointCurrent = points[-1]
        point_type = pointCurrent[0]
        cord_x = pointCurrent[1][0]
        cord_y = pointCurrent[1][1]

        if point_type != -99:
            map_offset = 5
            map_line.current_cord_x = self.cord_list[-1][1]
            map_line.current_cord_y = self.cord_list[-1][0]
            self.global_map[cord_y + map_offset][cord_x + map_offset] = Fore.GREEN + f'{point_type:02}' + Style.RESET_ALL
            self.global_map[self.target_cord[1] + map_offset][self.target_cord[0] + map_offset] = Fore.BLUE + '**' + Style.RESET_ALL
            self.global_map[map_line.current_cord_x + map_offset][map_line.current_cord_y + map_offset] = Fore.RED + '**' + Style.RESET_ALL

    def show_map(self):
        for i in range(11):
            print(*self.global_map[i])
        print('---------------------')
        return f'00'


robot_controller = RobotController()
robot_controller.enable_devices()
robot_controller.set_wheel_motors_infinite()

timestep = int(robot_controller.robot.getBasicTimeStep())

pid_controller = PIDController(kp=0.2, ki=0.01, kd=0.01)

robot_controller.global_x_cord = 0
robot_controller.global_y_cord = 0

map_line = LineMap([[robot_controller.global_x_cord, robot_controller.global_y_cord]],
                   [[-99, [-99, -99]]], 0, 0, [0, 1])

print(f'Start Cord - {robot_controller.global_x_cord, robot_controller.global_y_cord}')
print(f'Target Cord - {map_line.target_cord[0], map_line.target_cord[1]}')
statusWhite = False
while robot_controller.robot.step(timestep) != -1:

    ir_sens = robot_controller.get_ir_data()
    encoders_data, delta_enc = robot_controller.get_enc_data()
    yaw, c_side = robot_controller.get_imu_data()

    check_line_type(robot_controller,
                    ir_sens, encoders_data,
                    c_side, delta_enc,
                    map_line.point_list,
                    map_line.cord_list)
    try:
        map_line.set_data_to_global_map()
        map_line.show_map()
    except Exception:
        map_line.show_map()
        robot_controller.r_motor.setVelocity(0)
        robot_controller.l_motor.setVelocity(0)
        break

    move_control_signal = follow_line(robot_controller, pid_controller, ir_sens)
    #print(ir_sens[2])
    if (map_line.cord_list[-1][0] == map_line.target_cord[0] and
            map_line.cord_list[-1][1] == map_line.target_cord[1]):  # set X and Y for dead
        map_line.global_map[map_line.target_cord[1]][map_line.target_cord[0]] = Fore.YELLOW + "GG" + Style.RESET_ALL
        robot_controller.r_motor.setVelocity(0)
        robot_controller.l_motor.setVelocity(0)
        print("Доехал!")
        for i in range(11):
            print(*map_line.global_map[i])
        print('FINAL MAP')
        break
