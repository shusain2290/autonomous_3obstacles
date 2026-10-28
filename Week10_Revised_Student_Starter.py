'''CDS215 Week 10 — Husky obstacle avoidance STUDENT STARTER.
Based on the instructor's Week 10 working code.
Requires: pip install pybullet. Run locally with a desktop GUI.

Work from Teaching Section 1 downward. In Section 6, switch between
existing and new movement logic by moving TWO triple-quote lines.
'''
import math
import time
import pybullet as p
import pybullet_data

# ============================================================
# TEACHING SECTION 1 — SETTINGS
# ============================================================
GOAL_RADIUS = 0.65
SPEED = 7.0
TURN_SPEED = 4.0
HEADING_TOLERANCE = 0.14
SENSOR_RANGE = 2.0
SENSOR_HEIGHT = 0.35
SENSOR_OFFSET = 0.65
STOP_DISTANCE = 1.0

# STUDENTS TYPE THREE SETTINGS BELOW:
# TURN_STEPS = ...     # 360 steps = 1.5 simulated seconds
# FORWARD_STEPS = ...  # 480 steps = 2 simulated seconds
# SENSOR_ANGLE = ...   # math.radians(45)
TURN_STEPS = 360
FORWARD_STEPS = 480
SENSOR_ANGLE = math.radians(45)



# Today's class uses ONE destination and ONE obstacle.
waypoints = [(4, 2), (6,7), (2,8)]
LEFT_WHEELS = [2, 4]
RIGHT_WHEELS = [3, 5]

# ============================================================
# EXISTING WHEEL COMMANDS — DO NOT EDIT
# ============================================================
def set_wheel_speeds(robot_id, left, right):
    for joint in LEFT_WHEELS:
        p.setJointMotorControl2(robot_id, joint, p.VELOCITY_CONTROL,
                                targetVelocity=left, force=100)
    for joint in RIGHT_WHEELS:
        p.setJointMotorControl2(robot_id, joint, p.VELOCITY_CONTROL,
                                targetVelocity=right, force=100)

def stop_robot(robot_id):
    set_wheel_speeds(robot_id, 0, 0)

def move_forward(robot_id):
    set_wheel_speeds(robot_id, SPEED, SPEED)

def turn_left(robot_id):
    set_wheel_speeds(robot_id, -TURN_SPEED, TURN_SPEED)

def turn_right(robot_id):
    set_wheel_speeds(robot_id, TURN_SPEED, -TURN_SPEED)

def angle_difference(target_angle, current_angle):
    return math.atan2(math.sin(target_angle - current_angle),
                      math.cos(target_angle - current_angle))

# ============================================================
# TEACHING SECTION 2 — SENSOR
# ============================================================
# CHANGE the function definition to add sensor_angle=0.
# CHANGE look_heading to heading + sensor_angle.
# Leave everything else in this function alone.
def detect_obstacle(robot_id, x, y, heading, sensor_angle=0):
    look_heading = heading + sensor_angle   
    ray_start = [x + SENSOR_OFFSET * math.cos(look_heading),
                 y + SENSOR_OFFSET * math.sin(look_heading), SENSOR_HEIGHT]
    ray_end = [ray_start[0] + SENSOR_RANGE * math.cos(look_heading),
               ray_start[1] + SENSOR_RANGE * math.sin(look_heading), SENSOR_HEIGHT]
    hit = p.rayTest(ray_start, ray_end)[0]
    if hit[0] != -1 and hit[0] != robot_id:
        p.addUserDebugLine(ray_start, ray_end, [1, 0, 0], 3, lifeTime=0.1)
        return hit[2] * SENSOR_RANGE
    p.addUserDebugLine(ray_start, ray_end, [0, 1, 0], 3, lifeTime=0.1)
    return None

# ============================================================
# TEACHING SECTION 3 — WHICH WAY IS CLEARER?
# ============================================================
# STUDENTS TYPE choose_direction(left, right) HERE.
# Convert a None reading to SENSOR_RANGE, compare, return
# "left" when left >= right; otherwise return "right".
def choose_direction(left, right):
    if left is None:
        left = SENSOR_RANGE
    if right is None:
        right = SENSOR_RANGE
    if left >= right:
        return "left"
    else:
        return "right"

# ============================================================
# EXISTING SCENE SETUP — DO NOT EDIT
# ============================================================
def add_target_markers():
    for i, (x, y) in enumerate(waypoints):
        marker = p.createVisualShape(p.GEOM_CYLINDER, radius=GOAL_RADIUS,
                                     length=0.02, rgbaColor=[1, 0.75, 0, 1])
        p.createMultiBody(baseMass=0, baseVisualShapeIndex=marker,
                          basePosition=[x, y, 0.015])
        p.addUserDebugText(f"TARGET {i + 1}", [x, y, 0.45],
                           textColorRGB=[0.5, 0.3, 0], textSize=1.5)

def add_obstacle(x, y, color=(1, 0, 0, 1)):
    size = [0.6, 0.6, 0.5]
    collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
    visual = p.createVisualShape(p.GEOM_BOX, halfExtents=size,
                                 rgbaColor=color)
    return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision,
                             baseVisualShapeIndex=visual,
                             basePosition=[x, y, 0.5])

client = p.connect(p.GUI)
if client < 0:
    raise RuntimeError("Cannot open PyBullet GUI")
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240)
p.loadURDF("plane.urdf")
robot_id = p.loadURDF("husky/husky.urdf", [-3, -1.5, 0.2])
add_target_markers()
add_obstacle(2, 1)
add_obstacle(5, 5)
add_obstacle(4, 7)
p.resetDebugVisualizerCamera(cameraDistance=14, cameraYaw=45,
                              cameraPitch=-65, cameraTargetPosition=[3, 4, 0])

current_target = 0
TARGET_X, TARGET_Y = waypoints[current_target]
mission_complete = False
last_report = time.monotonic()  # Console reporting only; NOT avoidance timing.

# ============================================================
# TEACHING SECTION 4 — AVOIDANCE MEMORY
# ============================================================
# STUDENTS TYPE FOUR STATE VARIABLES HERE:
avoiding_obstacle = False
avoid_direction = None
avoid_stage = None
avoid_counter = 0

# ============================================================
# MAIN SIMULATION LOOP — ONLY EDIT MARKED SECTIONS
# ============================================================
print("Husky code-along: press Q in PyBullet to quit.")
while p.isConnected():
    keys = p.getKeyboardEvents()
    if ord("q") in keys and keys[ord("q")] & p.KEY_WAS_TRIGGERED:
        break
    position, orientation = p.getBasePositionAndOrientation(robot_id)
    x, y = position[0], position[1]
    heading = p.getEulerFromQuaternion(orientation)[2]
    distance = math.hypot(TARGET_X - x, TARGET_Y - y)
    target_angle = math.atan2(TARGET_Y - y, TARGET_X - x)
    heading_error = angle_difference(target_angle, heading)

    # ========================================================
    # TEACHING SECTION 5 — READ THREE SENSORS
    # ========================================================
    center = detect_obstacle(robot_id, x, y, heading, 0)
    left = detect_obstacle(robot_id, x, y, heading, SENSOR_ANGLE)   # REPLACE with detect_obstacle(..., SENSOR_ANGLE)
    right = detect_obstacle(robot_id, x, y, heading, -SENSOR_ANGLE)  # REPLACE with detect_obstacle(..., -SENSOR_ANGLE)

    # ========================================================
    # TEACHING SECTION 6 — MOVEMENT DECISION
    # ========================================================
    # DO NOT change the waypoint-arrival section.
    if distance < GOAL_RADIUS and not mission_complete:
        stop_robot(robot_id)
        print(f"TARGET {current_target + 1} REACHED")
        current_target += 1
        if current_target < len(waypoints):
            TARGET_X, TARGET_Y = waypoints[current_target]
        else:
            mission_complete = True
            print("MISSION COMPLETE")
    elif not mission_complete:
        # OLD MOVEMENT LOGIC: ACTIVE AT FIRST.
        # When Section 6 begins, CUT the two lines containing ONLY
        # three single quotes from around NEW LOGIC, and PASTE them
        # around OLD LOGIC 
        '''
        if center is not None and center < STOP_DISTANCE:
            stop_robot(robot_id)
        elif heading_error > HEADING_TOLERANCE:
            turn_left(robot_id)
        elif heading_error < -HEADING_TOLERANCE:
            turn_right(robot_id)
        else:
            move_forward(robot_id)
        '''
        # NEW MOVEMENT LOGIC: INACTIVE UNTIL SECTION 6.
        # The only action needed is to MOVE the two ''' lines.
        # Students do not type or reindent any of the nested code.
        
        if avoiding_obstacle:
            avoid_counter += 1
            if avoid_stage == "turning":
                if avoid_counter <= TURN_STEPS:
                    if avoid_direction == "left":
                        turn_left(robot_id)
                    else:
                        turn_right(robot_id)
                else:
                    avoid_stage = "forward"
                    avoid_counter = 0
                    move_forward(robot_id)
            elif avoid_stage == "forward":
                if avoid_counter <= FORWARD_STEPS:
                    move_forward(robot_id)
                else:
                    stop_robot(robot_id)
                    avoiding_obstacle = False
                    avoid_direction = None
                    avoid_stage = None
                    avoid_counter = 0
                    print("Avoidance complete!")

        elif center is not None and center < STOP_DISTANCE:
            stop_robot(robot_id)
            avoiding_obstacle = True
            avoid_direction = choose_direction(left, right)
            avoid_stage = "turning"
            avoid_counter = 0
            print(f"Obstacle detected! Turning {avoid_direction}")
        elif heading_error > HEADING_TOLERANCE:
            turn_left(robot_id)
        elif heading_error < -HEADING_TOLERANCE:
            turn_right(robot_id)
        else:
            move_forward(robot_id)
        

    if not mission_complete and time.monotonic() - last_report >= 2:
        def reading(value):
            return "CLEAR" if value is None else f"{value:.2f} m"
        print(f"Target distance={distance:.2f} m | "
              f"L={reading(left)} C={reading(center)} R={reading(right)}")
        last_report = time.monotonic()

    # Exactly ONE simulation step per loop => 240 steps / simulated sec.
    p.stepSimulation()
    time.sleep(1 / 240)  # Display pacing, NOT maneuver timing.

if p.isConnected():
    stop_robot(robot_id)
    p.disconnect()
