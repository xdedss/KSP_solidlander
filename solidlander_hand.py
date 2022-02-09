
# 手动控制（WASDQE键）

import krpc
import numpy as np
import time, math, random, sys

import utils


DEBUG_LINES = True
DEBUG_UI = True


conn = krpc.connect(name='controller')
space_center = conn.space_center # SpaceCenter对象
vessel = space_center.active_vessel # 当前载具
body = vessel.orbit.body # 当前载具所处的天体
#vessel.control.sas = False

# lat = -0.0972
# lon = -74.5577
lat = -0.09678
lon = -74.61739
body_frame = body.reference_frame # 地固系



def find_module_by_tag_and_name(vessel, tag, name):
    p = vessel.parts.with_tag(tag)
    if (len(p) == 0):
        print('no part ' + tag)
        return
    p = p[0]
    m = [m for m in p.modules if m.name == name]
    if (len(m) == 0):
        print('no module ' + name)
        print([m.name for m in p.modules])
        return
    m = m[0]
    return m



if (DEBUG_LINES):
    line_l = conn.drawing.add_line((0,0,0),(0,0,0), vessel.reference_frame)
    line_l.color = (1, 0, 0)
    line_r = conn.drawing.add_line((0,0,0),(0,0,0), vessel.reference_frame)
    line_r.color = (1, 0, 0)
    line_c = conn.drawing.add_line((0,0,0),(0,0,0), vessel.reference_frame)
    line_c.color = (1, 1, 0)

# print(hinge_l.fields)
# print(servo_l.fields)


def decouple_input(pitch, yaw, roll, throttle):
    # 正方向： pitch:S yaw:D roll:E

    # 常量：矢量最大偏角
    MAX_TANGENT = np.tan(np.radians(20))
    MAX_ROLL_ANGLE = np.radians(20)


    # 限定throttle
    throttle = utils.clamp(throttle, 0, 1)

    # hinge不能超过180，故yaw方向有上限
    # pitch 可以不受限制，但为了保持一致性还是限制一下
    bias_limit = np.sqrt(1 - throttle ** 2) / MAX_TANGENT
    pitch = utils.clamp(pitch, -bias_limit, bias_limit) 
    yaw = utils.clamp(yaw, -bias_limit, bias_limit)
    roll = utils.clamp(roll, -1, 1)

    # 推力矢量（归一化）
    dst_thrust = utils.v3(-yaw * MAX_TANGENT, 1, pitch * MAX_TANGENT)
    dst_thrust = dst_thrust / np.linalg.norm(dst_thrust) * throttle

    # 解耦合
    roll_angle = roll * MAX_ROLL_ANGLE
    # decompose
    # ====^----> dir
    # \   |   /
    #  \  |  /
    #   \ |-/angle
    #    \|/
    decomp_angle = np.arccos(np.linalg.norm(dst_thrust))
    # 考虑throttle=0特殊处理
    if (throttle < 1e-5):
        decomp_dir = utils.v3(1, 0, 0)
        decomp_dir = utils.rotate_around_axis(decomp_dir, utils.v3(0, 1, 0), roll_angle)
    else:
        decomp_dir = np.cross(dst_thrust, utils.v3(0, 0, 1)) * np.tan(decomp_angle)
        decomp_dir = utils.rotate_around_axis(decomp_dir, dst_thrust, roll_angle)
    dir_l = dst_thrust + decomp_dir
    dir_r = dst_thrust - decomp_dir

    # 解算关节角度
    h_l, s_l = decompose_hinge_angles(dir_l, utils.v3(0, 0, 1))
    h_r, s_r = decompose_hinge_angles(dir_r, utils.v3(0, 0, -1))

    # debug lines
    if (DEBUG_LINES):
        line_l.end = tuple(dir_l * 10)
        line_r.end = tuple(dir_r * 10)
        line_c.end = tuple(dst_thrust * 10)
        #print(dst_thrust)

    # radians to degrees
    return np.degrees((h_l, s_l, h_r, s_r))


def decompose_hinge_angles(vec, hinge_axis):
    vec /= np.linalg.norm(vec)
    hinge_axis /= np.linalg.norm(hinge_axis)
    servo_angle = np.pi / 2 - np.arccos(np.dot(vec, hinge_axis))
    hinge_moving_plane_normal = np.cross(vec, hinge_axis)
    hinge_angle = np.pi - np.arccos(np.dot(hinge_moving_plane_normal, np.cross(utils.v3(0, 1, 0), hinge_axis)))
    return hinge_angle, servo_angle



# 固定命名获取module
# h.l h.r s.l s.r
print('find parts with tag')
hinge_l = find_module_by_tag_and_name(vessel, 'h.l', 'ModuleRoboticServoHinge')
hinge_r = find_module_by_tag_and_name(vessel, 'h.r', 'ModuleRoboticServoHinge')
servo_l = find_module_by_tag_and_name(vessel, 's.l', 'ModuleRoboticRotationServo')
servo_r = find_module_by_tag_and_name(vessel, 's.r', 'ModuleRoboticRotationServo')
print(hinge_l, hinge_r, servo_l, servo_r)


# UI
if (DEBUG_UI):
    canvas = conn.ui.stock_canvas

    # Get the size of the game window in pixels
    screen_size = canvas.rect_transform.size

    # Add a panel to contain the UI elements
    panel = canvas.add_panel()

    # Position the panel on the left of the screen
    rect = panel.rect_transform
    rect.size = (200, 200)
    rect.position = (110-(screen_size[0]/2), 0)

    # Add some text displaying the total engine thrust
    pad = 10
    text = panel.add_text("txt1")
    text.rect_transform.position = (pad, pad)
    text.rect_transform.size = (200 - pad * 2, 200 - pad * 2)
    text.color = (1, 1, 1)
    text.size = 18


# PID
# x & z rotation
ctrl_rot = utils.PIDn(2, kp=1.0, kd=1.0, sd=0.3)
ctrl_roll = utils.PID(kp=1.0, kd=1.0, sd=0.3)

# throttle
des_throttle = 0.7
min_throttle = 0.1
max_throttle = 0.9
throttle_k = 5.0
vel_yz_k = 0.04

#sys.exit(0)
vessel.control.sas = False

game_prev_time = space_center.ut # 记录上一帧时间
while (True):
    time.sleep(0.001)
    ut = space_center.ut # 获取游戏内时间
    game_delta_time = ut - game_prev_time # 计算上一帧到这一帧消耗的时间
    if game_delta_time < 0.019: # 如果游戏中还没有经过一个物理帧，不进行计算
        continue
    

    h_l, s_l, h_r, s_r = decouple_input(vessel.control.pitch, vessel.control.yaw, vessel.control.roll, vessel.control.throttle)
    hinge_l.set_field_float('Target Angle', h_l)
    hinge_r.set_field_float('Target Angle', h_r)
    servo_l.set_field_float('Target Angle', s_l)
    servo_r.set_field_float('Target Angle', s_r)

    
    game_prev_time = ut # 更新上一帧时间记录



