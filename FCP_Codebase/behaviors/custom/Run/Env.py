from agent.Base_Agent import Base_Agent
from behaviors.custom.Step.Step_Generator import Step_Generator
from behaviors.custom.Step.Step import Step

from math_ops.Math_Ops import Math_Ops as M
import math
import numpy as np


class Env():
    def __init__(self, base_agent : Base_Agent) -> None:

        self.player = base_agent
        self.step_counter = 0 # to limit episode size
        # self.world = base_agent.world
        # self.ik = base_agent.inv_kinematics
        
        # State space  
        obs_size = 70
        self.obs = np.zeros(obs_size, np.float32)
        
        # # Step behavior defaults
        # self.STEP_DUR = 7
        # self.STEP_Z_SPAN = 0.035
        # self.STEP_Z_MAX = 0.70

        # Action space
        MAX = np.finfo(np.float32).max
        self.no_of_actions = 22
        
        # Step behavior defaults
        self.step_default_dur = 7
        self.step_default_z_span = 0.035
        self.step_default_z_max = 0.70

        # memory variables
        self.act = np.zeros(self.no_of_actions,np.float32)

        # # IK 
        # nao_specs = self.ik.NAO_SPECS
        # self.leg_length = nao_specs[1] + nao_specs[3] # upper leg height + lower leg height
        # feet_y_dev = nao_specs[0] * 1.12 # wider step
        # sample_time = self.world.robot.STEPTIME
        # max_ankle_z = nao_specs[5]

        # self.step_generator = Step_Generator(feet_y_dev, sample_time, max_ankle_z)

        self.step_obj = Step(self.player)

        # self.step_obj : Step = self.player.behavior.get_custom_behavior_object("Step") # Step behavior object

        self.DEFAULT_ARMS = np.array([-90,-90,8,8,90,90,70,70],np.float32)

        self.walk_rel_orientation = None
        self.walk_rel_target = None
        self.walk_distance = None


    def observe(self, init=False):

        r = self.player.world.robot

        if init: # reset variables
            self.step_counter = 0
            self.act = np.zeros(22, np.float32) # memory variable

        # index       observation              naive normalization
        self.obs[0] = self.step_counter        /100  # simple counter: 0,1,2,3...
        self.obs[1] = r.loc_head_z             *3    # z coordinate (torso)
        self.obs[2] = r.loc_head_z_vel         /2    # z velocity (torso)  
        self.obs[3] = r.imu_torso_orientation  /50   # absolute orientation in deg
        self.obs[4] = r.imu_torso_roll         /15   # absolute torso roll  in deg
        self.obs[5] = r.imu_torso_pitch        /15   # absolute torso pitch in deg
        self.obs[6:9] = r.gyro                 /100  # gyroscope
        self.obs[9:12] = r.acc                 /10   # accelerometer

        self.obs[12:18] = r.frp.get('lf', (0,0,0,0,0,0)) #  left foot: relative point of origin (p) and force vector (f) -> (px,py,pz,fx,fy,fz)*
        self.obs[18:24] = r.frp.get('rf', (0,0,0,0,0,0)) # right foot: relative point of origin (p) and force vector (f) -> (px,py,pz,fx,fy,fz)*
        self.obs[15:18] /= 100 # naive normalization of force vector
        self.obs[21:24] /= 100 # naive normalization of force vector
        self.obs[24:44] = r.joints_position[2:22] /100    # position of all joints except head & toes (for robot type 4)
        self.obs[44:64] = r.joints_speed[2:22]    /6.1395 # speed of    all joints except head & toes (for robot type 4)
        # *if foot is not touching the ground, then (px=0,py=0,pz=0,fx=0,fy=0,fz=0)

        if init: # the walking parameters refer to the last parameters in effect (after a reset, they are pointless)
            self.obs[64] = self.step_default_dur    /10 # step duration in time steps
            self.obs[65] = self.step_default_z_span *20 # vertical movement span
            self.obs[66] = self.step_default_z_max      # relative extension of support leg
            self.obs[67] = 1 # step progress
            self.obs[68] = 1 # 1 if left  leg is active
            self.obs[69] = 0 # 1 if right leg is active
        else:
            self.obs[64] = self.step_obj.step_generator.ts_per_step   /10 # step duration in time steps
            self.obs[65] = self.step_obj.step_generator.swing_height  *20 # vertical movement span
            self.obs[66] = self.step_obj.step_generator.max_leg_extension / self.step_obj.leg_length # relative extension of support leg
            self.obs[67] = self.step_obj.step_generator.external_progress # step progress
            self.obs[68] = float(self.step_obj.step_generator.state_is_left_active)     # 1 if left  leg is active
            self.obs[69] = float(not self.step_obj.step_generator.state_is_left_active) # 1 if right leg is active

        '''
        Expected observations for walking parameters/state (example):
        Time step        R  0  1  2  0   1   2   3  4
        Progress         1  0 .5  1  0 .25  .5 .75  1
        Left leg active  T  F  F  F  T   T   T   T  T
        Parameters       A  A  A  B  B   B   B   B  C
        Example note: (A) has a step duration of 3ts, (B) has a step duration of 5ts
        '''

        return self.obs

    def execute(self, action):
        
        r = self.player.world.robot

        # exponential moving average
        self.act = 0.4 * self.act + 0.6 * action

        # execute Step behavior to extract the target positions of each leg (we will override these targets)
        step_zsp =     np.clip(self.step_default_z_span + self.act[20]/300,   0,     0.07)
        step_zmx =     np.clip(self.step_default_z_max  + self.act[21]/30,    0.6,   0.9)
        self.step_obj.execute(self.step_counter == 0, self.step_default_dur, step_zsp, step_zmx)

        # if self.step_counter == 0:
        #     '''
        #     The first time step will change the parameters of the next footstep
        #     It uses default parameters so that the agent can anticipate the next generated pose
        #     Reason: the agent decides the parameters during the previous footstep
        #     '''
        #     # self.player.behavior.execute("Step", self.step_default_dur, self.step_default_z_span, self.step_default_z_max)
        #     self.step_obj.execute(self.step_default_dur, self.step_default_z_span, self.step_default_z_max)
        # else:
            
        #     step_zsp =     np.clip(self.step_default_z_span + self.act[20]/300,   0,     0.07)
        #     step_zmx =     np.clip(self.step_default_z_max  + self.act[21]/30,    0.6,   0.9)

        #     # self.player.behavior.execute("Step", self.step_default_dur, step_zsp, step_zmx)
        #     self.step_obj.execute(self.step_default_dur, step_zsp, step_zmx)

        
        # add action as residuals to Step behavior (the index of these actions is not the typical index because both head joints are excluded)
        new_action = self.act[:20] * 2 # scale up actions to motivate exploration
        new_action[[0,2,4,6,8,10]] += self.step_obj.values_l
        new_action[[1,3,5,7,9,11]] += self.step_obj.values_r
        new_action[12] -= 90 # arms down
        new_action[13] -= 90 # arms down
        new_action[16] += 90 # untwist arms
        new_action[17] += 90 # untwist arms
        new_action[18] += 90 # elbows at 90 deg
        new_action[19] += 90 # elbows at 90 deg

        r.set_joints_target_position_direct( # commit actions:
            slice(2,22),        # act on all joints except head & toes (for robot type 4)
            new_action,         # target joint positions 
            harmonize=False     # there is no point in harmonizing actions if the targets change at every step  
        )

        # self.sync() # run simulation step
        self.step_counter += 1
         
        # reward = r.cheat_abs_pos[0] - self.lastx
        # self.lastx = r.cheat_abs_pos[0]

        # # terminal state: the robot is falling or timeout
        # terminal = r.cheat_abs_pos[2] < 0.3 or self.step_counter > 300

        # return self.observe(), reward, terminal, {}

    # def execute(self, action):
        
    #     r = self.world.robot

    #     # Actions:
    #     # 0,1,2    left ankle pos
    #     # 3,4,5    right ankle pos
    #     # 6,7,8    left foot rotation
    #     # 9,10,11  right foot rotation
    #     # 12,13    left/right arm pitch
    #     # 14,15    left/right arm roll

    #     internal_dist = np.linalg.norm( self.internal_target )
    #     action_mult = 1 if internal_dist > 0.2 else (0.7/0.2) * internal_dist + 0.3

    #     # exponential moving average
    #     self.act = 0.8 * self.act + 0.2 * action * action_mult * 0.7
        
    #     # execute Step behavior to extract the target positions of each leg (we will override these targets)
    #     lfy,lfz,rfy,rfz = self.step_generator.get_target_positions(self.step_counter == 0, self.STEP_DUR, self.STEP_Z_SPAN, self.leg_length * self.STEP_Z_MAX)


    #     # Leg IK
    #     a = self.act
    #     l_ankle_pos = (a[0]*0.02, max(0.01,  a[1]*0.02 + lfy), a[2]*0.01 + lfz) # limit y to avoid self collision
    #     r_ankle_pos = (a[3]*0.02, min(a[4]*0.02 + rfy, -0.01), a[5]*0.01 + rfz) # limit y to avoid self collision
    #     l_foot_rot = a[6:9]  * (3,3,5)
    #     r_foot_rot = a[9:12] * (3,3,5)

    #     # Limit leg yaw/pitch
    #     l_foot_rot[2] = max(0,l_foot_rot[2] + 7)
    #     r_foot_rot[2] = min(0,r_foot_rot[2] - 7)

    #     # Arms actions
    #     arms = np.copy(self.DEFAULT_ARMS) # default arms pose
    #     arm_swing = math.sin(self.step_generator.state_current_ts / self.STEP_DUR * math.pi) * 6
    #     inv = 1 if self.step_generator.state_is_left_active else -1
    #     arms[0:4] += a[12:16]*4 + (-arm_swing*inv,arm_swing*inv,0,0) # arms pitch+roll

    #     # Set target positions
    #     self.execute_ik(l_ankle_pos, l_foot_rot, r_ankle_pos, r_foot_rot)           # legs 
    #     r.set_joints_target_position_direct( slice(14,22), arms, harmonize=False )  # arms

    #     self.step_counter += 1