import time
import sys

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

#from g1_clap_trajectory import trajectory, cycle_time
from g1_bend_trajectory import trajectory, cycle_time

G1_NUM_MOTOR = 29

Kp = [
    # Legs
    300,300,300,400,400,400,
    300,300,300,400,400,400,

    # Waist
    300,300,300,

    # Arms
    50,50,50,30,20,20,20,
    50,50,50,30,20,20,20
]

Kd = [
    # Legs
    5,5,5,5,5,5,
    5,5,5,5,5,5,

    # Waist
    4,4,4,

    # Arms
    3,3,3,2,2,2,2,
    3,3,3,2,2,2,2
]

class G1JointIndex:
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleB = 4
    LeftAnkleRoll = 5
    LeftAnkleA = 5
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleB = 10
    RightAnkleRoll = 11
    RightAnkleA = 11
    WaistYaw = 12
    WaistRoll = 13        # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistA = 13           # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistPitch = 14       # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistB = 14           # NOTE: INVALID for g1 23dof/29dof with waist locked
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20   # NOTE: INVALID for g1 23dof
    LeftWristYaw = 21     # NOTE: INVALID for g1 23dof
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27  # NOTE: INVALID for g1 23dof
    RightWristYaw = 28    # NOTE: INVALID for g1 23dof

class Mode:
    PR = 0  # Series Control for Pitch/Roll Joints
    AB = 1  # Parallel Control for A/B Joints

class Custom:
    joint_map = {
        # Left leg
        "left_hip_pitch_joint": 0,
        "left_hip_roll_joint": 1,
        "left_hip_yaw_joint": 2,
        "left_knee_joint": 3,
        "left_ankle_pitch_joint": 4,
        "left_ankle_roll_joint": 5,

        # Right leg
        "right_hip_pitch_joint": 6,
        "right_hip_roll_joint": 7,
        "right_hip_yaw_joint": 8,
        "right_knee_joint": 9,
        "right_ankle_pitch_joint": 10,
        "right_ankle_roll_joint": 11,

        # Waist
        "waist_yaw_joint": 12,
        "waist_roll_joint": 13,
        "waist_pitch_joint": 14,

        # Left arm
        "left_shoulder_pitch_joint": 15,
        "left_shoulder_roll_joint": 16,
        "left_shoulder_yaw_joint": 17,
        "left_elbow_joint": 18,
        "left_wrist_roll_joint": 19,
        "left_wrist_pitch_joint": 20,
        "left_wrist_yaw_joint": 21,

        # Right arm
        "right_shoulder_pitch_joint": 22,
        "right_shoulder_roll_joint": 23,
        "right_shoulder_yaw_joint": 24,
        "right_elbow_joint": 25,
        "right_wrist_roll_joint": 26,
        "right_wrist_pitch_joint": 27,
        "right_wrist_yaw_joint": 28
    }

    def __init__(self):
        self.control_dt_ = 0.002
        self.mode_pr_ = Mode.PR
        self.mode_machine_ = 0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()  
        self.low_state = None 
        self.update_mode_machine_ = False
        self.crc = CRC()
        self.stand_q = [
            0,0,0,0,0,0,
            0,0,0,0,0,0,

            0,0,-0.1,
            
            0,0,0,0,0,0,0,
            0,0,0,0,0,0,0
        ]
        self.start_q = None
        self.start_time = None
        self.interp_duration = 3.0
        self.trajectory_ready_time = None
        self.trajectory_interp_duration = 3.0
        self.trajectory_start_q = None
        self.start_motion = False
        self.restart_requested = False
        self.trajectory_finished = False
        self.last_trajectory_q = None
        self.trajectory_start_time = None
        self.restart_start_q = None
        self.trajectory_started = False

    def Init(self):
        # simulation does not need motionswitcher, but the real robot does
        '''
        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()

        status, result = self.msc.CheckMode()
        while result['name']:
            self.msc.ReleaseMode()
            status, result = self.msc.CheckMode()
            time.sleep(1)
        '''
        self.lowcmd_publisher_ = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher_.Init()

        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )

        while self.update_mode_machine_ == False:
            time.sleep(1)

        if self.update_mode_machine_ == True:
            self.lowCmdWriteThreadPtr.Start()

    def GetTrajectoryCommand(self, t):
        if t >= cycle_time:
            t = cycle_time

        for i in range(len(trajectory)-1):
            p1 = trajectory[i]
            p2 = trajectory[i+1]

            if p1["time"] <= t <= p2["time"]:
                alpha = (
                    t - p1["time"]
                ) / (
                    p2["time"] - p1["time"]
                )

                result = {}

                joints = (
                    set(p1["joints"].keys()) |
                    set(p2["joints"].keys())
                )

                for joint in joints:
                    q1 = p1["joints"].get(
                        joint,
                        p2["joints"].get(joint,0)
                    )

                    q2 = p2["joints"].get(
                        joint,
                        q1
                    )

                    result[joint] = (
                        q1 +
                        alpha*(q2-q1)
                    )

                return result

        return {}

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg

        if self.update_mode_machine_ == False:
            self.mode_machine_ = self.low_state.mode_machine
            self.update_mode_machine_ = True

    def LowCmdWrite(self):
        if self.low_state is None:
            return

        self.low_cmd.mode_pr = Mode.PR
        self.low_cmd.mode_machine = self.mode_machine_

        if self.restart_requested:
            self.restart_requested = False

            self.trajectory_finished = False

            self.restart_start_q = self.last_trajectory_q.copy()

            self.trajectory_start_q = self.restart_start_q.copy()

            for name, value in trajectory[0]["joints"].items():
                self.trajectory_start_q[self.joint_map[name]] = value

            self.trajectory_ready_time = time.time()

        if self.start_q is None:
            self.start_q = []

            for i in range(G1_NUM_MOTOR):
                self.start_q.append(
                    self.low_state.motor_state[i].q
                )

            self.start_time = time.time()

            print("Starting stand interpolation")

        elapsed = time.time() - self.start_time

        if elapsed < self.interp_duration:
            alpha = elapsed / self.interp_duration

            target_q = []

            for i in range(G1_NUM_MOTOR):
                q = (
                    (1-alpha)*self.start_q[i]
                    +
                    alpha*self.stand_q[i]
                )

                target_q.append(q)

        else:
            if not self.start_motion:
                target_q = self.stand_q.copy()

            else:
                if self.trajectory_finished and self.last_trajectory_q is not None:
                    target_q = self.last_trajectory_q.copy()

                elif self.trajectory_ready_time is None:
                    self.trajectory_ready_time = time.time()

                    if self.last_trajectory_q is None:
                        self.restart_start_q = self.stand_q.copy()

                    else:
                        self.restart_start_q = self.last_trajectory_q.copy()

                    self.trajectory_start_q = self.restart_start_q.copy()

                    for name, value in trajectory[0]["joints"].items():
                        self.trajectory_start_q[self.joint_map[name]] = value

                elapsed2 = time.time() - self.trajectory_ready_time

                if elapsed2 < self.trajectory_interp_duration:
                    alpha = elapsed2 / self.trajectory_interp_duration

                    target_q = []

                    for i in range(G1_NUM_MOTOR):
                        q = (
                            (1 - alpha) * self.restart_start_q[i]
                            + alpha * self.trajectory_start_q[i]
                        )

                        target_q.append(q)

                else:
                    self.trajectory_started = True

                    motion_time = elapsed2 - self.trajectory_interp_duration

                    commands = self.GetTrajectoryCommand(motion_time)

                    target_q = self.trajectory_start_q.copy()

                    for name, value in commands.items():
                        target_q[self.joint_map[name]] = value

                    if motion_time >= cycle_time:
                        self.last_trajectory_q = target_q.copy()
                        self.trajectory_finished = True

        for i in range(G1_NUM_MOTOR):
            motor = self.low_cmd.motor_cmd[i]
            motor.mode = 1
            motor.q = target_q[i]
            motor.dq = 0
            motor.tau = 0
            motor.kp = Kp[i]
            motor.kd = Kd[i]

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)

        self.lowcmd_publisher_.Write(self.low_cmd)

def wait_for_enter(custom):
    while True:
        input("Press Enter to start/restart trajectory...")

        if not custom.trajectory_started:
            custom.start_motion = True
            custom.restart_requested = False
            continue

        if not custom.trajectory_finished:
            print("Trajectory is running, wait until it finishes.")
            continue

        custom.start_motion = True
        custom.restart_requested = True

if __name__ == "__main__":
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    custom = Custom()
    custom.Init()
    custom.Start()

    import threading

    threading.Thread(
        target=wait_for_enter,
        args=(custom,),
        daemon=True
    ).start()

    while True:
        time.sleep(1)