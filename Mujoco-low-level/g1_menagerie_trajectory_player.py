import mujoco
import mujoco.viewer
import numpy as np
import time

#from g1_clap_trajectory import trajectory, cycle_time
from g1_bend_trajectory import trajectory, cycle_time

# 1. Load the G1 model from MuJoCo Menagerie
model_path = "mujoco_menagerie/unitree_g1/scene.xml"
m = mujoco.MjModel.from_xml_path(model_path)
d = mujoco.MjData(m)

# 2. Match the real robot's low-level control frequency (500Hz / 2ms)
m.opt.timestep = 0.002 

# 3. Define Sim2Real PD Gains
# We assign specific stiffness and damping based on the joint name.
gains = {
    "hip": {"kp": 150, "kd": 2},
    "knee": {"kp": 300, "kd": 4},
    "ankle": {"kp": 40, "kd": 2},
    "waist": {"kp": 100, "kd": 2},
    "shoulder": {"kp": 100, "kd": 2},
    "elbow": {"kp": 40, "kd": 1},
    "wrist": {"kp": 20, "kd": 1},
    "default": {"kp": 30, "kd": 1} # For hands/other joints
}

# Pre-allocate gain arrays for all actuators
kp = np.zeros(m.nu)
kd = np.zeros(m.nu)

for i in range(m.nu):
    # Find which joint this actuator controls
    joint_id = m.actuator_trnid[i, 0]
    joint_name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
    
    # Map the gains
    assigned_kp, assigned_kd = gains["default"]["kp"], gains["default"]["kd"]
    for key, val in gains.items():
        if key in joint_name.lower():
            assigned_kp, assigned_kd = val["kp"], val["kd"]
            break
            
    kp[i] = assigned_kp
    kd[i] = assigned_kd

# 4. Target Posture and Interpolation Setup
# m.qpos0 holds the default standing pose defined in the URDF/MJCF
qpos_target = m.qpos0.copy()
interp_duration = 3.0 # Interpolate over 3 seconds

# Launch the interactive viewer
with mujoco.viewer.launch_passive(m, d) as viewer:
    # Read the initial "crouched" or "dropped" state right as sim starts
    mujoco.mj_step(m, d)
    qpos_init = d.qpos.copy()
    
    print("Starting 3-second interpolation loop...")
    
    def get_trajectory_command(t):
        t = t % cycle_time

        for i in range(len(trajectory)-1):
            p1 = trajectory[i]
            p2 = trajectory[i+1]

            if p1["time"] <= t <= p2["time"]:
                alpha = (
                    t-p1["time"]
                ) / (
                    p2["time"]-p1["time"]
                )

                commands={}

                joints=set(
                    p1["joints"].keys()
                ) | set(
                    p2["joints"].keys()
                )

                for joint in joints:
                    q1=p1["joints"].get(
                        joint,
                        p2["joints"].get(joint,0)
                    )

                    q2=p2["joints"].get(
                        joint,
                        q1
                    )

                    commands[joint]=(
                        q1+
                        alpha*(q2-q1)
                    )

                return commands

        return {}

    # Control Loop
    while viewer.is_running():
        step_start = time.time()

        t = d.time

        # ----------------------------
        # 1. Standing interpolation
        # ----------------------------

        if t < interp_duration:
            alpha = t / interp_duration

            q_desired = (
                (1-alpha)*qpos_init
                +
                alpha*qpos_target
            )

        else:
            q_desired = qpos_target.copy()

        # ----------------------------
        # Trajectory command
        # ----------------------------

        motion_time = t - interp_duration

        if motion_time > 0:
            commands = get_trajectory_command(
                motion_time
            )

            for name,value in commands.items():
                jid = mujoco.mj_name2id(
                    m,
                    mujoco.mjtObj.mjOBJ_JOINT,
                    name
                )

                idx = m.jnt_qposadr[jid]

                q_desired[idx]=value

        # ----------------------------
        # 3. Send commands
        # ----------------------------

        for i in range(m.nu):
            joint_id = m.actuator_trnid[i,0]

            qpos_idx = m.jnt_qposadr[joint_id]

            d.ctrl[i] = q_desired[qpos_idx]

        mujoco.mj_step(m,d)

        viewer.sync()

        wait = (
            m.opt.timestep
            -
            (time.time()-step_start)
        )

        if wait > 0:
            time.sleep(wait)
        
        # Throttle simulation to run roughly at real-time for visual debugging
        time_until_next = m.opt.timestep - (time.time() - step_start)
        if time_until_next > 0:
            time.sleep(time_until_next)