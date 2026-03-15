import time
import mujoco.viewer
import mujoco
import numpy as np
import torch
import yaml
import matplotlib.pyplot as plt
import argparse
import pickle
# from gamepaded import gamepad_reader
NUM_MOTOR = 12

mujoco.mj_loadPluginLibrary('/home/ray/mujoco/plugin/mujoco_ray_caster/lib/libsensor_raycaster.so')

###
def get_ray_caster_info(model: mujoco.MjModel, data: mujoco.MjData, sensor_name: str):
    data_ps = []
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
    if sensor_id == -1:
        print("Sensor not found")
        return 0, 0, data_ps
    sensor_plugin_id = model.sensor_plugin[sensor_id]
    state_idx = model.plugin_stateadr[sensor_plugin_id]
    state_num = model.plugin_statenum[sensor_plugin_id]
    for i in range(state_idx + 2, state_idx + state_num, 2):
        if i + 1 < len(data.plugin_state):
            data_ps.append((int(data.plugin_state[i]), int(data.plugin_state[i + 1])))
    h_ray_num = (
        int(data.plugin_state[state_idx]) if state_idx < len(data.plugin_state) else 0
    )
    v_ray_num = (
        int(data.plugin_state[state_idx + 1])
        if state_idx + 1 < len(data.plugin_state)
        else 0
    )
    return h_ray_num, v_ray_num, data_ps

def get_sensor_data(sensor_name):
    sensor_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
    if sensor_id == -1:
        raise ValueError(f"Sensor '{sensor_name}' not found in model!")
    start_idx = m.sensor_adr[sensor_id]
    dim = m.sensor_dim[sensor_id]
    sensor_values = d.sensordata[start_idx : start_idx + dim]
    return sensor_values

###

def calculate_com_in_base_frame(model, data, base_body_id):
    total_mass = 0.0
    com_sum = np.zeros(3)

    # Get base position and rotation
    base_pos = data.xipos[base_body_id]  # Position of the base in world coordinates
    base_rot = data.ximat[base_body_id].reshape(3, 3)  # Rotation matrix of the base

    for i in range(model.nbody):
        # Get body mass and world COM position
        mass = model.body_mass[i]
        world_com = data.xipos[i]

        # Transform COM to base coordinates
        local_com = world_com - base_pos  # Translate to base origin
        local_com = base_rot.T @ local_com  # Rotate into base frame

        # Accumulate mass-weighted positions
        com_sum += mass * local_com
        total_mass += mass

    # Compute overall COM in base coordinates
    center_of_mass_base = com_sum / total_mass
    return center_of_mass_base

def quat_rotate_inverse(q, v):
    """
    Rotate a vector by the inverse of a quaternion.
    Direct translation from the PyTorch version to NumPy.
    
    Args:
        q: The quaternion in (w, x, y, z) format. Shape is (..., 4).
        v: The vector in (x, y, z) format. Shape is (..., 3).
        
    Returns:
        The rotated vector in (x, y, z) format. Shape is (..., 3).
    """
    q_w = q[..., 0]
    q_vec = q[..., 1:]
    
    # Equivalent to (2.0 * q_w**2 - 1.0).unsqueeze(-1)
    term1 = 2.0 * np.square(q_w) - 1.0
    term1_expanded = np.expand_dims(term1, axis=-1)
    a = v * term1_expanded
    
    # Equivalent to torch.cross(q_vec, v, dim=-1) * q_w.unsqueeze(-1) * 2.0
    q_w_expanded = np.expand_dims(q_w, axis=-1)
    b = np.cross(q_vec, v) * q_w_expanded * 2.0
    
    # Equivalent to the torch.bmm or torch.einsum operations
    # This calculates the dot product between q_vec and v
    dot_product = np.sum(q_vec * v, axis=-1)
    dot_product_expanded = np.expand_dims(dot_product, axis=-1)
    c = q_vec * dot_product_expanded * 2.0
    
    return a - b + c

def get_gravity_orientation(quaternion):
    """
    Get the gravity vector in the robot's base frame.
    Uses the exact algorithm from your PyTorch code.
    
    Args:
        quaternion: Quaternion in (w, x, y, z) format.
        
    Returns:
        3D gravity vector in the robot's base frame.
    """
    # Ensure quaternion is a numpy array
    quaternion = np.array(quaternion)
    
    # Standard gravity vector in world frame (pointing down)
    gravity_world = np.array([0, 0, -1])
    
    # Handle both single quaternion and batched quaternions
    if quaternion.shape == (4,):
        quaternion = quaternion.reshape(1, 4)
        gravity_world = gravity_world.reshape(1, 3)
        result = quat_rotate_inverse(quaternion, gravity_world)[0]
    else:
        gravity_world = np.broadcast_to(gravity_world, quaternion.shape[:-1] + (3,))
        result = quat_rotate_inverse(quaternion, gravity_world)
    
    return result

def quat_to_rot_matrix(quat: np.ndarray) -> np.ndarray:
    """Convert input quaternion to rotation matrix.

    Args:
        quat (np.ndarray): Input quaternion (w, x, y, z).

    Returns:
        np.ndarray: A 3x3 rotation matrix.
    """
    q = np.array(quat, dtype=np.float64, copy=True)
    nq = np.dot(q, q)
    if nq < 1e-10:
        return np.identity(3)
    q *= np.sqrt(2.0 / nq)
    q = np.outer(q, q)
    return np.array(
        (
            (1.0 - q[2, 2] - q[3, 3], q[1, 2] - q[3, 0], q[1, 3] + q[2, 0]),
            (q[1, 2] + q[3, 0], 1.0 - q[1, 1] - q[3, 3], q[2, 3] - q[1, 0]),
            (q[1, 3] - q[2, 0], q[2, 3] + q[1, 0], 1.0 - q[1, 1] - q[2, 2]),
        ),
        dtype=np.float64,
    )

def pd_control(target_q, q, kp, target_dq, dq, kd):
    """Calculates torques from position commands"""
    return (target_q - q) * kp + (target_dq - dq) * kd


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("config_file", type=str, help="config file name in the config folder")
    args = parser.parse_args()
    config_file = args.config_file
    with open(f"{config_file}", "r") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
        policy_path = config["policy_path"]
        policy = torch.jit.load(policy_path)
        policy.eval()
        xml_path = config["xml_path"]
        simulation_duration = config["simulation_duration"]
        simulation_dt = config["simulation_dt"]
        control_decimation = config["control_decimation"]

        kps = np.array(config["kps"], dtype=np.float32)
        kds = np.array(config["kds"], dtype=np.float32)

        default_angles = np.array(config["default_angles"], dtype=np.float32)

        ang_vel_scale = config["ang_vel_scale"]
        dof_pos_scale = config["dof_pos_scale"]
        dof_vel_scale = config["dof_vel_scale"]
        action_scale = config["action_scale"]
        cmd_scale = np.array(config["cmd_scale"], dtype=np.float32)

        num_actions = config["num_actions"]
        num_obs = config["num_obs"]
        one_step_obs_size = config["one_step_obs_size"]
        obs_buffer_size = config["obs_buffer_size"]
        
        cmd = np.array(config["cmd_init"], dtype=np.float32)


    target_dof_pos = default_angles.copy()
    action = np.zeros(num_actions, dtype=np.float32)
    obs = np.zeros(num_obs, dtype=np.float32)
    obs_tensor_buf = torch.zeros((1, one_step_obs_size * obs_buffer_size))

    # gamepad = gamepad_reader.Gamepad(vel_scale_x=0.5, vel_scale_y=0.5, vel_scale_rot=0.8)

    # Load robot model
    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    m.opt.timestep = simulation_dt
    base_body_id = 1


    # Record data
    
    lin_vel_data_list = []
    ang_vel_data_list = []
    gravity_b_list = []
    joint_vel_list = []
    joint_pos_list=[]
    joint_tau_list=[]
    action_list = []
    feet_list=[]
    
    counter = 0
    h_rays, v_rays, pairs = get_ray_caster_info(m, d, "raycasterlidar")
    with mujoco.viewer.launch_passive(m, d) as viewer:
        # Close the viewer automatically after simulation_duration wall-seconds.
        start = time.time()
        while viewer.is_running() and time.time() - start < simulation_duration:
            step_start = time.time()
    
            tau = pd_control(target_dof_pos, d.sensordata[:NUM_MOTOR], kps, np.zeros(12), d.sensordata[NUM_MOTOR:NUM_MOTOR + NUM_MOTOR], kds)

            d.ctrl[:] = tau

            # mj_step can be replaced with code that also evaluates
            # a policy and applies a control signal before stepping the physics.
            mujoco.mj_step(m, d)

            h_rays, v_rays, pairs = get_ray_caster_info(m, d, "raycaster")
            # print(f"[data_point,data_size]: {pairs}")
            img = d.sensor("raycaster").data
            depth_img = np.array(img[pairs[0][0] : pairs[0][0] + pairs[0][1]], dtype=np.float32).reshape(v_rays, h_rays)
            
            depth_img=depth_img.T
            
            # print(depth_img.shape)
            depth_img=-(depth_img-0.419)
            # print ("dep_img",depth_img)

            depth_img = depth_img.reshape(-1)
            print ("dep_img",depth_img)

            counter += 1
            if counter % control_decimation == 0 and counter > 0:

                # create observation
                foot_ids = [4, 6, 8, 10]  
                foot_heights = [d.xipos[i][2] for i in foot_ids]  #get feethigh
                

                position = d.qpos[:3]
                qpos = d.sensordata[:12]
                qvel = d.sensordata[12:24]
                ang_vel_B = d.sensordata[40:43]
                imu_quat =d.sensordata[36:40]
                cmd_vel = np.array(config["cmd_init"], dtype=np.float32)

                gravity_b = get_gravity_orientation(imu_quat)

                #
                scan_dots = np.zeros(10, dtype=np.float32)
                #

                obs_list = [
                    cmd_vel * cmd_scale,
                    ang_vel_B * ang_vel_scale,
                    gravity_b,
                    (qpos - default_angles) * dof_pos_scale,
                    qvel * dof_vel_scale,
                    action.astype(np.float32),
                    depth_img.astype(np.float32)
                ]

                # print(obs_list)
                ## Record Data ##
                ang_vel_data_list.append(ang_vel_B * ang_vel_scale)
                gravity_b_list.append(gravity_b)
                joint_vel_list.append(qvel * dof_vel_scale)
                joint_pos_list.append(np.array(qpos))
                joint_tau_list.append(tau)

                action_list.append(action)
                feet_list.append(foot_heights)
   
                ###
                obs_list = [torch.tensor(obs, dtype=torch.float32) if isinstance(obs, np.ndarray) else obs for obs in obs_list]

                # for i, obs in enumerate(obs_list):
                #     print(i, type(obs), obs.shape if isinstance(obs, torch.Tensor) else "not tensor")

                obs = torch.cat(obs_list, dim=0).unsqueeze(0)

                obs_tensor_buf = torch.cat([
                    obs,
                    obs_tensor_buf[:, :-one_step_obs_size]
                ], dim=1)
                obs_tensor_buf = torch.clip(obs_tensor_buf, -100, 100)

                # policy inference
                action = policy(obs_tensor_buf).detach().numpy().squeeze()

                # transform action to target_dof_pos
                if counter < 300:
                    target_dof_pos = default_angles
                else:
                    target_dof_pos = action * action_scale + default_angles

            # Pick up changes to the physics state, apply perturbations, update options from GUI.
            viewer.sync()

            # Rudimentary time keeping, will drift relative to wall clock.
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    # Plot the collected data after the simulation ends
    plt.figure(figsize=(18, 20))

    plt.subplot(4, 2, 1)
    for i in range(3): 
        plt.plot([step[i] for step in lin_vel_data_list], label=f"Linear Velocity {i}")
    plt.title(f"History Linear Velocity", fontsize=10, pad=10)  # Added pad for spacing
    plt.legend()
    plt.subplot(4, 2, 2)
    for i in range(3):
        plt.plot([step[i] for step in ang_vel_data_list], label=f"Angular Velocity {i}")
    plt.title(f"History Angular Velocity", fontsize=10, pad=10)  # Added pad for spacing
    plt.legend()
    plt.subplot(4, 2, 3)
    for i in range(3):
        plt.plot([step[i] for step in gravity_b_list], label=f"Project Gravity {i}")
    plt.title(f"History Project Gravity", fontsize=10, pad=10)  # Added pad for spacing
    plt.legend()
    plt.subplot(4, 2, 5)
    for i in range(12):
        plt.plot([step[i] for step in joint_vel_list], label=f"Joint Velocity {i}")
    plt.title(f"History Joint Velocity", fontsize=10, pad=10)  # Added pad for spacing
    plt.legend()
    plt.subplot(4, 2, 6)
    for i in range(2):
        plt.plot([step[i] for step in action_list], label=f"velocity Command {i}")
    plt.title(f"History Torque Command", fontsize=10, pad=10)  # Added pad for spacing
    plt.subplot(4, 2, 7)
    Footname=["FL","RL","FR","RR"]
    for i in range(len(feet_list[0])):
        plt.plot([step[i] for step in feet_list], label=f"{Footname[i]} Height")# 0FL 1FR 2RL 3RR
    plt.title("Foot Height (z)", fontsize=10, pad=10)
    plt.legend()
    plt.tight_layout()
    plt.show()