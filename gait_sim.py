"""Reduced-order planar biped simulation for the MAE 598 passive-energy project."""
from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
import csv
import math
import time
import xml.etree.ElementTree as ET

import mujoco
from mujoco import viewer
import numpy as np

ROOT = Path(__file__).resolve().parent
MODEL_XML = ROOT / "biped.xml"
PASSIVE_JOINT_SPRINGS = {
    "knee_L": (60.0, 2.5),
    "knee_R": (60.0, 2.5),
    "ankle_L": (8.0, 0.5),
    "ankle_R": (8.0, 0.5),
}


@dataclass(frozen=True)
class GaitConfig:
    name: str
    speed: float
    stride_length: float
    duty_factor: float
    duration: float = 6.0
    hip_amplitude: float = 0.30
    hip_kp: float = 48.0
    hip_kd: float = 2.8
    pitch_kp: float = 80.0
    pitch_kd: float = 12.0
    height_kp: float = 1200.0
    height_kd: float = 80.0
    height_amplitude: float = 0.015
    support_fraction: float = 0.35
    swing_retract_kp: float = 1800.0
    swing_retract_kd: float = 50.0
    swing_retract: float = -0.10
    drive_kp: float = 250.0
    drive_ki: float = 100.0
    drive_force_limit: float = 450.0

    @property
    def period(self) -> float:
        return self.stride_length / self.speed


GAITS = {
    # Alternating support with an overlap; a compliant extension is added to the
    # inverted-pendulum-like walking baseline for the project comparison.
    "walk": GaitConfig("walk", speed=0.85, stride_length=0.72, duty_factor=0.62),
    # Shorter stance fraction creates an aerial interval and SLIP-like rebound.
    "run": GaitConfig(
        "run",
        speed=2.0,
        stride_length=1.05,
        duty_factor=0.60,
        drive_kp=150.0,
        drive_ki=50.0,
    ),
}


@dataclass(frozen=True)
class SpringParameters:
    dimensionless_stiffness: float = 15.0  # k L / (m g)
    damping_ratio: float = 0.20


def physical_spring_parameters(
    params: SpringParameters, mass: float, leg_length: float
):
    """Convert kappa=kL/(mg), zeta=c/(2 sqrt(k m_eff)) to SI values."""
    k = params.dimensionless_stiffness * mass * 9.81 / leg_length
    effective_mass_per_leg = mass / 2.0
    c = 2.0 * params.damping_ratio * math.sqrt(k * effective_mass_per_leg)
    return k, c


def make_model(params: SpringParameters):
    """Load the MJCF and insert the candidate spring and damper values."""
    tree = ET.parse(MODEL_XML)
    root = tree.getroot()
    # The two slider joints are the axial leg springs in this template.
    k, c = physical_spring_parameters(params, mass=10.66, leg_length=0.74)
    for name in ("spring_L", "spring_R"):
        joint = root.find(f".//joint[@name='{name}']")
        joint.set("stiffness", f"{k:.9g}")
        joint.set("damping", f"{c:.9g}")
    xml = ET.tostring(root, encoding="unicode")
    return mujoco.MjModel.from_xml_string(xml), k, c


def smoothstep(x: float) -> float:
    x = min(1.0, max(0.0, x))
    return x * x * (3.0 - 2.0 * x)


def hip_reference(t: float, gait: GaitConfig, leg_offset: float) -> float:
    phase = ((t / gait.period) + leg_offset) % 1.0
    d = gait.duty_factor
    a = gait.hip_amplitude
    if phase < d:
        # During stance the leg sweeps from ahead of the body to behind it.
        return -a + 2.0 * a * (phase / d)
    # Swing phase brings the leg forward for the next touchdown.
    u = (phase - d) / (1.0 - d)
    return a - 2.0 * a * smoothstep(u)


def gait_phase(t: float, gait: GaitConfig, leg_offset: float) -> float:
    return ((t / gait.period) + leg_offset) % 1.0


def running_knee_reference(
    phase: float, duty_factor: float, peak_flexion: float = 0.9
) -> float:
    """Bend the knee during swing, then extend it before the next touchdown."""
    if phase < duty_factor or duty_factor >= 1.0:
        return 0.0
    swing_phase = (phase - duty_factor) / (1.0 - duty_factor)
    if swing_phase < 0.5:
        return peak_flexion * smoothstep(swing_phase / 0.5)
    return peak_flexion * (1.0 - smoothstep((swing_phase - 0.5) / 0.5))


def _id(model, object_type, name):
    value = mujoco.mj_name2id(model, object_type, name)
    if value < 0:
        raise ValueError(f"Missing MuJoCo object: {name}")
    return value


def run_simulation(
    gait: GaitConfig,
    params: SpringParameters,
    output_csv: Path | None = None,
    visualize: bool = False,
):
    model, k, c = make_model(params)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)

    jids = {
        n: _id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
        for n in (
            "root_x", "root_z", "root_pitch", "hip_L", "hip_R",
            "spring_L", "spring_R", "knee_L", "knee_R", "ankle_L", "ankle_R",
        )
    }
    qadr = {n: model.jnt_qposadr[j] for n, j in jids.items()}
    dadr = {n: model.jnt_dofadr[j] for n, j in jids.items()}
    aids = {
        n: _id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
        for n in (
            "drive", "vertical_support", "pitch_stabilizer",
            "hip_motor_L", "hip_motor_R", "knee_motor_L", "knee_motor_R",
            "spring_reset_L", "spring_reset_R",
        )
    }

    # Begin with the feet near contact and the trunk at its nominal height.
    data.qpos[qadr["root_z"]] = 0.0  # torso body has a 0.82 m worldbody offset
    data.qpos[qadr["hip_L"]] = -gait.hip_amplitude
    data.qpos[qadr["hip_R"]] = gait.hip_amplitude
    mujoco.mj_forward(model, data)

    dt = model.opt.timestep
    nsteps = int(gait.duration / dt)
    positive_work = 0.0
    negative_work = 0.0
    spring_loaded = 0.0
    spring_released = 0.0
    damping_loss = 0.0
    previous_spring_energy = 0.0
    joint_spring_loaded = 0.0
    joint_spring_released = 0.0
    joint_damping_loss = 0.0
    previous_joint_spring_energy = 0.0
    max_knee_angle = 0.0
    max_ankle_angle = 0.0
    rows = []
    stable = True
    max_height_error = 0.0
    max_spring_deflection = 0.0
    speed_sq_error = 0.0
    speed_error_samples = 0
    single_support_steps = 0
    double_support_steps = 0
    flight_steps = 0
    start_x = data.qpos[qadr["root_x"]]
    drive_integral = 0.0
    floor_geom = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    foot_geoms = {
        _id(model, mujoco.mjtObj.mjOBJ_GEOM, "foot_L"),
        _id(model, mujoco.mjtObj.mjOBJ_GEOM, "foot_R"),
    }
    foot_geom_by_side = {
        side: _id(model, mujoco.mjtObj.mjOBJ_GEOM, f"foot_{side}")
        for side in ("L", "R")
    }
    anchor_site = {
        side: _id(model, mujoco.mjtObj.mjOBJ_SITE, f"anchor_{side}")
        for side in ("L", "R")
    }
    stance_eq = {
        side: _id(model, mujoco.mjtObj.mjOBJ_EQUALITY, f"stance_{side}")
        for side in ("L", "R")
    }
    trunk_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "torso")
    foot_target_x = {
        side: float(data.geom_xpos[geom, 0])
        for side, geom in foot_geom_by_side.items()
    }
    previous_phase = {"L": 0.0, "R": 0.5}
    for side, offset in (("L", 0.0), ("R", 0.5)):
        phase0 = gait_phase(0.0, gait, offset)
        data.eq_active[stance_eq[side]] = int(phase0 < gait.duty_factor)
        model.site_pos[anchor_site[side], :] = (
            foot_target_x[side], 0.075 if side == "L" else -0.075, 0.04
        )
    mujoco.mj_forward(model, data)

    sim_viewer = viewer.launch_passive(model, data) if visualize else None
    render_period = 0.02  # approximately 50 rendered frames per second
    next_render_time = time.perf_counter()
    if sim_viewer is not None:
        sim_viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        sim_viewer.cam.trackbodyid = trunk_id
        sim_viewer.cam.distance = 2.5

    for step in range(nsteps):
        t = step * dt
        vx = data.qvel[dadr["root_x"]]
        speed_error = gait.speed - vx
        drive_integral = float(np.clip(drive_integral + speed_error * dt, -5.0, 5.0))
        drive = np.clip(
            gait.drive_kp * speed_error + gait.drive_ki * drive_integral,
            -gait.drive_force_limit,
            gait.drive_force_limit,
        )
        data.ctrl[aids["drive"]] = drive

        root_z = data.qpos[qadr["root_z"]]
        root_z_rate = data.qvel[dadr["root_z"]]
        zref = gait.height_amplitude * (
            1.0 - math.cos(2.0 * math.pi * t / gait.period)
        )
        data.ctrl[aids["vertical_support"]] = np.clip(
            gait.support_fraction * 10.66 * 9.81
            + gait.height_kp * (zref - root_z)
            - gait.height_kd * root_z_rate,
            -60.0,
            240.0,
        )

        pitch = data.qpos[qadr["root_pitch"]]
        pitch_rate = data.qvel[dadr["root_pitch"]]
        data.ctrl[aids["pitch_stabilizer"]] = np.clip(
            -gait.pitch_kp * pitch - gait.pitch_kd * pitch_rate,
            -120.0,
            120.0,
        )

        torso_x = float(data.xpos[trunk_id, 0])
        for side, offset, actuator in (
            ("L", 0.0, "hip_motor_L"),
            ("R", 0.5, "hip_motor_R"),
        ):
            joint = f"hip_{side}"
            q = data.qpos[qadr[joint]]
            qd = data.qvel[dadr[joint]]
            phase = gait_phase(t, gait, offset)
            if phase < previous_phase[side]:
                # Touchdown: place the foot ahead of the torso and hold its
                # world-frame target through stance to approximate no-slip support.
                foot_target_x[side] = float(data.geom_xpos[foot_geom_by_side[side], 0])
                model.site_pos[anchor_site[side], :] = (
                    foot_target_x[side], 0.075 if side == "L" else -0.075, 0.04
                )
            previous_phase[side] = phase
            data.eq_active[stance_eq[side]] = int(phase < gait.duty_factor)

            if phase < gait.duty_factor:
                leg_length = max(0.55, 0.74 + data.qpos[qadr[f"spring_{side}"]])
                offset_x = np.clip(foot_target_x[side] - torso_x, -0.45, 0.45)
                qref = -math.asin(float(offset_x / leg_length))
            else:
                qref = hip_reference(t, gait, offset)

            torque = np.clip(
                gait.hip_kp * (qref - q) - gait.hip_kd * qd,
                -30.0,
                30.0,
            )
            data.ctrl[aids[actuator]] = torque

            # In running, flex each knee during swing and extend it before touchdown.
            # Walking leaves the knee motors inactive.
            knee_name = f"knee_{side}"
            knee_motor = f"knee_motor_{side}"
            if gait.name == "run":
                knee_ref = running_knee_reference(phase, gait.duty_factor)
                knee_q = data.qpos[qadr[knee_name]]
                knee_qd = data.qvel[dadr[knee_name]]
                knee_torque = np.clip(
                    130.0 * (knee_ref - knee_q) - 5.0 * knee_qd,
                    -70.0,
                    70.0,
                )
                data.ctrl[aids[knee_motor]] = knee_torque
            else:
                data.ctrl[aids[knee_motor]] = 0.0

            spring_joint = f"spring_{side}"
            spring_motor = f"spring_reset_{side}"
            if phase >= gait.duty_factor:
                qspring = data.qpos[qadr[spring_joint]]
                vspring = data.qvel[dadr[spring_joint]]
                retract_force = np.clip(
                    gait.swing_retract_kp * (gait.swing_retract - qspring)
                    - gait.swing_retract_kd * vspring,
                    -180.0,
                    180.0,
                )
                data.ctrl[aids[spring_motor]] = retract_force
            else:
                data.ctrl[aids[spring_motor]] = 0.0

        # Motor work uses power=torque*joint speed; preserve positive and
        # negative work separately so regeneration assumptions remain explicit.
        for act_name, joint_name in (
            ("drive", "root_x"),
            ("vertical_support", "root_z"),
            ("pitch_stabilizer", "root_pitch"),
            ("hip_motor_L", "hip_L"),
            ("hip_motor_R", "hip_R"),
            ("knee_motor_L", "knee_L"),
            ("knee_motor_R", "knee_R"),
            ("spring_reset_L", "spring_L"),
            ("spring_reset_R", "spring_R"),
        ):
            power = data.ctrl[aids[act_name]] * data.qvel[dadr[joint_name]]
            positive_work += max(0.0, power) * dt
            negative_work += max(0.0, -power) * dt

        qL, qR = data.qpos[qadr["spring_L"]], data.qpos[qadr["spring_R"]]
        vL, vR = data.qvel[dadr["spring_L"]], data.qvel[dadr["spring_R"]]
        max_spring_deflection = max(max_spring_deflection, abs(qL), abs(qR))
        spring_energy = 0.5 * k * (qL * qL + qR * qR)
        change = spring_energy - previous_spring_energy
        spring_loaded += max(0.0, change)
        spring_released += max(0.0, -change)
        previous_spring_energy = spring_energy
        damping_loss += c * (vL * vL + vR * vR) * dt

        # Track passive torsional spring storage and damping in the knees/ankles.
        joint_spring_energy = 0.0
        for joint_name, (joint_stiffness, joint_damping) in PASSIVE_JOINT_SPRINGS.items():
            q = data.qpos[qadr[joint_name]]
            qd = data.qvel[dadr[joint_name]]
            joint_spring_energy += 0.5 * joint_stiffness * q * q
            joint_damping_loss += joint_damping * qd * qd * dt
            if joint_name.startswith("knee_"):
                max_knee_angle = max(max_knee_angle, abs(q))
            else:
                max_ankle_angle = max(max_ankle_angle, abs(q))
        joint_energy_change = joint_spring_energy - previous_joint_spring_energy
        joint_spring_loaded += max(0.0, joint_energy_change)
        joint_spring_released += max(0.0, -joint_energy_change)
        previous_joint_spring_energy = joint_spring_energy

        mujoco.mj_step(model, data)
        if sim_viewer is not None and step % 10 == 0:
            if not sim_viewer.is_running():
                break
            sim_viewer.sync()
            next_render_time += render_period
            delay = next_render_time - time.perf_counter()
            if delay > 0.0:
                time.sleep(delay)
            else:
                next_render_time = time.perf_counter()

        torso_z = data.xpos[trunk_id, 2]
        pitch = data.qpos[qadr["root_pitch"]]
        vx = data.qvel[dadr["root_x"]]
        max_height_error = max(max_height_error, abs(torso_z - 0.82))
        if t >= 1.0:  # exclude startup transient from steady-speed tracking metric
            speed_sq_error += (vx - gait.speed) ** 2
            speed_error_samples += 1

        contacting_feet = set()
        for ci in range(data.ncon):
            con = data.contact[ci]
            if con.geom1 == floor_geom and con.geom2 in foot_geoms:
                contacting_feet.add(con.geom2)
            elif con.geom2 == floor_geom and con.geom1 in foot_geoms:
                contacting_feet.add(con.geom1)
        if len(contacting_feet) == 0:
            flight_steps += 1
        elif len(contacting_feet) == 1:
            single_support_steps += 1
        else:
            double_support_steps += 1

        if (not np.isfinite(data.qpos).all() or torso_z < 0.40 or abs(pitch) > 0.9
                or max_spring_deflection >= 0.195):
            stable = False
            break

        # Save at 50 Hz, enough for plotting and report figures.
        if step % 10 == 0:
            rows.append(
                (
                    t,
                    data.qpos[qadr["root_x"]],
                    torso_z,
                    vx,
                    pitch,
                    qL,
                    qR,
                    spring_energy,
                    joint_spring_energy,
                    data.qpos[qadr["knee_L"]],
                    data.qpos[qadr["knee_R"]],
                    data.qpos[qadr["ankle_L"]],
                    data.qpos[qadr["ankle_R"]],
                    drive,
                )
            )

    # Keep the final pose visible until the user closes the viewer window.
    if sim_viewer is not None:
        if sim_viewer.is_running():
            sim_viewer.sync()
            while sim_viewer.is_running():
                time.sleep(0.02)
        sim_viewer.close()

    elapsed = data.time
    distance = max(0.0, data.qpos[qadr["root_x"]] - start_x)
    mass = 10.66
    cost_of_transport = positive_work / max(mass * 9.81 * distance, 1e-6)
    return_fraction = spring_released / max(spring_loaded, 1e-9)
    result = {
        "gait": gait.name,
        "target_speed_m_s": gait.speed,
        "distance_m": distance,
        "duration_s": elapsed,
        "stable": stable,
        "dimensionless_stiffness": params.dimensionless_stiffness,
        "damping_ratio": params.damping_ratio,
        "spring_stiffness_N_m": k,
        "spring_damping_Ns_m": c,
        "knee_spring_stiffness_Nm_rad": 60.0,
        "knee_spring_damping_Nms_rad": 2.5,
        "ankle_spring_stiffness_Nm_rad": 8.0,
        "ankle_spring_damping_Nms_rad": 0.5,
        "positive_actuator_work_J": positive_work,
        "negative_actuator_work_J": negative_work,
        "cost_of_transport": cost_of_transport,
        "spring_energy_loaded_J": spring_loaded,
        "spring_energy_released_J": spring_released,
        "joint_spring_energy_loaded_J": joint_spring_loaded,
        "joint_spring_energy_released_J": joint_spring_released,
        "total_spring_energy_loaded_J": spring_loaded + joint_spring_loaded,
        "total_spring_energy_released_J": spring_released + joint_spring_released,
        "spring_return_ratio": return_fraction,
        "total_spring_return_ratio": (spring_released + joint_spring_released)
        / max(spring_loaded + joint_spring_loaded, 1e-9),
        "damper_dissipation_J": damping_loss,
        "joint_damper_dissipation_J": joint_damping_loss,
        "total_damper_dissipation_J": damping_loss + joint_damping_loss,
        "max_abs_knee_angle_rad": max_knee_angle,
        "max_abs_ankle_angle_rad": max_ankle_angle,
        "max_com_height_deviation_m": max_height_error,
        "max_spring_deflection_m": max_spring_deflection,
        "rms_speed_error_m_s": math.sqrt(speed_sq_error / max(1, speed_error_samples)),
        "steady_rms_speed_error_m_s": math.sqrt(speed_sq_error / max(1, speed_error_samples)),
        "single_support_fraction": single_support_steps / max(1, step + 1),
        "double_support_fraction": double_support_steps / max(1, step + 1),
        "flight_fraction": flight_steps / max(1, step + 1),
        "final_com_height_m": float(data.xpos[trunk_id, 2]),
    }
    if output_csv:
        output_csv.parent.mkdir(parents=True, exist_ok=True)
        with output_csv.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                (
                    "time_s",
                    "x_m",
                    "com_height_m",
                    "speed_m_s",
                    "pitch_rad",
                    "left_spring_deflection_m",
                    "right_spring_deflection_m",
                    "leg_elastic_energy_J",
                    "joint_elastic_energy_J",
                    "left_knee_angle_rad",
                    "right_knee_angle_rad",
                    "left_ankle_angle_rad",
                    "right_ankle_angle_rad",
                    "drive_force_N",
                )
            )
            writer.writerows(rows)
    return result


def result_to_csv(results, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(results[0])
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(results)
