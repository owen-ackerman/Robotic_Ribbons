"""
Script CHOP Callbacks - Spherical Coordinate Robot Arm
Theta: azimuth in XZ plane, wraps 0->360, output as velocity
Phi:   elevation from Y+ axis [-135, 135], direct servo angle
       tracked as continuous delta to avoid redundant state flips
       soft limits at hardware bounds and speed-dependent ribbon cone
"""

from typing import Any
import numpy as np

PHI_MIN = -135.0
PHI_MAX =  135.0

def onSetupParameters(scriptOp: scriptCHOP):
    page = scriptOp.appendCustomPage('Arm')
    for n, l, d, mn, mx in [
        ('Theta',           'Theta (deg)',              90.0,   0.0, 360.0),
        ('Phi',             'Phi (deg)',                90.0, -135.0, 135.0),
        ('Rotaxisx',        'Rot Axis X',               0.0,  -1.0,   1.0),
        ('Rotaxisy',        'Rot Axis Y',               1.0,  -1.0,   1.0),
        ('Rotaxisz',        'Rot Axis Z',               0.0,  -1.0,   1.0),
        ('Rotspeed',        'Rot Speed (rad/s)',         1.0, -10.0,  10.0),
        ('Philimitsmooth',  'Phi Limit Smooth (s)',      0.3,   0.0,   1.0),
        ('Coneanglemax',    'Ribbon Cone Angle (rad)',  0.26,   0.0,  1.57),
        ('Conespeedthresh', 'Cone Speed Thresh (rad/s)', 1.57,  0.0,  12.57),
        ('Lasttime',        'Last Time',                0.0,   0.0,   0.0),
        ('Delta',           'Delta (s)',                0.0,   0.0,   0.0),
        ('Fps',             'FPS',                      0.0,   0.0,   0.0),
    ]:
        p = page.appendFloat(n, label=l)
        p[0].default = d
        p[0].min = mn
        p[0].max = mx
    return

def sph_to_xyz(theta_deg, phi_deg):
    t = np.radians(theta_deg)
    p = np.radians(phi_deg)
    return np.array([
        np.sin(p) * np.sin(t),
        np.cos(p),
        np.sin(p) * np.cos(t)
    ])

def xyz_to_dtheta(v_old, v_new):
    xz_old = np.array([v_old[0], v_old[2]])
    xz_new = np.array([v_new[0], v_new[2]])
    n_old  = np.linalg.norm(xz_old)
    n_new  = np.linalg.norm(xz_new)
    if n_old < 1e-4 or n_new < 1e-4:
        return 0.0
    xz_old /= n_old
    xz_new /= n_new
    cos_dt = np.clip(np.dot(xz_old, xz_new), -1.0, 1.0)
    sin_dt = xz_old[0]*xz_new[1] - xz_old[1]*xz_new[0]
    return float(np.degrees(np.arctan2(sin_dt, cos_dt)))

def xyz_to_dphi(v_old, v_new):
    xz = np.array([v_old[0], 0.0, v_old[2]])
    n  = np.linalg.norm(xz)
    if n < 1e-4:
        xz = np.array([0.0, 0.0, 1.0])
    else:
        xz /= n
    y_axis = np.array([0.0, 1.0, 0.0])
    def proj(v):
        return np.array([np.dot(v, xz), np.dot(v, y_axis)])
    p_old = proj(v_old);  n_old = np.linalg.norm(p_old)
    p_new = proj(v_new);  n_new = np.linalg.norm(p_new)
    if n_old < 1e-4 or n_new < 1e-4:
        return 0.0
    p_old /= n_old
    p_new /= n_new
    cos_dp = np.clip(np.dot(p_old, p_new), -1.0, 1.0)
    sin_dp = p_old[0]*p_new[1] - p_old[1]*p_new[0]
    return float(np.degrees(np.arctan2(sin_dp, cos_dp)))

def dynamic_cone_min(theta_vel, phi_vel, cone_angle_rad, speed_threshold_rad):
    """
    Dynamic minimum phi based on current speed.
    Slow speed -> cone_angle_rad enforced (ribbon protection).
    Fast speed -> cone shrinks to 0 (arm can pass through).
    """
    speed_rad = np.radians(np.sqrt(theta_vel**2 + phi_vel**2))
    t         = np.clip(speed_rad / max(speed_threshold_rad, 1e-4), 0.0, 1.0)
    return float(np.degrees(cone_angle_rad) * (1.0 - t))

def soft_limit(phi_prev, dphi, phi_min, phi_max, smooth_secs):
    """
    Scale dphi down as phi approaches either limit.
    smooth_secs=0 -> hard clip.
    smooth_secs=1 -> zone covers full half-range.
    Quadratic falloff for smooth onset at zone boundary.
    """
    if smooth_secs < 1e-4:
        return float(np.clip(phi_prev + dphi, phi_min, phi_max))
    half_range = (phi_max - phi_min) * 0.5
    zone       = smooth_secs * half_range
    if dphi < 0:
        dist  = np.clip((phi_prev - phi_min) / zone, 0.0, 1.0)
        scale = dist * dist
    else:
        dist  = np.clip((phi_max - phi_prev) / zone, 0.0, 1.0)
        scale = dist * dist
    return float(np.clip(phi_prev + dphi * scale, phi_min, phi_max))

def rotate(v, axis, angle_rad):
    ax = np.asarray(axis, float)
    n  = np.linalg.norm(ax)
    if n < 1e-9:
        return v.copy()
    ax /= n
    return (v * np.cos(angle_rad)
            + np.cross(ax, v) * np.sin(angle_rad)
            + ax * np.dot(ax, v) * (1 - np.cos(angle_rad)))

def onPulse(par: Any):
    return

def onCook(scriptOp: scriptCHOP):

    # ── Read rotation axis from external CHOP ─────────────────────
    try:
        scriptOp.par.Rotaxisx = op('null_rot_axis')['x'][0]
        scriptOp.par.Rotaxisy = op('null_rot_axis')['y'][0]
        scriptOp.par.Rotaxisz = op('null_rot_axis')['z'][0]
    except:
        pass

    # ── Timing ────────────────────────────────────────────────────
    now   = absTime.seconds
    delta = min(now - scriptOp.par.Lasttime.val, 0.1)
    scriptOp.par.Lasttime = now
    scriptOp.par.Delta    = delta
    scriptOp.par.Fps      = 1.0 / delta if delta > 0 else 0.0

    # ── Read current state ────────────────────────────────────────
    theta_prev      = scriptOp.par.Theta.val
    phi_prev        = scriptOp.par.Phi.val
    axis            = [scriptOp.par.Rotaxisx.val,
                       scriptOp.par.Rotaxisy.val,
                       scriptOp.par.Rotaxisz.val]
    speed           = scriptOp.par.Rotspeed.val
    smooth_secs     = scriptOp.par.Philimitsmooth.val
    cone_angle      = scriptOp.par.Coneanglemax.val
    speed_threshold = scriptOp.par.Conespeedthresh.val

    # ── Rotate ────────────────────────────────────────────────────
    v     = sph_to_xyz(theta_prev, phi_prev)
    v_new = rotate(v, axis, speed * delta)
    v_new = v_new / np.linalg.norm(v_new)

    # ── Accumulate deltas ─────────────────────────────────────────
    dtheta = xyz_to_dtheta(v, v_new)
    dphi   = xyz_to_dphi(v, v_new)

    # ── Velocities (deg/sec) ──────────────────────────────────────
    theta_vel = dtheta / delta if delta > 1e-6 else 0.0
    phi_vel   = dphi   / delta if delta > 1e-6 else 0.0

    # ── Dynamic cone min phi (ribbon protection) ──────────────────
    phi_min_dyn = dynamic_cone_min(theta_vel, phi_vel, cone_angle, speed_threshold)

    # ── Apply soft limits — cone floor first, then hardware floor ─
    # use the larger (more restrictive) of the two minimums
    phi_min_eff = max(PHI_MIN, phi_min_dyn)
    phi_new     = soft_limit(phi_prev, dphi, phi_min_eff, PHI_MAX, smooth_secs)

    theta_new = (theta_prev + dtheta) % 360

    # ── Store state ───────────────────────────────────────────────
    scriptOp.par.Theta = theta_new
    scriptOp.par.Phi   = phi_new

    # ── Output channels ───────────────────────────────────────────
    scriptOp.clear()
    scriptOp.appendChan('theta')[0]          = float(theta_new)
    scriptOp.appendChan('phi')[0]            = float(phi_new)
    scriptOp.appendChan('theta_velocity')[0] = float(theta_vel)
    scriptOp.appendChan('phi_servo')[0]      = float(phi_new)
    scriptOp.appendChan('phi_min_dyn')[0]    = float(phi_min_dyn)
    scriptOp.appendChan('x')[0]              = float(v_new[0])
    scriptOp.appendChan('y')[0]              = float(v_new[1])
    scriptOp.appendChan('z')[0]              = float(v_new[2])
    scriptOp.appendChan('cone_height')[0] = float(np.cos(np.radians(phi_min_dyn)))
    scriptOp.appendChan('cone_radius')[0] = float(np.sin(np.radians(phi_min_dyn)))
    return

def onGetCookLevel(scriptOp: scriptCHOP) -> CookLevel:
    return CookLevel.ON_CHANGE