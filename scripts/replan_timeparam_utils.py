#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

# =========================
# Config
# =========================
MASTER_PATH = '/home/tp2/ws_humble/MTC_connect_visualization/leader_no_matching_tcp_001.txt'
SLAVE_PATH  = '/home/tp2/ws_humble/MTC_connect_visualization/follower_no_matching_tcp_001.txt'
OUT_FILE_MASTER = Path(MASTER_PATH).with_name(Path(MASTER_PATH).stem + "_reparam.txt")
OUT_FILE_SLAVE  = Path(SLAVE_PATH).with_name(Path(SLAVE_PATH).stem + "_reparam.txt")

# 自适应权重（基于曲率；用于急弯放大正交项）
ADAPT_R_LO   = 0.18  # m
ADAPT_R_HI   = 0.03  # m
ADAPT_GAIN   = 4.0   # W_O 最大增益

# 曲率下限约束（仅位置）
R_MIN = ADAPT_R_HI  # m

# 轨迹加密（Catmull–Rom + SLERP）
DENSIFY_MAX_POS_STEP = 0.002  # m
DENSIFY_MAX_ANG_DEG  = 1.0    # deg
DENSIFY_MAX_DT       = None
CR_ALPHA             = 0.5

# 匹配代价（(p_m - p_s) ⟂ y,z ⇒ 连线尽量沿 EE-x）
ORTHO_AXES   = ('y','z')
D_TARGET     = 0.12   # m
Y_COMP_SLAVE = 0.562  # 仅用于匹配（不回写文件）
W_D, W_O     = 6.0, 1.0
TOL_D, TOL_PERP = 0.002, 0.001

# 搜索约束
NO_BACKTRACK          = True
PRESERVE_MASTER_KNOTS = True
MAX_LOOKAHEAD_SEG     = 3
ANCHOR_SNAP_U         = 0.03
SNAP_FLAT_GAIN        = 1.5
LOOKAHEAD_OK_RATIO    = 1.12

# 鲁棒曲率
CURV_FIT_WIN     = 9
CURV_MIN_POINTS  = 5

# —— 急弯回填设置（借位 + 密集/重复过滤） —— #
REINSTATE_R_THRESH     = (ADAPT_R_HI * 2)   # m
REINSTATE_MIN_SAMPLES  = 15
REINSTATE_BORROW_MAX   = 40
REINSTATE_MIN_ARC      = max(3*DENSIFY_MAX_POS_STEP, 0.012)
REINSTATE_DUP_STEP_EPS = 1e-3
REINSTATE_DUP_FRAC     = 0.5
REINSTATE_VERBOSE      = True

# —— 异常步长均衡 —— #
EVEN_MAX_STEP = 0.005   # m（下限）
EVEN_CONTEXT  = 5
EVEN_ITERS    = 2

# =========================
# I/O
# =========================
def parse_T_colmajor(flat16):
    v = np.asarray(flat16, float); assert v.size == 16
    return v.reshape(4, 4, order='F')

def load_path_txt(path):
    # each row: x y z qx qy qz qw
    cols = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
    df = pd.read_csv(path, delim_whitespace=True, header=None, names=cols)
    T = np.stack([df[col].to_numpy() for col in cols], axis=1)
    return T

def load_traj_txt(path):
    # return time and trajectory
    cols = ['t'] + [f'c{i}' for i in range(16)]
    df = pd.read_csv(path, delim_whitespace=True, header=None, names=cols)
    t = df['t'].to_numpy()
    T = np.stack([parse_T_colmajor(df.loc[i, [f'c{j}' for j in range(16)]].values)
                  for i in range(len(df))], axis=0)
    return t, T

def save_traj_colmajor(path, t, T_seq):
    assert len(t) == len(T_seq)
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w') as f:
        for ti, Ti in zip(t, T_seq):
            flat16 = Ti.reshape(16, order='F')
            f.write(' '.join(f'{x:.10g}' for x in (ti, *flat16)) + '\n')
    print(f"[OK] Saved {len(t)} rows -> {path}")

# =========================
# SO(3) & quaternion
# =========================
def R_to_quat(R):
    m, t = R, np.trace(R)
    if t > 0:
        s = np.sqrt(t+1.0)*2; w=0.25*s
        x=(m[2,1]-m[1,2])/s; y=(m[0,2]-m[2,0])/s; z=(m[1,0]-m[0,1])/s
    else:
        i = np.argmax([m[0,0], m[1,1], m[2,2]])
        if i == 0:
            s=np.sqrt(1.0+m[0,0]-m[1,1]-m[2,2])*2; w=(m[2,1]-m[1,2])/s; x=0.25*s
            y=(m[0,1]+m[1,0])/s; z=(m[0,2]+m[2,0])/s
        elif i == 1:
            s=np.sqrt(1.0+m[1,1]-m[0,0]-m[2,2])*2; w=(m[0,2]-m[2,0])/s; x=(m[0,1]+m[1,0])/s
            y=0.25*s; z=(m[1,2]+m[2,1])/s
        else:
            s=np.sqrt(1.0+m[2,2]-m[0,0]-m[1,1])*2; w=(m[1,0]-m[0,1])/s; x=(m[0,2]+m[2,0])/s
            y=(m[1,2]+m[2,1])/s; z=0.25*s
    q = np.array([w,x,y,z], float); q /= (np.linalg.norm(q)+1e-12); return q

def quat_to_R(q):
    w,x,y,z = q; xx,yy,zz=x*x,y*y,z*z; wx,wy,wz=w*x,w*y,w*z; xy,xz,yz=x*y,x*z,y*z
    return np.array([[1-2*(yy+zz), 2*(xy-wz),    2*(xz+wy)],
                     [2*(xy+wz),   1-2*(xx+zz), 2*(yz-wx)],
                     [2*(xz-wy),   2*(yz+wx),   1-2*(xx+yy)]])

def quat_slerp(q0, q1, u, eps=1e-9):
    if np.dot(q0, q1) < 0: q1 = -q1
    c = np.clip(np.dot(q0, q1), -1.0, 1.0)
    if 1.0 - c < eps:
        q = (1-u)*q0 + u*q1
        return q/(np.linalg.norm(q)+1e-12)
    ang = np.arccos(c)
    s0 = np.sin((1-u)*ang)/np.sin(ang); s1 = np.sin(u*ang)/np.sin(ang)
    q = s0*q0 + s1*q1
    return q/(np.linalg.norm(q)+1e-12)

import numpy as np
from scipy.spatial.transform import Rotation

def _unit(vector, eps=1e-12):
    vector = np.asarray(vector, float)
    norm = float(np.linalg.norm(vector))
    return (vector / norm if norm > eps else vector), norm

def _axis_from_local_name(name):
    return {'x': np.array([1.0,0.0,0.0]),
            'y': np.array([0.0,1.0,0.0]),
            'z': np.array([0.0,0.0,1.0])}[name.lower()]

def _rotation_aligning_vector_a_to_b(vector_a, vector_b, eps=1e-12):
    """Minimal rotation that maps vector_a -> vector_b (both 3D vectors)."""
    a_hat, _ = _unit(vector_a, eps)
    b_hat, _ = _unit(vector_b, eps)
    c = float(np.clip(np.dot(a_hat, b_hat), -1.0, 1.0))
    if 1.0 - c < eps:
        return np.eye(3)  # already aligned
    if 1.0 + c < eps:
        # opposite: 180° about any axis ⟂ a_hat (pick a stable one)
        pick = np.array([1.0, 0.0, 0.0]) if abs(a_hat[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        axis = np.cross(a_hat, pick); axis, _ = _unit(axis)
        return Rotation.from_rotvec(np.pi * axis).as_matrix()
    axis = np.cross(a_hat, b_hat)
    s = float(np.linalg.norm(axis))
    axis = axis / s
    angle = np.arctan2(s, c)
    return Rotation.from_rotvec(angle * axis).as_matrix()

def decompose_rotation_about_local_axis(R_end_effector, local_axis='y'):
    """
    Decompose a single EE rotation into:
        R_end_effector = R_without_local_spin @ R_only_local_spin
    where R_only_local_spin is a pure rotation about the EE's *local* axis.
    """
    if isinstance(local_axis, str):
        local_axis_vec = _axis_from_local_name(local_axis)
    else:
        local_axis_vec = np.asarray(local_axis, float)
        if local_axis_vec.shape != (3,):
            raise ValueError("local_axis vector must be length-3")

    # Final world direction of that local axis
    world_axis_final = R_end_effector @ local_axis_vec

    # Swing: minimal rotation taking the local axis to its final world direction
    R_swing = _rotation_aligning_vector_a_to_b(local_axis_vec, world_axis_final)

    # Residual is the twist about the *local* axis (right factor)
    R_only_local_spin = R_swing.T @ R_end_effector

    # Remove that spin to get the "no-spin" version
    R_without_local_spin = R_end_effector @ R_only_local_spin.T
    return R_without_local_spin, R_only_local_spin

def remove_rotation_about_local_axis(R_end_effector,
                                     local_axis='y',
                                     reference_world_vector=None,
                                     reference_local_axis='x',
                                     use_unsigned=False):
    """
    Remove spin about the EE's *local* axis. If a reference vector is provided,
    choose the 180° branch whose resulting world direction of `reference_local_axis`
    is closer to that reference.

    Args:
        R_end_effector (3x3): rotation matrix of the EE in the base/world frame.
        local_axis: 'x'|'y'|'z' or 3-vector in the EE *local* frame (axis whose spin is removed).
        reference_world_vector: optional 3-vector in world frame. If provided, the function
            evaluates both branches and returns the one whose resulting
            (R_no_spin @ reference_local_axis) is closer (larger dot) to this vector.
        reference_local_axis: which EE local axis to compare to the reference ('x'|'y'|'z' or 3-vector).
        use_unsigned: if True, use |dot| (ignore sign) when comparing closeness.

    Returns:
        R_without_local_spin (3x3): rotation with spin about `local_axis` removed, branch chosen by rule above.
    """
    # First, do the natural decomposition (no manual flip yet)
    R_without_local_spin, _ = decompose_rotation_about_local_axis(
        R_end_effector=R_end_effector, local_axis=local_axis
    )

    # If no reference provided, return the default branch
    if reference_world_vector is None:
        return R_without_local_spin

    # Build the "flipped" alternative branch: multiply by 180° around the LOCAL axis
    if isinstance(local_axis, str):
        local_axis_vec = _axis_from_local_name(local_axis)
    else:
        local_axis_vec = np.asarray(local_axis, float)
    R_local_pi = Rotation.from_rotvec(np.pi * local_axis_vec).as_matrix()

    R_without_local_spin_alt = R_without_local_spin @ R_local_pi

    # Which local axis should be compared to the world reference?
    if isinstance(reference_local_axis, str):
        reference_local_vec = _axis_from_local_name(reference_local_axis)
    else:
        reference_local_vec = np.asarray(reference_local_axis, float)

    # Normalize the world reference
    reference_world_vec_unit, ref_norm = _unit(reference_world_vector)
    if ref_norm == 0.0:
        # Degenerate reference; fall back to default
        return R_without_local_spin

    # Compute the world directions of the chosen local axis after each candidate
    world_direction_default = R_without_local_spin @ reference_local_vec
    world_direction_flipped = R_without_local_spin_alt @ reference_local_vec
    world_direction_default_unit, _ = _unit(world_direction_default)
    world_direction_flipped_unit, _ = _unit(world_direction_flipped)

    # Compare closeness via dot product (optionally unsigned)
    dot_default = float(np.dot(world_direction_default_unit, reference_world_vec_unit))
    dot_flipped = float(np.dot(world_direction_flipped_unit, reference_world_vec_unit))
    if use_unsigned:
        dot_default = abs(dot_default)
        dot_flipped = abs(dot_flipped)

    # Pick the branch whose axis is closer to the reference
    return R_without_local_spin if dot_default >= dot_flipped else R_without_local_spin_alt

# =========================
# 几何 & 曲率
# =========================
def _circumradius(A,B,C,eps=1e-12):
    a=np.linalg.norm(B-C); b=np.linalg.norm(C-A); c=np.linalg.norm(A-B)
    area2=np.linalg.norm(np.cross(B-A, C-A))
    return np.inf if area2<eps else (a*b*c)/(2.0*area2)

def enforce_min_radius_positions(T_in, R_min, iters=60, alpha=0.6, fix=True, beta_max=0.5):
    T=T_in.copy(); P=T[:, :3, 3]; N=len(P); i0=1 if fix else 0; i1=N-1 if fix else N
    for _ in range(iters):
        changed=0; Pn=P.copy()
        for i in range(i0, i1):
            R=_circumradius(P[i-1], P[i], P[i+1])
            if R < R_min:
                beta=min(beta_max, alpha*(R_min-R)/max(R_min,1e-9))
                Pn[i] = P[i] + beta*(0.5*(P[i-1]+P[i+1]) - P[i]); changed+=1
        P[:]=Pn
        if changed==0: break
    T[:, :3, 3]=P; return T

def _pca_plane(Pw):
    C = Pw.mean(axis=0); X = Pw - C
    U,S,Vt = np.linalg.svd(X, full_matrices=False)
    n = Vt[2]; u = Vt[0]; v = Vt[1]
    return C, u, v, n

def _fit_circle_2d(x, y):
    A = np.stack([x, y, np.ones_like(x)], axis=1)
    b = x*x + y*y
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    a, b2, c = sol
    cx, cy = a/2.0, b2/2.0
    r2 = cx*cx + cy*cy + c
    if r2 <= 0: return None, None, np.inf
    return cx, cy, float(np.sqrt(r2))

def robust_radius_series(P, win=9, min_pts=5):
    N = len(P); R = np.full(N, np.inf, float)
    w = max(3, int(win)); w += (w % 2 == 0)
    half = w // 2
    for i in range(N):
        L = max(0, i - half); Rr = min(N, i + half + 1)
        if Rr - L < min_pts: continue
        Pw = P[L:Rr]
        C,u,v,_ = _pca_plane(Pw)
        x = np.array([ (p - C) @ u for p in Pw ])
        y = np.array([ (p - C) @ v for p in Pw ])
        _, _, r = _fit_circle_2d(x, y)
        if np.isfinite(r):
            R[i] = r
        else:
            i0 = max(L, i-1); i1 = i; i2 = min(Rr-1, i+1)
            R[i] = _circumradius(P[i0], P[i1], P[i2])
    if N >= 2:
        R[0] = R[1]; R[-1] = R[-2]
    return R

def robust_curvature(P, win=9, min_pts=5):
    R = robust_radius_series(P, win, min_pts)
    k = np.zeros_like(R)
    finite = R < np.inf
    k[finite] = 1.0 / np.maximum(R[finite], 1e-12)
    k[~finite] = 0.0
    return k, R

def _smoothstep(e0, e1, x):
    t=np.clip((x-e0)/max(e1-e0,1e-12), 0.0, 1.0)
    return t*t*(3-2*t)

# =========================
# 误差/轴
# =========================
def _axis_indices(axes):
    if isinstance(axes, (list, tuple, set)):
        return [ {'x':0,'y':1,'z':2}[a] for a in axes ]
    return [ {'x':0,'y':1,'z':2}[axes] ]

def _orth_err_agg(diff, R, axes):
    idxs = _axis_indices(axes)
    return float(np.sqrt(sum((float(np.dot(diff, R[:,i])))**2 for i in idxs)))

# =====================================
# Pad trajectories to the same lengths
# =====================================

def pad_trajectories(t1, T1, t2, T2):
    """
    Pads the shorter sequence to the length of the longer one by repeating
    the last sample (timestamp and 4x4 pose). Returns (t1p, T1p, t2p, T2p).
    """
    t1 = np.asarray(t1); T1 = np.asarray(T1)
    t2 = np.asarray(t2); T2 = np.asarray(T2)
    n1, n2 = len(t1), len(t2)
    if n1 == 0 or n2 == 0:
        raise ValueError("Cannot pad: one of the trajectories is empty.")

    if n1 == n2:
        return t1, T1, t2, T2

    if n1 < n2:
        pad_n = n2 - n1
        t1_pad = np.repeat(t1[-1], pad_n)
        T1_pad = np.repeat(T1[-1][None, ...], pad_n, axis=0)
        return np.concatenate([t1, t1_pad]), np.concatenate([T1, T1_pad], axis=0), t2, T2
    else:
        pad_n = n1 - n2
        t2_pad = np.repeat(t2[-1], pad_n)
        T2_pad = np.repeat(T2[-1][None, ...], pad_n, axis=0)
        return t1, T1, np.concatenate([t2, t2_pad]), np.concatenate([T2, T2_pad], axis=0)

# =========================
# Catmull–Rom + SLERP（加密）
# =========================
def chord_params(P, alpha=0.5):
    t=np.zeros(len(P))
    for i in range(1, len(P)):
        d=max(np.linalg.norm(P[i]-P[i-1]), 1e-12); t[i]=t[i-1]+d**alpha
    return t

def _pad_pts(P, t):
    dt0=max(t[1]-t[0], 1.0); dtn=max(t[-1]-t[-2], 1.0)
    return np.vstack([P[0],P,P[-1]]), np.concatenate([[t[0]-dt0], t, [t[-1]+dtn]])

def cr_eval(P_ext, t_ext, i, u01):
    P0,P1,P2,P3 = P_ext[i:i+4]; t0,t1,t2,t3 = t_ext[i:i+4]
    tau=(1-u01)*t1 + u01*t2
    def lerp(A,B,ta,tb):
        return A if abs(tb-ta)<1e-12 else (1-(tau-ta)/(tb-ta))*A + (tau-ta)/(tb-ta)*B
    A1=lerp(P0,P1,t0,t1); A2=lerp(P1,P2,t1,t2); A3=lerp(P2,P3,t2,t3)
    B1=lerp(A1,A2,t0,t2); B2=lerp(A2,A3,t1,t3)
    return lerp(B1,B2,t1,t2)

def densify_trajectory(t_in, T_in,
                       max_pos_step=DENSIFY_MAX_POS_STEP,
                       max_ang_deg=DENSIFY_MAX_ANG_DEG,
                       max_dt=DENSIFY_MAX_DT,
                       cr_alpha=CR_ALPHA):
    t_in = np.asarray(t_in); T_in=np.asarray(T_in); N=len(t_in); assert N>=2 and N==len(T_in)
    Pk, Rk = T_in[:, :3, 3], T_in[:, :3, :3]
    Qk = np.array([R_to_quat(Rk[i]) for i in range(N)])

    t_cr = chord_params(Pk, cr_alpha); P_ext,t_ext = _pad_pts(Pk, t_cr)
    p_eval = lambda i,u: cr_eval(P_ext, t_ext, i, u)

    t_out=[float(t_in[0])]; T_out=[T_in[0].copy()]
    for i in range(N-1):
        p0,p1 = Pk[i], Pk[i+1]; q0,q1 = Qk[i], Qk[i+1]; dt=float(t_in[i+1]-t_in[i])
        dist=float(np.linalg.norm(p1-p0))
        c = np.clip(abs(np.dot(q0, q1)), -1.0, 1.0)
        ang=float(np.degrees(2*np.arccos(c)))
        n_pos = max(0, int(np.ceil(dist/max_pos_step))-1) if max_pos_step else 0
        n_ang = max(0, int(np.ceil(ang/max_ang_deg))-1)  if max_ang_deg  else 0
        n_dt  = max(0, int(np.ceil(dt/max_dt))-1)        if max_dt      else 0
        n_ins = max(n_pos, n_ang, n_dt)

        for j in range(1, n_ins+1):
            u = j/(n_ins+1); p=p_eval(i,u); q=quat_slerp(q0,q1,u)
            T=np.eye(4); T[:3,:3]=quat_to_R(q); T[:3,3]=p
            t_out.append(float(t_in[i]+u*dt)); T_out.append(T)

        T_end=np.eye(4); T_end[:3,:3]=Rk[i+1]; T_end[:3,3]=p1
        t_out.append(float(t_in[i+1])); T_out.append(T_end)

    t_arr=np.array(t_out); T_arr=np.array(T_out)
    keep=np.ones(len(t_arr), bool); keep[1:]=np.abs(np.diff(t_arr))>1e-12
    return t_arr[keep], T_arr[keep]

# =========================
# 时间重采样（共同等间隔网格）
# =========================
def robust_dt(t_arr: np.ndarray):
    t_arr = np.asarray(t_arr, float)
    if len(t_arr) < 2: return 0.01
    d = np.diff(t_arr); d = d[np.isfinite(d) & (d > 0)]
    return float(np.median(d)) if d.size else 0.01

def make_uniform_grid(t0: float, t1: float, dt: float):
    t0 = float(t0); t1 = float(t1); dt = max(float(dt), 1e-9)
    if t1 <= t0: return np.array([t0], float)
    n = int(np.floor((t1 - t0)/dt)) + 1
    dt_adj = (t1 - t0) / (n - 1) if n >= 2 else dt
    return t0 + np.arange(n, dtype=float) * dt_adj

def interp_pose_between(T0: np.ndarray, T1: np.ndarray, a: float):
    p0, p1 = T0[:3, 3], T1[:3, 3]
    q0, q1 = R_to_quat(T0[:3, :3]), R_to_quat(T1[:3, :3])
    p = (1.0 - a) * p0 + a * p1
    q = quat_slerp(q0, q1, float(a))
    Tout = np.eye(4); Tout[:3, :3] = quat_to_R(q); Tout[:3, 3] = p
    return Tout

def resample_by_time_grid(t_in: np.ndarray, T_in: np.ndarray, t_grid: np.ndarray):
    t_in = np.asarray(t_in, float); T_in = np.asarray(T_in, float)
    t_grid = np.asarray(t_grid, float)
    assert len(t_in) == len(T_in) and len(t_in) >= 1
    if len(t_in) == 1:
        return t_grid, np.repeat(T_in[[0]], len(t_grid), axis=0)

    out = []
    k = 0
    for tg in t_grid:
        while k+1 < len(t_in) and t_in[k+1] < tg:
            k += 1
        if k >= len(t_in) - 1:
            T = T_in[-1]
        else:
            t0, t1 = float(t_in[k]), float(t_in[k+1])
            a = 0.0 if t1 <= t0 else float((tg - t0) / (t1 - t0))
            a = 0.0 if tg <= t0 else (1.0 if tg >= t1 else a)
            T = _interp_pose_between(T_in[k], T_in[k+1], a)
        out.append(T)
    return t_grid, np.stack(out, axis=0)

# =========================
# 自适应权重（基于 slave 曲率）
# =========================
def make_weight_schedule_from_path(
    T_path, W_D0, W_O0,
    R_lo=0.12, R_hi=0.06,
    gain_max=6.0,
    dilate_pre=0.03, dilate_post=0.03, blur_sigma=0.02,
    curv_win=CURV_FIT_WIN, curv_min_pts=CURV_MIN_POINTS
):
    P = T_path[:, :3, 3]
    kappa, _ = robust_curvature(P, win=curv_win, min_pts=curv_min_pts)
    K_lo, K_hi = 1/max(R_lo,1e-9), 1/max(R_hi,1e-9)
    s = _smoothstep(K_lo, K_hi, kappa)

    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    s_cum = np.concatenate([[0.0], np.cumsum(seg)])
    core = (kappa >= K_hi).astype(float)

    def dilate(core, pre, post):
        out = np.zeros_like(core)
        idx = np.where(core>0.5)[0]
        for i in idx:
            L, R = s_cum[i]-pre, s_cum[i]+post
            li = np.searchsorted(s_cum, L, 'left')
            ri = np.searchsorted(s_cum, R, 'right')-1
            out[li:ri+1] = 1.0
        return out

    s = np.maximum(s, dilate(core, dilate_pre, dilate_post))

    if blur_sigma > 0 and len(seg) > 0:
        mean_step = max(seg.mean(), 1e-9)
        halfw = int(np.ceil(3*blur_sigma/mean_step))
        x = np.arange(-halfw, halfw+1)
        g = np.exp(-0.5*(x*mean_step/blur_sigma)**2); g/=g.sum()
        s = np.convolve(s, g, mode='same')

    w_o = W_O0 * (1.0 + (gain_max-1.0)*s)
    w_d = W_D0 * np.ones_like(s, float)
    return w_d, w_o

# =========================
# 丢段报告 + 急弯判定 + 回填
# =========================
def _compact_ranges(sorted_idx: np.ndarray):
    if len(sorted_idx) == 0: return []
    ranges = []; s = e = int(sorted_idx[0])
    for v in sorted_idx[1:]:
        v = int(v)
        if v == e + 1: e = v
        else: ranges.append((s, e)); s = e = v
    ranges.append((s, e))
    return ranges

def report_missing_segments(used_idx: np.ndarray, t_master: np.ndarray, T_master: np.ndarray):
    pM = T_master[:, :3, 3]
    total_segments = len(pM) - 1
    all_segments = np.arange(total_segments, dtype=int)
    used_idx = np.asarray(used_idx, dtype=int)
    used_idx = used_idx[(used_idx >= 0) & (used_idx < total_segments)]
    visited = np.unique(used_idx)
    missing = np.setdiff1d(all_segments, visited)
    miss_ranges = _compact_ranges(missing)

    print(f"[COVERAGE] visited={len(visited)}/{total_segments}, missing={len(missing)}")
    if len(miss_ranges) > 0:
        preview = ", ".join([f"{a}-{b}" if a != b else f"{a}" for (a, b) in miss_ranges[:8]])
        print(f"[COVERAGE] missing ranges: {preview}{' ...' if len(miss_ranges)>50 else ''}")
    return visited, missing, miss_ranges

def _cum_arclen(P):
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)])

def _subpath_steps(P, s, e):
    s0 = int(max(0, s))
    e1 = int(min(len(P)-1, e+1))
    if e1 <= s0:
        return np.array([]), 0.0, 1.0
    steps = np.linalg.norm(P[s0+1:e1+1] - P[s0:e1], axis=1)
    arc = float(np.sum(steps)) if steps.size else 0.0
    small_frac = float(np.mean(steps < REINSTATE_DUP_STEP_EPS)) if steps.size else 1.0
    return steps, arc, small_frac

def classify_sharp_ranges(miss_ranges, R_master, P_master):
    sharp_all = []
    sharp_clean = []
    for (s,e) in miss_ranges:
        i0, i1 = s, min(len(R_master)-1, e+1)
        minR = float(np.min(R_master[i0:i1+1]))
        if not (np.isfinite(minR) and minR <= REINSTATE_R_THRESH):
            continue
        sharp_all.append((s,e,minR))
        _, arc, smallf = _subpath_steps(P_master, s, e)
        dense_dup = (arc < REINSTATE_MIN_ARC) or (smallf >= REINSTATE_DUP_FRAC) or (minR <= 1e-9)
        if not dense_dup:
            sharp_clean.append((s,e,minR))

    if sharp_all:
        msg = ", ".join([f"{s}-{e}(Rmin={R:.3f}m)" for (s,e,R) in sharp_all[:20]])
        print(f"[SHARP] ranges (Rmin <= {REINSTATE_R_THRESH:.3f}m): {msg}{' ...' if len(sharp_all)>20 else ''}")
    else:
        print(f"[SHARP] no ranges satisfy Rmin <= {REINSTATE_R_THRESH:.3f}m")
    if sharp_clean:
        msg2 = ", ".join([f"{s}-{e}(Rmin={R:.3f}m)" for (s,e,R) in sharp_clean[:20]])
        print(f"[SHARP] clean after de-dup: {msg2}{' ...' if len(sharp_clean)>20 else ''}")
    else:
        print(f"[SHARP] no clean sharp ranges after de-dup.")
    return sharp_all, sharp_clean

def _sample_master_subpath(Tm, seg_s, seg_e, n_out):
    assert seg_s <= seg_e
    n_out = int(max(1, n_out))
    P = Tm[:, :3, 3]; Rm = Tm[:, :3, :3]
    knots = np.arange(seg_s, seg_e + 2)
    Pk = P[knots]; Rk = Rm[knots]
    s = _cum_arclen(Pk); L = float(max(s[-1], 1e-12))
    targets = np.linspace(0.0, L, n_out)
    out_T = []
    Qk = np.array([R_to_quat(R) for R in Rk], float)

    for tau in targets:
        j = int(np.searchsorted(s, tau, 'right') - 1)
        j = max(0, min(j, len(Pk) - 2))
        s0, s1 = float(s[j]), float(s[j+1])
        u = 0.0 if s1 <= s0 else float((tau - s0) / (s1 - s0))
        p = (1.0 - u) * Pk[j] + u * Pk[j+1]
        q = quat_slerp(Qk[j], Qk[j+1], u)
        T = np.eye(4); T[:3, :3] = quat_to_R(q); T[:3, 3] = p
        out_T.append(T)
    return np.stack(out_T, axis=0)

def reinstate_sharp_missing_paths_with_borrow(
    T_m_new, seg_used, T_m, t_s, sharp_ranges_clean
):
    K = len(t_s)
    seg_used = np.asarray(seg_used)
    Tout = T_m_new.copy()
    reinstated = []

    for (s, e, minR) in sharp_ranges_clean:
        ks = np.where(seg_used < s)[0]; kL = int(ks[-1]) if ks.size>0 else -1
        ks = np.where(seg_used > e)[0]; kR = int(ks[0]) if ks.size>0 else K

        a = max(0, kL + 1)
        b = min(K-1, kR - 1)
        width = b - a + 1 if b >= a else 0

        need = max(REINSTATE_MIN_SAMPLES - width, 0)
        capL = min(REINSTATE_BORROW_MAX, max(0, (kL if kL >= 0 else -1) - 0 + 1))
        capR = min(REINSTATE_BORROW_MAX, max(0, (K-1) - (kR if kR < K else K) + 1))

        if need > 0:
            borrowL = min(need//2 + need%2, capL)
            borrowR = min(need - borrowL, capR)
            if borrowL + borrowR < need:
                remain = need - (borrowL + borrowR)
                moreL = min(remain, capL - borrowL); borrowL += moreL; remain -= moreL
                moreR = min(remain, capR - borrowR); borrowR += moreR; remain -= moreR
            a = max(0, a - borrowL)
            b = min(K-1, b + borrowR)
            width = b - a + 1

        if width <= 0:
            if REINSTATE_VERBOSE:
                print(f"[REINSTATE] skip seg {s}-{e}: no capacity even after borrow (Rmin={minR:.3f}m)")
            continue

        subT = _sample_master_subpath(T_m, s, e, width)
        Tout[a:b+1] = subT

        reinstated.append((s, e, minR, a, b))
        if REINSTATE_VERBOSE:
            print(f"[REINSTATE] seg {s}-{e} | Rmin={minR:.3f}m | window=[{a}:{b}] (n={width})")

    if reinstated:
        msg = ", ".join([f"{s}-{e}(Rmin={r:.3f}m->[k{a}:{b}])" for (s,e,r,a,b) in reinstated[:10]])
        print(f"[REINSTATE] reinstated sharp ranges: {msg}{' ...' if len(reinstated)>10 else ''}")
    else:
        print("[REINSTATE] no sharp missing ranges reinstated.")
    return Tout, reinstated

# =========================
# 异常步长均衡
# =========================
def _step_stats(steps: np.ndarray):
    steps = np.asarray(steps, float)
    steps = steps[np.isfinite(steps)]
    if steps.size == 0:
        return dict(mean=np.nan, std=np.nan, median=np.nan, mad=np.nan, q1=np.nan, q3=np.nan, iqr=np.nan)
    mean = float(steps.mean())
    std  = float(steps.std(ddof=0))
    median = float(np.median(steps))
    mad = float(np.median(np.abs(steps - median)))
    q1, q3 = float(np.quantile(steps, 0.25)), float(np.quantile(steps, 0.75))
    iqr = q3 - q1
    return dict(mean=mean, std=std, median=median, mad=mad, q1=q1, q3=q3, iqr=iqr)

def detect_outlier_steps_gaussian(steps: np.ndarray, z: float = 3.0, min_abs: float = EVEN_MAX_STEP):
    steps = np.asarray(steps, float)
    stats = _step_stats(steps)
    mu, sigma = stats["mean"], stats["std"]
    # 高斯 3σ（或自定 z），并做下限保护
    if np.isfinite(mu) and np.isfinite(sigma):
        thr = float(mu + z * sigma)
    else:
        thr = float(np.nanmax(steps)) if steps.size else 0.0
    thr = float(max(thr, float(min_abs)))
    bad_mask = steps > thr
    return thr, bad_mask, stats


def spread_large_jumps_evenly(t, T_in, max_step=EVEN_MAX_STEP, context=EVEN_CONTEXT, iters=EVEN_ITERS,
                              protected_ranges=None, bad_mask_override=None):
    T = T_in.copy()
    P = T[:, :3, 3].copy()
    Q = np.array([R_to_quat(T[i, :3, :3]) for i in range(len(T))], float)

    prot = np.zeros(len(T), dtype=bool)
    if protected_ranges:
        for (a, b) in protected_ranges:
            a = max(0, int(a)); b = min(len(T) - 1, int(b))
            if a <= b:
                prot[a:b+1] = True

    for _ in range(iters):
        seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
        bad = bad_mask_override.copy() if (bad_mask_override is not None and len(bad_mask_override)==len(seg)) else (seg > max_step)
        bad = bad & ~(prot[:-1] | prot[1:])
        if not np.any(bad): break

        i = 0
        while i < len(bad):
            if not bad[i]:
                i += 1; continue
            j = i
            while j + 1 < len(bad) and bad[j + 1]:
                j += 1

            seg_cluster = seg[i:j+1]
            median_all = float(np.median(seg)) if np.isfinite(np.median(seg)) else max_step
            severity = float(np.clip(np.mean(seg_cluster) / max(median_all, 1e-12), 1.0, 5.0))
            ctx_boost = int(min(2, max(0, round(severity - 1.0))))
            ctx = int(context + ctx_boost)

            L = max(0, i - ctx)
            while L > 0 and prot[L]: L -= 1
            R = min(len(P) - 1, j + 1 + ctx)
            while R < len(P) - 1 and prot[R]: R += 1
            if R <= L or prot[L] or prot[R]:
                i = j + 1; continue

            t0, t1 = float(t[L]), float(t[R]); denom = max(t1 - t0, 1e-9)
            alphas = (t[L:R+1] - t0) / denom
            pL, pR = P[L].copy(), P[R].copy()
            qL, qR = Q[L].copy(), Q[R].copy()
            for k, a in enumerate(alphas):
                P[L+k] = (1.0 - a) * pL + a * pR
                Q[L+k] = quat_slerp(qL, qR, float(a))
            i = j + 1

    Tout = T.copy()
    Tout[:, :3, 3] = P
    for k in range(len(Tout)):
        Tout[k, :3, :3] = quat_to_R(Q[k])
    return Tout

def fix_seam_jumps(t, T_in, protected_ranges, max_step=0.006, context=3):
    T = T_in.copy()
    P = T[:, :3, 3].copy()
    Q = np.array([R_to_quat(T[i, :3, :3]) for i in range(len(T))], float)

    def in_prot(idx):
        for (a,b) in protected_ranges:
            if a <= idx <= b: return True
        return False

    N = len(T)

    for (a,b) in protected_ranges:
        if a-1 >= 0:
            i = a-1
            step = np.linalg.norm(P[i+1]-P[i])
            if step > max_step:
                L = max(0, i-context)
                while L > 0 and in_prot(L): L -= 1
                t0, t1 = float(t[L]), float(t[a]); denom = max(t1 - t0, 1e-9)
                alphas = (t[L:a+1] - t0) / denom
                for k, idx in enumerate(range(L, a+1)):
                    if idx == a: continue
                    a01 = float(alphas[k])
                    P[idx] = (1.0 - a01) * P[L] + a01 * P[a]
                    Q[idx] = quat_slerp(Q[L], Q[a], a01)

        if b < N-1:
            i = b
            step = np.linalg.norm(P[i+1]-P[i])
            if step > max_step:
                R = min(N-1, i+1+context)
                while R < N-1 and in_prot(R): R += 1
                t0, t1 = float(t[b]), float(t[R]); denom = max(t1 - t0, 1e-9)
                alphas = (t[b:R+1] - t0) / denom
                for k, idx in enumerate(range(b, R+1)):
                    if idx == b: continue
                    a01 = float(alphas[k])
                    P[idx] = (1.0 - a01) * P[b] + a01 * P[R]
                    Q[idx] = quat_slerp(Q[b], Q[R], a01)

    Tout = T.copy()
    Tout[:, :3, 3] = P
    for k in range(N):
        Tout[k, :3, :3] = quat_to_R(Q[k])
    return Tout

# =========================
# 主重参数化（固定 Catmull–Rom + SLERP）
# =========================
def reparam_master_for_slave_times(
    Tm, Ts, t_s,
    d_target=D_TARGET, y_comp_slave=Y_COMP_SLAVE,
    w_d=W_D, w_o=W_O, w_d_sched=None, w_o_sched=None,
    tol_d=TOL_D, tol_perp=TOL_PERP
):
    pM_knots, Rm_knots = Tm[:, :3, 3], Tm[:, :3, :3]
    pS = Ts[:, :3, 3].copy()
    if y_comp_slave != 0.0: pS[:,1] -= y_comp_slave
    Nseg = len(pM_knots)-1

    t_cr=chord_params(pM_knots, CR_ALPHA); P_ext,t_ext=_pad_pts(pM_knots, t_cr)
    p_eval_for = lambda i: (lambda u: cr_eval(P_ext, t_ext, i, u))
    q_knots = np.array([R_to_quat(Rm_knots[i]) for i in range(len(Rm_knots))])
    R_eval_for = lambda i: (lambda u: quat_to_R(quat_slerp(q_knots[i], q_knots[i+1], u)))

    kappa_m, _ = robust_curvature(pM_knots, win=CURV_FIT_WIN, min_pts=CURV_MIN_POINTS)
    eps_k=1e-9

    def segment_cost(p_eval, R_eval, u, pSk, w_dk, w_ok):
        pM = p_eval(u); R = R_eval(u)
        diff = pM - pSk
        d_err = np.linalg.norm(diff) - d_target
        o_err = _orth_err_agg(diff, R, ORTHO_AXES)
        return w_dk*d_err*d_err + w_ok*(o_err*o_err), d_err, o_err, pM, R

    def argmin_on_seg(i, pSk, w_dk, w_ok, u_lo=0.0, u_hi=1.0, grid=60):
        p_eval, R_eval = p_eval_for(i), R_eval_for(i)
        us=np.linspace(u_lo, u_hi, grid+1)
        costs=[segment_cost(p_eval,R_eval,u,pSk,w_dk,w_ok)[0] for u in us]
        kmin=int(np.argmin(costs)); u_best=us[kmin]
        phi=0.61803398875; a=max(u_lo, u_best-(u_hi-u_lo)/grid); b=min(u_hi, u_best+(u_hi-u_lo)/grid)
        f=lambda u: segment_cost(p_eval,R_eval,u,pSk,w_dk,w_ok)[0]
        c=b-phi*(b-a); d=a+phi*(b-a); fc,fd=f(c),f(d)
        for _ in range(24):
            if fc<fd: b,d,fd = d,c,fc; c=b-phi*(b-a); fc=f(c)
            else: a,c,fc = c,d,fd; d=a+phi*(b-a); fd=f(d)
        u=0.5*(a+b); C,de,oe,p,R = segment_cost(p_eval,R_eval,u,pSk,w_dk,w_ok)
        return u,C,de,oe,p,R

    K=len(t_s); out_T=[]; seg_used=np.zeros(K,int)
    prev_seg=-1; prev_u=0.0; i_start=0

    for k in range(K):
        pSk=pS[k]
        w_dk = w_d_sched[k] if w_d_sched is not None else w_d
        w_ok = w_o_sched[k] if w_o_sched is not None else w_o

        best=(np.inf, None); accepted=None
        for i in range(i_start, Nseg):
            u_lo = max(0.0, prev_u) if (NO_BACKTRACK and i==prev_seg) else 0.0
            u, C, de, oe, p_opt, R_opt = argmin_on_seg(i, pSk, w_dk, w_ok, u_lo=u_lo, u_hi=1.0, grid=60)
            if C < best[0]: best=(C, (i,u,de,oe,p_opt,R_opt))
            if abs(de)<=tol_d and abs(oe)<=tol_perp:
                accepted=(i,u,de,oe,p_opt,R_opt); break

        if accepted is None:
            if best[1] is None:
                raise RuntimeError(f"Reparam failed at k={k}: no feasible segment.")
            i_final,u_final,de,oe,p_fin,R_fin = best[1]
        else:
            i_final,u_final,de,oe,p_fin,R_fin = accepted

        if PRESERVE_MASTER_KNOTS:
            km=float(kappa_m[min(i_final, len(kappa_m)-1)])
            R_here=(1.0/max(km,eps_k)) if km>eps_k else np.inf
            snap_u = ANCHOR_SNAP_U * (1.0 + SNAP_FLAT_GAIN * min(max(0.0, R_here/max(ADAPT_R_LO,1e-9)-1.0), 1.5))
            if u_final <= snap_u or (1.0-u_final) <= snap_u:
                if u_final <= snap_u:
                    p_fin, R_fin, u_final = pM_knots[i_final],   Rm_knots[i_final],   0.0
                else:
                    p_fin, R_fin, u_final = pM_knots[i_final+1], Rm_knots[i_final+1], 1.0

        T=np.eye(4); T[:3,:3]=R_fin; T[:3,3]=p_fin; out_T.append(T)
        seg_used[k] = i_final
        prev_seg, prev_u, i_start = i_final, u_final, i_final

    return np.stack(out_T, 0), seg_used
