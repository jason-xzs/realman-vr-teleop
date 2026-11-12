#!/usr/bin/env python3
"""
诊断脚本：验证 RM-75 臂角法 IK 是否能解决 home 附近 J1 跨解问题。

执行方式：
    conda activate robocoin
    cd /home/ubuntu/Desktop/Workspace/lerobot_policy_deploy/lerobotv3/vr_teleop/spacemouse_control_arm/ros2_realman_ws
    python diagnose_ik_arm_angle.py

⚠️  此脚本仅调用 SDK 算法接口，不向机械臂发送任何运动指令，可安全执行。
   只需要机械臂上电连网（不需要先回 home，也不需要启用跟随）。

验证项：
1. rm_algo_calculate_arm_angle_from_config_rm75 能否算出 home 配置的手臂角
2. SDK FK(home_joint_deg) 给出的笛卡尔 pose
3. 臂角法 IK：用 (home_pose, arm_angle) 反求解，应 ≈ home_joint_deg（容差 <1°）
4. 对照普通 IK：复现今天 -75° 跨解（如果 SDK 通用 IK 真的不约束手臂角）
5. 微扰动稳定性：每帧 X+1mm，对比两套 IK 的 J1 是否平滑
"""
import sys

# 与 driver yaml 完全一致
ROBOT_IP = "192.168.1.18"
ROBOT_PORT = 8080
HOME_JOINT_DEG = [-0.263, -0.505, -0.157, 43.953, 1.225, 121.751, -0.187]


def main():
    try:
        from Robotic_Arm.rm_robot_interface import (
            RoboticArm,
            rm_thread_mode_e,
            rm_inverse_kinematics_params_t,
        )
    except ImportError as e:
        print(f"[FATAL] 无法导入 SDK: {e}")
        print("       请确认: conda activate robocoin（Robotic_Arm 已安装）")
        sys.exit(1)

    arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    handle = arm.rm_create_robot_arm(ROBOT_IP, ROBOT_PORT)
    if handle is None or getattr(handle, "id", -1) == -1:
        print(f"[FATAL] 无法连接机械臂 {ROBOT_IP}:{ROBOT_PORT}")
        sys.exit(1)
    print(f"[OK] 机械臂已连接 ({ROBOT_IP}:{ROBOT_PORT})\n")

    try:
        run_tests(arm, rm_inverse_kinematics_params_t)
    finally:
        arm.rm_delete_robot_arm()
        print("\n[OK] 机械臂连接已关闭")


def run_tests(arm, IKParams):
    ref = HOME_JOINT_DEG
    print(f"参考关节配置 home_joint_deg: {[f'{x:+.3f}' for x in ref]}\n")

    # === 测试 1: 计算手臂角 ===
    print("=" * 70)
    print("测试 1: 计算 home 配置对应的手臂角")
    print("=" * 70)
    ret, arm_angle = arm.rm_algo_calculate_arm_angle_from_config_rm75(ref)
    print(f"  rm_algo_calculate_arm_angle_from_config_rm75(home)")
    print(f"  ret       = {ret}  (0=成功 / -1=失败或非RM75 / -2=参数非法)")
    print(f"  arm_angle = {arm_angle:.3f}°")
    if ret != 0:
        print("  [FAIL] 手臂角计算失败，臂角法不可用，终止后续测试")
        return
    print("  [PASS] 手臂角计算成功\n")

    # === 测试 2: FK 算 home 对应 EEF pose ===
    print("=" * 70)
    print("测试 2: FK(home_joint_deg) → home 笛卡尔 pose")
    print("=" * 70)
    home_pose = arm.rm_algo_forward_kinematics(ref, flag=1)  # flag=1 欧拉角
    print(f"  home_pose [x,y,z,rx,ry,rz] = {[f'{x:+.4f}' for x in home_pose]}\n")

    # === 测试 3: 臂角法 IK（关键验证）===
    print("=" * 70)
    print("测试 3: 臂角法 IK — rm_algo_inverse_kinematics_rm75_for_arm_angle")
    print("=" * 70)
    p = IKParams(q_in=ref, q_pose=home_pose, flag=1)
    ret_a, q_a = arm.rm_algo_inverse_kinematics_rm75_for_arm_angle(p, arm_angle)
    print(f"  ret      = {ret_a}  (0=成功 / -1=失败 / -2=超限位 / -3=非RM75)")
    if ret_a == 0:
        print(f"  解出关节 = {[f'{x:+.3f}' for x in q_a]}")
        print(f"  期望关节 = {[f'{x:+.3f}' for x in ref]}")
        diffs = [abs(a - b) for a, b in zip(q_a, ref)]
        max_diff = max(diffs)
        verdict = "[PASS] 臂角法解 ≈ home（关节空间稳定）" if max_diff < 1.0 else f"[FAIL] 跨解 max_diff={max_diff:.3f}°"
        print(f"  最大关节差 = {max_diff:.3f}°  →  {verdict}")
    else:
        print("  [FAIL] 臂角法 IK 调用错误")
    print()

    # === 测试 4: 对照普通 IK ===
    print("=" * 70)
    print("测试 4: 对照 — 普通 IK rm_algo_inverse_kinematics（当前 driver 用的）")
    print("=" * 70)
    p2 = IKParams(q_in=ref, q_pose=home_pose, flag=1)
    ret_b, q_b = arm.rm_algo_inverse_kinematics(p2)
    print(f"  ret      = {ret_b}")
    if ret_b == 0:
        print(f"  解出关节 = {[f'{x:+.3f}' for x in q_b]}")
        print(f"  期望关节 = {[f'{x:+.3f}' for x in ref]}")
        diffs = [abs(a - b) for a, b in zip(q_b, ref)]
        max_diff = max(diffs)
        j1_diff = abs(q_b[0] - ref[0])
        if max_diff > 5.0:
            print(f"  最大关节差 = {max_diff:.3f}°  J1 差 = {j1_diff:.3f}°")
            print(f"  [BAD] 普通 IK 跨解 — 复现今天的根因")
        else:
            print(f"  最大关节差 = {max_diff:.3f}°")
            print(f"  [OK] 普通 IK 这次给出接近 home 的解（这种 case 未暴露）")
    print()

    # === 测试 5: 微扰动稳定性 ===
    print("=" * 70)
    print("测试 5: 微扰动稳定性 — 每帧 X+1mm 共 8 帧，对比两套 IK 的 J1 是否平滑")
    print("=" * 70)
    print(f"  {'iter':>4} | {'X 偏移(mm)':>10} | {'臂角法 J1':>14} | {'普通 IK J1':>14}")
    print(f"  {'-'*4} | {'-'*10} | {'-'*14} | {'-'*14}")
    seed_a = list(q_a) if ret_a == 0 else list(ref)
    for i in range(8):
        dx = 0.001 * (i + 1)
        pert = list(home_pose)
        pert[0] += dx
        # 臂角法（hold seed 模拟 driver 行为）
        pa = IKParams(q_in=seed_a, q_pose=pert, flag=1)
        ra, qa = arm.rm_algo_inverse_kinematics_rm75_for_arm_angle(pa, arm_angle)
        # 普通 IK（每次 seed 用 ref，模拟今天 driver 修复前的行为）
        pb = IKParams(q_in=ref, q_pose=pert, flag=1)
        rb, qb = arm.rm_algo_inverse_kinematics(pb)
        a_j1 = f"{qa[0]:+.3f}°" if ra == 0 else f"err{ra}"
        b_j1 = f"{qb[0]:+.3f}°" if rb == 0 else f"err{rb}"
        print(f"  {i+1:>4} | {dx*1000:>10.1f} | {a_j1:>14} | {b_j1:>14}")
        if ra == 0:
            seed_a = list(qa)
    print()
    print("  判断标准：臂角法 J1 应在 8 帧内单调或平滑变化（每步 < 1°）；")
    print("           普通 IK 若出现 J1 大幅 swing（>20°）即复现今天的抖动根因。")


if __name__ == "__main__":
    main()
