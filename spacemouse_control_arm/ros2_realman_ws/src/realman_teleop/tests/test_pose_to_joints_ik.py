"""pose→joints IK 包装函数的单元测试（纯函数，mock SDK arm）

测试臂角法 IK：rm_algo_inverse_kinematics_rm75_for_arm_angle
"""
from unittest.mock import MagicMock

import pytest

# 待测函数：从 realman_driver_node 模块顶层导出
from realman_teleop.realman_driver_node import _pose_to_target_joint_deg


def _make_mock_arm(ret_code: int, q_out_deg: list[float]) -> MagicMock:
    """构造 mock arm.rm_algo_inverse_kinematics_rm75_for_arm_angle 行为。"""
    arm = MagicMock()
    arm.rm_algo_inverse_kinematics_rm75_for_arm_angle.return_value = (ret_code, q_out_deg)
    return arm


def test_ik_success_with_euler_pose_returns_joint_deg():
    """6 维欧拉 pose（flag=1）→ IK 成功 → 返回 joint_deg 列表。"""
    arm = _make_mock_arm(0, [10.0, 20.0, -5.0, 30.0, -15.0, 5.0, 0.0])
    pose = [0.3, 0.0, 0.4, 0.0, 1.57, 0.0]  # xyz + rx,ry,rz
    q_in_deg = [0.0] * 7

    ok, q_out = _pose_to_target_joint_deg(arm, pose, q_in_deg, arm_angle=0.0)

    assert ok is True
    assert q_out == [10.0, 20.0, -5.0, 30.0, -15.0, 5.0, 0.0]
    args, _ = arm.rm_algo_inverse_kinematics_rm75_for_arm_angle.call_args
    ik_params = args[0]
    assert ik_params.flag == 1


def test_ik_success_with_quat_pose_uses_flag_zero():
    """7 维四元数 pose（flag=0）→ IK 成功 → 返回 joint_deg 列表。"""
    arm = _make_mock_arm(0, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0])
    pose = [0.3, 0.0, 0.4, 1.0, 0.0, 0.0, 0.0]  # xyz + qw,qx,qy,qz
    q_in_deg = [0.0] * 7

    ok, q_out = _pose_to_target_joint_deg(arm, pose, q_in_deg, arm_angle=0.0)

    assert ok is True
    assert q_out == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
    args, _ = arm.rm_algo_inverse_kinematics_rm75_for_arm_angle.call_args
    ik_params = args[0]
    assert ik_params.flag == 0


def test_ik_failure_returns_false_and_none():
    """IK 返回非 0 → ok=False, q_out=None（调用方负责 hold last）。"""
    arm = _make_mock_arm(1, [0.0] * 7)
    pose = [10.0, 10.0, 10.0, 0.0, 0.0, 0.0]
    q_in_deg = [0.0] * 7

    ok, q_out = _pose_to_target_joint_deg(arm, pose, q_in_deg, arm_angle=0.0)

    assert ok is False
    assert q_out is None


def test_ik_invalid_pose_dim_raises():
    """pose 维度非 6/7 → 抛 ValueError。"""
    arm = _make_mock_arm(0, [0.0] * 7)
    with pytest.raises(ValueError, match="pose dim"):
        _pose_to_target_joint_deg(arm, [0.0] * 5, [0.0] * 7, arm_angle=0.0)


def test_q_in_deg_passed_to_ik_params():
    """q_in_deg 必须传给 IK 参数（IK 收敛依赖参考关节）。"""
    arm = _make_mock_arm(0, [0.0] * 7)
    q_in_deg = [11.1, 22.2, 33.3, 44.4, 55.5, 66.6, 77.7]
    pose = [0.3, 0.0, 0.4, 0.0, 0.0, 0.0]

    _pose_to_target_joint_deg(arm, pose, q_in_deg, arm_angle=0.0)

    args, _ = arm.rm_algo_inverse_kinematics_rm75_for_arm_angle.call_args
    ik_params = args[0]
    for i, expected in enumerate(q_in_deg):
        assert abs(ik_params.q_in[i] - expected) < 1e-3


def test_arm_angle_passed_to_ik_call():
    """arm_angle 必须作为第二参数传给臂角法 IK（锁定 7-DoF 零空间）。"""
    arm = _make_mock_arm(0, [0.0] * 7)
    pose = [0.3, 0.0, 0.4, 0.0, 0.0, 0.0]
    q_in_deg = [0.0] * 7

    _pose_to_target_joint_deg(arm, pose, q_in_deg, arm_angle=42.5)

    args, _ = arm.rm_algo_inverse_kinematics_rm75_for_arm_angle.call_args
    assert args[1] == pytest.approx(42.5)
