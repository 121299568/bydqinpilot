#!/usr/bin/env python3
"""安全三修复离线验证套件（驱动真实代码，零外部依赖，只需 numpy）。

覆盖：
  P 系列 画龙（lib/latcontrol_torque.py 救急修正块）— 真实 LatControlTorque.update() 逐帧驱动
         读 pid_log.desiredLateralAccel（== κ·v² + 救急修正），CI 记录器做交叉校验
  C 系列 弯道外漂（desired_lateral_accel 去 0.85）+ drive_helpers jerk 兜底
  T 系列 TTC 闭合速度（controlsd.py）— AST 静态锁定真实表达式 + 源码解析真实常量 + 行为矩阵

用法： python3 tools/test_safety_fixes.py [--out out.html]
"""
import argparse
import ast
import importlib.util
import os
import re
import sys
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CONTROLS = os.path.dirname(HERE)

FRAMES = 160          # 1.6s @100Hz，低通（τ≈67ms）足够收敛
DT_CTRL = 0.01
_NPT = 20             # 合成车道线点数（代码只读 y[0]，长度>0 即可）


# --------------------------------------------------------------------------
# 依赖 mock：让真实 latcontrol_torque.py / drive_helpers.py 在无 openpilot 环境可 import
# --------------------------------------------------------------------------
class _FF:  # 一阶低通（替代 openpilot.common.filter_simple.FirstOrderFilter）
    def __init__(self, init, ts, dt, editable=False):
        self.y = float(init)
        self.k = min(1.0, dt / max(ts, 1e-9))

    def update(self, x):
        self.y += self.k * (float(x) - self.y)
        return self.y


class _FakeCloudlog:
    def warning(self, *a, **k): pass
    def info(self, *a, **k): pass


class _FakeParams:
    def get_bool(self, key): return False
    def get(self, key, encoding=None): return "0"


class _SubMaster:
    def __init__(self, services): self.services = list(services)
    def update(self, *a, **k): pass
    def __getitem__(self, k): return types.SimpleNamespace()


class _ButtonType:
    accelCruise = "accelCruise"
    decelCruise = "decelCruise"


class _ButtonEvent:
    Type = _ButtonType


def _cereal_box():
    """假 cereal.car / cereal.log：只满足 drive_helpers 模块级按名取值与注解求值。"""
    return types.SimpleNamespace(
        car=types.SimpleNamespace(
            CarState=types.SimpleNamespace(ButtonEvent=_ButtonEvent),
            CarParams=types.SimpleNamespace(LateralTorqueTuning=type("LateralTorqueTuning", (object,), {}))),
        log=types.SimpleNamespace(
            ModelDataV2=type("ModelDataV2", (object,), {})))


def _new_msg():
    return types.SimpleNamespace()


class _FakeCarInterfaceBase:
    @staticmethod
    def torque_from_lateral_accel_linear(*a, **k):
        return 0.0


class _FakeCI:
    """假车接口：has_lateral_torque_nn=False 走非 NN 路径；
    torque_from_lateral_accel 记录入参（setpoint/measurement）做交叉校验。"""
    has_lateral_torque_nn = False

    def __init__(self):
        self.torque_calls = []

    def torque_from_lateral_accel(self):
        calls = self.torque_calls

        def _f(x, *a, **k):
            calls.append(float(x))
            return 0.0
        return _f

    def low_speed_factor_handler(self):
        return lambda *a, **k: 0.0


class _FakeVM:
    def __init__(self, kappa=0.0):
        self.kappa = kappa

    def calc_curvature(self, angle, v, roll):
        return self.kappa


T_IDXS = list(np.linspace(0.0, 2.0, 33))   # 33 点时间轴（真实 ModelConstants.T_IDXS 结构）


def _register(name, **attrs):
    parts = name.split(".")
    for i in range(1, len(parts) + 1):
        sub = ".".join(parts[:i])
        if sub not in sys.modules:
            sys.modules[sub] = types.ModuleType(sub)
    for k, v in attrs.items():
        setattr(sys.modules[name], k, v)


def _install_mocks():
    _register("openpilot.common.realtime", DT_MDL=0.05, DT_CTRL=DT_CTRL)
    _register("openpilot.common.numpy_fast", interp=np.interp, clip=lambda v, lo, hi: lo if v < lo else (hi if v > hi else v))
    _register("openpilot.common.filter_simple", FirstOrderFilter=_FF)
    _register("openpilot.common.conversions",
              Conversions=types.SimpleNamespace(MPH_TO_MS=0.44704, KPH_TO_MS=1.0 / 3.6))
    _register("openpilot.common.params", Params=_FakeParams)
    _register("openpilot.system.swaglog", cloudlog=_FakeCloudlog())
    _register("openpilot.selfdrive.car.interfaces", CarInterfaceBase=_FakeCarInterfaceBase)
    _register("openpilot.selfdrive.car.byd.values", BYDForceTorqueFix="BYDForceTorqueFix")
    _register("openpilot.selfdrive.modeld.constants",
              ModelConstants=types.SimpleNamespace(T_IDXS=T_IDXS))
    _register("openpilot.selfdrive.hybrid_modeld.constants",
              ModelConstants=types.SimpleNamespace(T_IDXS=T_IDXS))
    _register("openpilot.selfdrive.controls.lib.drive_helpers",
              CONTROL_N=17, MIN_SPEED=1.0, MAX_LATERAL_JERK=5.0)
    box = _cereal_box()
    _register("cereal",
              car=box.car,
              log=types.SimpleNamespace(
                  ModelDataV2=box.log.ModelDataV2,
                  ControlsState=types.SimpleNamespace(
                      LateralTorqueState=types.SimpleNamespace(new_message=_new_msg))),
              custom=types.SimpleNamespace())
    _register("cereal.messaging", SubMaster=_SubMaster, new_message=lambda *a, **k: None)

    # 真实子模块（依赖已被上面 mock 覆盖，直接装入 sys.modules，绕过包路径解析）：
    # latcontrol.py 是 LatControlTorque 的基类；pid.py 是真实 PID 控制器。
    for rel, full in [("lib/latcontrol.py", "openpilot.selfdrive.controls.lib.latcontrol"),
                      ("lib/pid.py", "openpilot.selfdrive.controls.lib.pid")]:
        sys.modules[full] = _load(os.path.join(CONTROLS, rel), full.rsplit(".", 1)[-1] + "_under_test")
    _register("openpilot.selfdrive.controls.lib.vehicle_model", ACCELERATION_DUE_TO_GRAVITY=9.81)


def _load(mod_path, mod_name):
    spec = importlib.util.spec_from_file_location(mod_name, mod_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# --------------------------------------------------------------------------
# 消息/状态工厂
# --------------------------------------------------------------------------
def _lane(y):
    return types.SimpleNamespace(y=[y] * _NPT)


def _make_model(ll_y=1.9, rl_y=-1.9, probs=None, stds=None):
    """合成 modelV2（model_good=False：orientation/acceleration 给短数组，
    跳过 NN/前馈 jerk 分支，聚焦被测的救急修正块与稳态目标）。"""
    if probs is None:
        probs = [1.0, 1.0, 1.0, 1.0]
    if stds is None:
        stds = [0.01, 0.01, 0.01, 0.01]
    return types.SimpleNamespace(
        laneLines=[_lane(ll_y * 2), _lane(ll_y), _lane(rl_y), _lane(rl_y * 2)],
        laneLineProbs=[float(p) for p in probs],
        laneLineStds=[float(s) for s in stds],
        orientation=types.SimpleNamespace(x=[0.0] * 5),
        acceleration=types.SimpleNamespace(y=[0.0] * 5),
    )


def _make_CP():
    torque = types.SimpleNamespace(
        kp=1.0, ki=0.0, kf=0.0,
        useSteeringAngle=True, steeringAngleDeadzoneDeg=0.0,
        latAccelFactor=1.0, latAccelOffset=0.0, friction=0.0)
    return types.SimpleNamespace(
        steerLimitTimer=3.0,
        lateralTuning=types.SimpleNamespace(torque=torque),
        steerActuatorDelay=0.1)


def _make_CS(vEgo=20.0, leftBlinker=False, rightBlinker=False):
    return types.SimpleNamespace(
        vEgo=vEgo, aEgo=0.0,
        steeringAngleDeg=0.0, steeringRateDeg=0.0, steeringPressed=False,
        leftBlinker=leftBlinker, rightBlinker=rightBlinker)


def _run_lat(mod, kappa, vEgo, model_fn, frames=FRAMES, left_blinker=False):
    """逐帧驱动真实 LatControlTorque.update()，返回 (desiredLateralAccel 历史, CI 入参记录)。"""
    CI = _FakeCI()
    CP = _make_CP()
    CS = _make_CS(vEgo, leftBlinker=left_blinker)
    VM = _FakeVM(kappa=0.0)          # 实际曲率（未转），让 desired 侧干净
    params = types.SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)
    lat = mod.LatControlTorque(CP, CI)
    hist = []
    for _ in range(frames):
        md = model_fn()
        _t, _s, pid_log = lat.update(True, CS, VM, params, None, False,
                                      kappa, 0.0, types.SimpleNamespace(), md)
        hist.append(getattr(pid_log, "desiredLateralAccel", None))
    return np.array([0.0 if h is None else h for h in hist]), CI.torque_calls


# --------------------------------------------------------------------------
# P 系列：画龙（救急修正块）
# --------------------------------------------------------------------------
def sc_p1_steady(mod):
    """P1 直行稳定偏离 60cm：修正应收敛到 0.22·(0.6-0.45)/0.75=0.044 且无振荡。"""
    tgt = 0.22 * (0.60 - 0.45) / 0.75
    hist, calls = _run_lat(mod, 0.0, 20.0, lambda: _make_model(1.6, -2.2))
    tail = hist[-60:]
    ok = (abs(hist[-1] - tgt) < 0.003
          and np.all(tail > 0)                              # 符号恒定，无来回穿越
          and float(np.max(np.abs(np.diff(tail)))) < 0.002   # 稳态无抖动
          and abs(calls[-1] - hist[-1]) < 1e-9)              # CI 通道交叉校验
    return hist, ok, (f"末帧={hist[-1]:+.4f} (期望 {tgt:+.4f})，后60帧符号恒定"
                      f" 最大单帧变化={np.max(np.abs(np.diff(tail))):.5f}")


def sc_p2_release(mod):
    """P2 对称释放：先 60cm 触发收敛，再突然回正(diff→0)，衰减到 10% 应在
    ~14.2 帧（旧码 0.1275 系数会 ~1 帧阶跃关断，构成弛张振荡）。"""
    tgt = 0.22 * (0.60 - 0.45) / 0.75
    CI = _FakeCI()
    CP, CS = _make_CP(), _make_CS(20.0)
    VM, params = _FakeVM(0.0), types.SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)
    lat = mod.LatControlTorque(CP, CI)
    hist = []
    for i in range(FRAMES * 2):
        md = _make_model(1.6, -2.2) if i < 120 else _make_model(1.9, -1.9)
        _t, _s, pid_log = lat.update(True, CS, VM, params, None, False, 0.0, 0.0,
                                     types.SimpleNamespace(), md)
        hist.append(pid_log.desiredLateralAccel)
    hist = np.array(hist)
    n10 = int(np.argmax(hist[120:] <= tgt * 0.1))            # 回正后衰减到 10% 的帧数
    n10 = FRAMES * 2 if n10 == 0 else n10
    ok = (abs(hist[119] - tgt) < 0.003 and 9 <= n10 <= 22)
    return hist, ok, (f"触发段末帧={hist[119]:+.4f} (期望 {tgt:+.4f})；回正后衰减到 10%"
                      f" 用 {n10} 帧={n10 * DT_CTRL * 1000:.0f}ms（期望 ~142ms；"
                      f"旧 0.1275 系数约 10ms 阶跃）")


def sc_p3_conf_gate(mod):
    """P3 置信度门控：低 prob / 高 std 的车道线不得触发；满置信对照必须触发。"""
    tgt = 0.22 * (0.60 - 0.45) / 0.75
    h_lowp, _ = _run_lat(mod, 0.0, 20.0,
                         lambda: _make_model(1.6, -2.2, probs=[1.0, 0.2, 1.0, 1.0]))
    h_highs, _ = _run_lat(mod, 0.0, 20.0,
                          lambda: _make_model(1.6, -2.2, stds=[0.01, 0.5, 0.01, 0.01]))
    h_ok, _ = _run_lat(mod, 0.0, 20.0, lambda: _make_model(1.6, -2.2))
    ok = (np.max(np.abs(h_lowp)) < 1e-6 and np.max(np.abs(h_highs)) < 1e-6
          and abs(h_ok[-1] - tgt) < 0.003)
    return h_ok, ok, (f"低prob={np.max(np.abs(h_lowp)):.2e} 高std={np.max(np.abs(h_highs)):.2e}"
                      f" 满置信末帧={h_ok[-1]:+.4f} (期望 {tgt:+.4f})")


def sc_p4_blinker(mod):
    """P4 打灯变道抑制（匹配 L282 真实行为：打灯期间 _post_blinker_timer 被钉在
    LC_SETTLE_TIME，抑制窗 = 打灯全程 + 熄灯后 2.0s）：
    打灯 2.0s（延迟期 1.0s + 变道执行 1.0s）→ 熄灯，抑制窗内救急修正必须全零，
    抑制窗结束后恢复收敛到 0.044。"""
    tgt = 0.22 * (0.60 - 0.45) / 0.75
    CI = _FakeCI()
    CP = _make_CP()
    CS_on, CS_off = _make_CS(20.0, leftBlinker=True), _make_CS(20.0)
    VM, params = _FakeVM(0.0), types.SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)
    lat = mod.LatControlTorque(CP, CI)
    BLINK = 200                       # 打灯 2.0s
    SETTLE = int(round(2.0 / DT_CTRL))  # 熄灯后 LC_SETTLE_TIME=2.0s
    TOTAL = BLINK + SETTLE + FRAMES
    hist = []
    for i in range(TOTAL):
        CS = CS_on if i < BLINK else CS_off
        md = _make_model(1.6, -2.2)
        _t, _s, pid_log = lat.update(True, CS, VM, params, None, False, 0.0, 0.0,
                                     types.SimpleNamespace(), md)
        hist.append(pid_log.desiredLateralAccel)
    hist = np.array(hist)
    n_sup = BLINK + SETTLE                     # 抑制窗帧数上界
    suppress = hist[:n_sup - 1]               # 抑制严格成立的最后一帧（计时器归零那帧已放行）
    first_free = hist[n_sup - 1]              # 熄灯后整 2.0s 那帧：计时器=0，救急恢复
    after = hist[n_sup:]
    ok = (np.max(np.abs(suppress)) < 1e-9          # 抑制窗内严格为 0（elif 分支清零滤波状态）
          and abs(first_free - tgt * 0.15) < 0.002  # 恢复首帧恰为单帧低通台阶，证明 2.0s 计时精确
          and abs(after[-1] - tgt) < 0.003)        # 恢复后收敛到救急目标值
    return hist, ok, (f"抑制窗={BLINK}+{SETTLE - 1}帧（打灯全程+熄灯后2.0s）最大值="
                      f"{np.max(np.abs(suppress)):.2e}（应=0）；"
                      f"熄灯后整2s那帧={first_free:+.4f}（首个低通台阶 {tgt * 0.15:+.4f}）；"
                      f"恢复后末帧={after[-1]:+.4f} (期望 {tgt:+.4f})")


def sc_p5_guardrail(mod):
    """P5 回归保护：车道线 >4m 判为护栏误检 → diff 强制 0 → 不触发。"""
    hist, _ = _run_lat(mod, 0.0, 20.0, lambda: _make_model(5.0, -1.9))
    ok = np.max(np.abs(hist)) < 1e-9
    return hist, ok, f"全程最大修正={np.max(np.abs(hist)):.2e} (期望 0，护栏钳制未被破坏)"


# --------------------------------------------------------------------------
# C 系列：弯道外漂（去 0.85）+ jerk 兜底
# --------------------------------------------------------------------------
def sc_c1_curve_full(mod):
    """C1 弯道稳态满额：κ=0.01, v=25 → 6.25 m/s²（旧 *0.85 给 5.3125，差 0.94）。"""
    hist, calls = _run_lat(mod, 0.01, 25.0, lambda: _make_model(1.9, -1.9))
    want, legacy = 0.01 * 25.0 ** 2, 0.01 * 25.0 ** 2 * 0.85
    ok = abs(hist[-1] - want) < 0.02
    return hist, ok, f"末帧={hist[-1]:.4f} (期望 {want:.4f}；旧 0.85 为 {legacy:.4f}，差 {want - legacy:.2f})"


def sc_c2_curve_plus_correction(mod):
    """C2 弯道 + 救急叠加：6.25 + 0.044，两路独立、弯道满额不被压缩。"""
    tgt = 0.01 * 25.0 ** 2 + 0.22 * (0.60 - 0.45) / 0.75
    hist, _ = _run_lat(mod, 0.01, 25.0, lambda: _make_model(1.6, -2.2))
    ok = abs(hist[-1] - tgt) < 0.03
    return hist, ok, f"末帧={hist[-1]:.4f} (期望 κ·v²+救急={tgt:.4f}，两路独立叠加)"


def sc_c3_jerk_cap(dh_mod):
    """C3 jerk 兜底（真实 drive_helpers.get_lag_adjusted_curvature）：
    ① 曲率阶跃/过冲被钳在 κ_0 ± MAX_LATERAL_JERK/v²·DT_MDL 内；
    ② 曲率率被钳到 ±5/v²。这是"去 0.85 不引入阶跃过冲"的论证基础。"""
    CP = _make_CP()
    v = 25.0
    rate_cap = 5.0 / v ** 2
    # 稳态：ψ 与 κ 自洽 → desired=κ，无过冲
    psi = 0.01 * v * (0.1 + 0.2)   # ψ(delay=0.3s) = κ·v·delay
    steady = dh_mod.get_lag_adjusted_curvature(
        CP, v, [psi] * 17, [0.01] * 17, [0.0] * 17)
    # 过冲注入：ψ 翻 3 倍 → desired = 2·avg_ψ/(v·delay) - κ_0 偏离 κ_0
    over = dh_mod.get_lag_adjusted_curvature(
        CP, v, [psi * 3] * 17, [0.01] * 17, [0.0] * 17)
    # 率钳位：curvature_rates[0]=100 → 安全率 = 5/v²
    rate = dh_mod.get_lag_adjusted_curvature(
        CP, v, [psi] * 17, [0.01] * 17, [100.0] * 17)
    band = rate_cap * 0.05
    ok = (abs(steady[0] - 0.01) < 1e-9
          and abs(over[0] - 0.01) <= band * 1.0001
          and abs(rate[1]) <= rate_cap * 1.0001)
    hist = np.array([steady[0], over[0], rate[1]])
    return hist, ok, (f"稳态 desired={steady[0]:+.5f}；过冲注入后 safe={over[0]:+.5f}"
                      f"（钳位带 ±{band:.2e}）；率钳位 |rate|={abs(rate[1]):.5f}"
                      f"（上限 {rate_cap:.5f}）")


# --------------------------------------------------------------------------
# T 系列：TTC 闭合速度（controlsd.py）
# --------------------------------------------------------------------------
def _parse_consts(src):
    out = {}
    for line in src.splitlines():
        m = re.match(r"^\s*(LEAD_EBRAKE_\w+|EMERGENCY_STATIC_BRAKE)\s*=\s*(True|False|[-+]?\d+\.?\d*)", line)
        if m:
            v = m.group(2)
            out[m.group(1)] = v == "True" if v in ("True", "False") else float(v)
    return out


def _check_ttc_ast(src):
    """静态锁定 controlsd.py 里真实表达式：分母必须是闭合速度，不能回退成自车速度。"""
    tree = ast.parse(src)
    closing = ttc = None
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name):
                    if t.id == "closing_speed_eb":
                        closing = node.value
                    elif t.id == "ttc_eb":
                        ttc = node.value
    if closing is None or ttc is None:
        return False, "未找到 closing_speed_eb / ttc_eb 赋值节点"
    c_dump, t_dump = ast.dump(closing), ast.dump(ttc)
    if "max" not in c_dump or "vLead" not in c_dump or "vEgo" not in c_dump or "Sub" not in c_dump:
        return False, f"closing_speed_eb 不是 max(vEgo - vLead, ·): {c_dump}"
    if "Div" not in t_dump or "closing_speed_eb" not in t_dump or "d_rel_eb" not in t_dump:
        return False, f"ttc_eb 未除以闭合速度: {t_dump}"
    if re.search(r"ttc_eb\s*=\s*d_rel_eb\s*/\s*max\s*\(\s*CS\.vEgo", src):
        return False, "检测到 ttc_eb 回退为 vEgo 分母（旧 bug）"
    return True, f"closing_speed_eb={c_dump[:80]}…  ttc_eb={t_dump[:60]}…"


def _eb_legacy(vEgo, dRel):
    """旧公式（分母恒 vEgo）用于对照。"""
    return dRel / max(vEgo, 0.1)


def _eb_branch(vEgo, dRel, vLead, aLeadK, C):
    """前车急刹联动分支（表达式由 T1 AST 静态锁定；常量由源码解析）。"""
    if C["LEAD_EBRAKE_ENABLE"] and aLeadK < C["LEAD_EBRAKE_ACCEL_TH"] \
       and 0.0 < dRel < C["LEAD_EBRAKE_DIST_MAX_M"]:
        d_rel_eb = max(dRel, 0.1)
        closing = max(vEgo - vLead, 0.1)
        ttc = d_rel_eb / closing
        return ttc < C["LEAD_EBRAKE_TTC_S"], ttc
    return False, None


def _tcase(C, name, vEgo, dRel, vLead, aLeadK, expect, note):
    fire, ttc = _eb_branch(vEgo, dRel, vLead, aLeadK, C)
    gate = C["LEAD_EBRAKE_ENABLE"] and aLeadK < C["LEAD_EBRAKE_ACCEL_TH"] \
        and 0.0 < dRel < C["LEAD_EBRAKE_DIST_MAX_M"]
    leg_ttc = _eb_legacy(vEgo, dRel) if gate else None
    leg_fire = gate and leg_ttc < C["LEAD_EBRAKE_TTC_S"]
    ok = fire == expect
    return None, ok, (f"新公式触发={fire} (期望 {expect})，闭合速度 TTC="
                      f"{round(ttc, 2) if ttc is not None else '—'}s"
                      f"；旧 vEgo 公式 TTC={round(leg_ttc, 2) if leg_ttc is not None else '—'}s"
                      f"→旧触发={leg_fire}；{note}")


def run_ttc(controlsd_src):
    """返回 [(name, hist, ok, note)]，首项是 AST 静态检查。"""
    res = []
    ok_ast, note_ast = _check_ttc_ast(controlsd_src)
    res.append(("T1 TTC 公式 AST 静态锁定", None, ok_ast, note_ast))
    C = _parse_consts(controlsd_src)
    need = {"LEAD_EBRAKE_ENABLE", "LEAD_EBRAKE_ACCEL_TH", "LEAD_EBRAKE_DIST_MAX_M", "LEAD_EBRAKE_TTC_S"}
    if not need.issubset(C):
        res.append(("T1b 常量解析", None, False,
                    f"缺少常量: {sorted(need - set(C))}"))
        return res

    res.append(("T2 120km/h跟100km/h前车急刹@55m", None) + _tcase(
        C, "", 33.3, 55.0, 27.8, -5.0, False,
        "同向高差→新公式不误触（旧公式会触发全力刹）")[1:])
    res.append(("T3 72km/h跟36km/h前车急刹@20m", None) + _tcase(
        C, "", 20.0, 20.0, 10.0, -4.0, True,
        "真急刹→新公式仍触发且更精确")[1:])
    res.append(("T4 72km/h对近停前车@15m", None) + _tcase(
        C, "", 20.0, 15.0, 0.5, -8.0, True,
        "vLead→0 时闭合速度退化为 vEgo，与静态安全网一致")[1:])
    res.append(("T5a 边界: dRel=60m 不触发", None) + _tcase(
        C, "", 20.0, 60.0, 5.0, -5.0, False, "距离上限排排除")[1:])
    res.append(("T5b 边界: aLeadK=-2.9 不触发", None) + _tcase(
        C, "", 20.0, 30.0, 10.0, -2.9, False, "正常刹车不算急刹")[1:])
    res.append(("T5c 边界: dRel=0 不触发", None) + _tcase(
        C, "", 20.0, 0.0, 0.0, -8.0, False, "无目标")[1:])
    res.append(("T6 30m/s跟25m/s前车急刹@50m", None) + _tcase(
        C, "", 30.0, 50.0, 25.0, -4.0, False,
        "远距同向→新公式不误触（旧公式误触）")[1:])
    return res


# --------------------------------------------------------------------------
# HTML 输出
# --------------------------------------------------------------------------
def _svg(ys, label, color, w=860, h=180):
    ymax = max(0.05, float(np.max(np.abs(ys))) * 1.25)
    X = lambda i: 40 + (w - 60) * i / max(1, len(ys) - 1)
    Y = lambda v: h / 2 - (h / 2 - 16) * (v / ymax)
    pts = " ".join(f"{X(i):.1f},{Y(v):.1f}" for i, v in enumerate(ys))
    return f"""
<svg viewBox="0 0 {w} {h}" class="chart">
  <line x1="40" y1="{h/2}" x2="{w-20}" y2="{h/2}" stroke="#c8ccd4" stroke-dasharray="4 3"/>
  <polyline points="{pts}" fill="none" stroke="{color}" stroke-width="2"/>
  <text x="40" y="14" class="lbl">{label}</text>
  <text x="{w-20}" y="{h-4}" text-anchor="end" class="axis">帧 @100Hz</text>
  <text x="42" y="{h-4}" class="axis">m/s^2: -{ymax:.2f} … +{ymax:.2f}</text>
</svg>"""


def build_html(results, out_path):
    colors = ["#d33", "#389", "#3a3", "#a3a", "#e80", "#08c", "#d80", "#069"]
    rows, charts = [], []
    allpass = True
    for idx, (name, off, ok, note) in enumerate(results):
        allpass = allpass and ok
        rows.append(f'<tr><td>{name}</td><td class="{"pass" if ok else "fail"}">'
                    f'{"PASS" if ok else "FAIL"}</td><td>{note}</td></tr>')
        if off is not None and len(off) > 2:
            charts.append(f'<div class="card"><h3>{name}</h3>'
                          f'{_svg(np.asarray(off, dtype=float), name, colors[idx % len(colors)])}</div>')
    css = """
<style>
body{font-family:-apple-system,"PingFang SC",sans-serif;background:#f5f6f8;color:#222;margin:24px}
h1{font-size:20px} h2{font-size:15px;margin-top:28px}
table{border-collapse:collapse;width:100%;background:#fff;font-size:13px}
th,td{border:1px solid #dde;padding:7px 10px;text-align:left}
th{background:#eef}
.pass{color:#1a7f37;font-weight:700}.fail{color:#d33;font-weight:700}
.card{background:#fff;border:1px solid #dde;border-radius:8px;padding:12px;margin:14px 0}
.chart{width:100%;height:auto}
.lbl{font-size:11px;fill:#666}.axis{font-size:10px;fill:#999}
.summary{font-size:14px;padding:10px 14px;border-radius:8px;margin:12px 0;display:inline-block}
</style>"""
    html = f"""<!DOCTYPE html>
<html lang="zh"><head><meta charset="utf-8"><title>安全三修复离线验证</title>{css}</head>
<body>
<h1>安全三修复离线验证（TTC 闭合速度 / 画龙 / 弯道外漂）</h1>
<div class="summary" style="background:{"#e6f4e1" if allpass else "#fde8e8"};color:{"#1a7f37" if allpass else "#d33"}">
<b>{'全部通过' if allpass else '存在失败项'}：{sum(1 for r in results if r[2])}/{len(results)} 项 PASS</b>
&nbsp;|&nbsp; 被测对象（真实代码，非复刻）：<code>lib/latcontrol_torque.py</code> 救急修正块与稳态目标、
<code>lib/drive_helpers.py::get_lag_adjusted_curvature</code>、<code>controlsd.py</code> 前车急刹联动分支
</div>
<table><tr><th>场景</th><th>结果</th><th>数据</th></tr>{''.join(rows)}</table>
<h2>曲线（纵轴 = 期望横向加速度 m/s²，含 κ·v² 与救急修正）</h2>
{''.join(charts)}
</body></html>"""
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(html)
    return allpass


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default=os.path.join(HERE, "safety_fixes_validation.html"))
    args = ap.parse_args()

    _install_mocks()
    lat_mod = _load(os.path.join(CONTROLS, "lib", "latcontrol_torque.py"), "lat_under_test")
    dh_mod = _load(os.path.join(CONTROLS, "lib", "drive_helpers.py"), "dh_under_test")
    controlsd_src = open(os.path.join(CONTROLS, "controlsd.py"), encoding="utf-8").read()

    print("已加载真实实现：lib/latcontrol_torque.py / lib/drive_helpers.py / controlsd.py（AST）\n")

    results = [
        ("P1 直行稳定偏离收敛无振荡", *sc_p1_steady(lat_mod)),
        ("P2 释放系数对称（防弛张振荡）", *sc_p2_release(lat_mod)),
        ("P3 车道线置信度门控", *sc_p3_conf_gate(lat_mod)),
        ("P4 打灯变道抑制救急", *sc_p4_blinker(lat_mod)),
        ("P5 护栏误检钳制（回归）", *sc_p5_guardrail(lat_mod)),
        ("C1 弯道稳态满额 κ·v²", *sc_c1_curve_full(lat_mod)),
        ("C2 弯道+救急独立叠加", *sc_c2_curve_plus_correction(lat_mod)),
        ("C3 jerk 兜底（drive_helpers）", *sc_c3_jerk_cap(dh_mod)),
    ] + run_ttc(controlsd_src)

    for name, _h, ok, note in results:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name:28s} {note}")

    allpass = build_html(results, args.out)
    print(f"\n报告: {args.out}")
    print(f"结果: {'全部通过' if allpass else '存在失败项'}")
    sys.exit(0 if allpass else 1)


if __name__ == "__main__":
    main()
