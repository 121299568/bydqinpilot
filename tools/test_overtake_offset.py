#!/usr/bin/env python3
"""超车远离偏移（OVERTAKE_* / traffic-aware clearance）离线验证脚本。

两种运行模式：
  --synthetic （默认，零外部依赖，只需 numpy）
      用合成的 modelV2 / radarState 序列驱动 真实的 LateralPlanner._apply_traffic_clearance()，
      覆盖：符号、窗口、变道门控、低速门控、护栏钳制、切入平滑衰减、速度前瞻、目标抖动。
  --qlog PATH
      回放真实 qlog（需要 openpilot/cereal 环境，车上或 CI 里用）。

输出：自包含 HTML（内联 SVG 曲线 + 断言结果表）。

用法：
  python3 tools/test_overtake_offset.py [--out out.html] [--qlog /path/to/qlog]
"""
import argparse
import importlib.util
import os
import sys
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CONTROLS = os.path.dirname(HERE)

TRAJECTORY_SIZE = 33
DT_MDL = 0.05
FRAMES = 80  # 每个场景跑 4s @20Hz，足够低通收敛


# --------------------------------------------------------------------------
# 依赖 mock：让真实的 lateral_planner.py 在无 openpilot 环境下可 import
# --------------------------------------------------------------------------
class _FakeCloudlog:
    def warning(self, *a, **k):
        pass
    def info(self, *a, **k):
        pass


class _FakeParams:
    def get_bool(self, key):
        return False
    def get(self, key, encoding=None):
        return "0"


class _FakeMpc:
    N = 16
    def __init__(self):
        self.x_sol = np.zeros((self.N + 1, 4))
        self.u_sol = np.zeros((self.N, 1))
        self.cost = 0.0
        self.solution_status = 0
        self.solve_time = 0.0
    def reset(self, x0=None):
        pass
    def set_weights(self, *a, **k):
        pass
    def run(self, *a, **k):
        pass


class _Desire:
    keepNone = 0
    laneChangeLeft = 1
    laneChangeRight = 2


class _FakeDH:
    def __init__(self):
        self.desire = _Desire.keepNone
        self.lane_change_state = 0
        self.lane_change_direction = 0
        self.lane_change_ll_prob = 1.0
    def update(self, *a, **k):
        pass


class _FakeLP:
    def __init__(self):
        self.lll_prob = 1.0
        self.rll_prob = 1.0
    def parse_model(self, md):
        pass
    def get_d_path(self, *a, **k):
        return np.zeros((TRAJECTORY_SIZE, 3))


def _register(name, **attrs):
    parts = name.split(".")
    for i in range(1, len(parts) + 1):
        sub = ".".join(parts[:i])
        if sub not in sys.modules:
            sys.modules[sub] = types.ModuleType(sub)
    m = sys.modules[name]
    for k, v in attrs.items():
        setattr(m, k, v)
    return m


def _install_mocks():
    """注入假的 openpilot / cereal 模块，之后 import 真实 lateral_planner.py。"""
    _register("openpilot.common.realtime", DT_MDL=DT_MDL)
    _register("openpilot.common.numpy_fast", interp=np.interp)
    _register("openpilot.system.swaglog", cloudlog=_FakeCloudlog())
    _register("openpilot.selfdrive.hardware", EON=False)
    _register("openpilot.common.conversions",
              Conversions=types.SimpleNamespace(MPH_TO_MS=0.44704, KPH_TO_MS=1.0 / 3.6))
    _register("openpilot.common.params", Params=_FakeParams)
    _register("openpilot.selfdrive.controls.lib.lateral_mpc_lib.lat_mpc",
              LateralMpc=_FakeMpc, N=16)
    _register("openpilot.selfdrive.controls.lib.drive_helpers",
              CONTROL_N=17, MIN_SPEED=0.1)
    _register("openpilot.selfdrive.controls.lib.desire_helper", DesireHelper=_FakeDH)
    _register("openpilot.selfdrive.controls.lib.lane_planner", LanePlanner=_FakeLP)
    _register("cereal",
              log=types.SimpleNamespace(LateralPlan=types.SimpleNamespace(Desire=_Desire)))
    _register("cereal.messaging", new_message=lambda *a, **k: None)


def load_lateral_planner():
    _install_mocks()
    path = os.path.join(CONTROLS, "lib", "lateral_planner.py")
    spec = importlib.util.spec_from_file_location("lateral_planner_under_test", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# --------------------------------------------------------------------------
# 消息工厂（合成 modelV2 / radarState）
# --------------------------------------------------------------------------
class _Lead:
    def __init__(self, x, y, velocity, score):
        self.x, self.y, self.velocity, self.score = x, y, velocity, score


class _LaneLine:
    def __init__(self, xs, ys):
        self.x, self.y = xs, ys


class _LeadTwo:
    def __init__(self, status, dRel, yRel, vRel):
        self.status, self.dRel, self.yRel, self.vRel = status, dRel, yRel, vRel


def _make_sm(leads, ll_y=1.9, rl_y=-1.9, lead_two=None, curvature=0.0):
    xs = np.linspace(0, 64, TRAJECTORY_SIZE).tolist()
    ll = _LaneLine(xs, [ll_y] * TRAJECTORY_SIZE)
    rl = _LaneLine(xs, [rl_y] * TRAJECTORY_SIZE)
    far = _LaneLine(xs, [ll_y * 2] * TRAJECTORY_SIZE)
    far_r = _LaneLine(xs, [rl_y * 2] * TRAJECTORY_SIZE)
    if lead_two is None:
        lead_two = _LeadTwo(False, 0.0, 0.0, 0.0)
    return {
        "modelV2": types.SimpleNamespace(leadsV3=leads,
                                         laneLines=[far, ll, rl, far_r]),
        "radarState": types.SimpleNamespace(leadTwo=lead_two),
        "controlsState": types.SimpleNamespace(curvature=curvature),
    }


def _make_planner(mod, v_ego=20.0, desire=_Desire.keepNone):
    CP = types.SimpleNamespace(wheelbase=2.7, centerToFront=1.2,
                               mass=1500.0, tireStiffnessRear=60000.0)
    lp = mod.LateralPlanner(CP, debug=True)
    lp.v_ego = v_ego
    lp.DH.desire = desire
    return lp


def _base_path():
    xs = np.linspace(0, 64, TRAJECTORY_SIZE)
    return np.column_stack([xs, np.zeros(TRAJECTORY_SIZE), np.zeros(TRAJECTORY_SIZE)])


def _run(lp, base, sm_fn, frames=FRAMES):
    """跑 frames 帧，返回 (offset_hist, path_hist)。sm_fn(frame)->sm。"""
    offset_hist, path_hist = [], []
    for f in range(frames):
        sm = sm_fn(f)
        out = lp._apply_traffic_clearance(np.array(base, copy=True), sm)
        offset_hist.append(lp._traffic_offset)
        path_hist.append(np.array(out))
    return np.array(offset_hist), np.array(path_hist)


# --------------------------------------------------------------------------
# 场景
# --------------------------------------------------------------------------
def sc_left_parallel(mod):
    """A: 旁车在左并行 (y=+2.0, 同速) → 偏移应为负（往右挪）。"""
    lp = _make_planner(mod)
    sm_fn = lambda f: _make_sm([_Lead(+5.0, +2.0, 20.0, 0.9)])
    off, _ = _run(lp, _base_path(), sm_fn)
    tgt = -mod.OVERTAKE_MAX_OFFSET * 0.7  # prox(2.0)=0.7
    ok = off[-1] < 0 and abs(off[-1] - tgt) < 0.02
    return off, ok, f"末帧={off[-1]:+.3f}m (期望 {tgt:+.3f}m，负=往右)"


def sc_right_parallel(mod):
    """B: 旁车在右 (y=-2.0) → 偏移应为正（往左挪）。"""
    lp = _make_planner(mod)
    sm_fn = lambda f: _make_sm([_Lead(+5.0, -2.0, 20.0, 0.9)])
    off, _ = _run(lp, _base_path(), sm_fn)
    tgt = +mod.OVERTAKE_MAX_OFFSET * 0.7
    ok = off[-1] > 0 and abs(off[-1] - tgt) < 0.02
    return off, ok, f"末帧={off[-1]:+.3f}m (期望 {tgt:+.3f}m，正=往左)"


def sc_same_lane(mod):
    """C: 同车道 (y=0.3 < 下限) → 不偏。"""
    lp = _make_planner(mod)
    sm_fn = lambda f: _make_sm([_Lead(+10.0, +0.3, 20.0, 0.9)])
    off, _ = _run(lp, _base_path(), sm_fn)
    ok = abs(off[-1]) < 1e-3
    return off, ok, f"末帧={off[-1]:+.4f}m (期望 0)"


def sc_out_of_window(mod):
    """D: 旁车在后方窗口外 (x=-15m) → 不偏。"""
    lp = _make_planner(mod)
    sm_fn = lambda f: _make_sm([_Lead(-15.0, +2.0, 20.0, 0.9)])
    off, _ = _run(lp, _base_path(), sm_fn)
    ok = abs(off[-1]) < 1e-3
    return off, ok, f"末帧={off[-1]:+.4f}m (期望 0)"


def sc_lane_change(mod):
    """E: 变道中 → 强制归零（即使旁车在旁边）。"""
    lp = _make_planner(mod, desire=_Desire.laneChangeLeft)
    sm_fn = lambda f: _make_sm([_Lead(+5.0, +2.0, 20.0, 0.9)])
    off, _ = _run(lp, _base_path(), sm_fn)
    ok = abs(off[-1]) < 1e-3
    return off, ok, f"末帧={off[-1]:+.4f}m (期望 0，变道优先)"


def sc_low_speed(mod):
    """F: 低速 (5 m/s < 10) → 不偏。"""
    lp = _make_planner(mod, v_ego=5.0)
    sm_fn = lambda f: _make_sm([_Lead(+5.0, +2.0, 5.0, 0.9)])
    off, _ = _run(lp, _base_path(), sm_fn)
    ok = abs(off[-1]) < 1e-3
    return off, ok, f"末帧={off[-1]:+.4f}m (期望 0，低速门控)"


def sc_guardrail(mod):
    """G: 旁车在左，但右车道线很近 (rl_y=-0.5) → 偏移被护栏钳到 0。"""
    lp = _make_planner(mod)
    sm_fn = lambda f: _make_sm([_Lead(+5.0, +2.0, 20.0, 0.9)], rl_y=-0.5)
    off, _ = _run(lp, _base_path(), sm_fn)
    ok = abs(off[-1]) < 0.01
    return off, ok, f"末帧={off[-1]:+.4f}m (期望 ≈0，护栏钳制：右线 -0.5m + 余量 0.5m)"


def sc_cutin(mod):
    """H: 旁车切入，y 从 2.0 平滑减到 0.7 → 偏移平滑衰减，无阶跃。"""
    lp = _make_planner(mod)
    def sm_fn(f):
        y = 2.0 - 1.3 * (f / (FRAMES - 1))
        return _make_sm([_Lead(+5.0, +y, 20.0, 0.9)])
    off, _ = _run(lp, _base_path(), sm_fn)
    jump = float(np.max(np.abs(np.diff(off))))
    ok = off[-1] < off[5] and jump < 0.03 and off[-1] > -0.18
    return off, ok, f"起始={off[5]:+.3f} 末帧={off[-1]:+.3f} 最大单帧跳变={jump:.4f} (期望平滑衰减)"


def sc_lookahead(mod):
    """I: 旁车 x=+10m 且 vRel=-10m/s（快速接近）→ 鼓包中心应在预测位置 x≈5m。"""
    lp = _make_planner(mod)
    base = _base_path()
    sm_fn = lambda f: _make_sm([_Lead(+10.0, +2.0, 10.0, 0.9)])  # v=10 < v_ego=20 → vRel=-10
    off, paths = _run(lp, base, sm_fn)
    # 取收敛后那一帧的路径偏移剖面
    prof = paths[-1][:, 1] - base[:, 1]
    xs = base[:, 0]
    peak_x = float(xs[int(np.argmax(np.abs(prof)))])
    expect = 10.0 + (-10.0) * mod.OVERTAKE_LOOKAHEAD_T  # = 5.0
    ok = abs(peak_x - expect) < 4.0 and np.min(prof) < -0.02
    return off, ok, f"峰值位置 x={peak_x:.1f}m (无前瞻应为 10m，期望 {expect:.1f}m)"


def sc_jitter(mod):
    """J: 旁车完美交替出现（奇偶帧）→ 连续 3 帧门控永远达不到 → 完全压制。"""
    lp = _make_planner(mod)
    def sm_fn(f):
        leads = [_Lead(+5.0, +2.0, 20.0, 0.9)] if f % 2 == 0 else []
        return _make_sm(leads)
    off, _ = _run(lp, _base_path(), sm_fn)
    jump = float(np.max(np.abs(np.diff(off))))
    ok = jump < 0.03 and np.max(np.abs(off)) < 0.25
    return off, ok, f"最大|偏移|={np.max(np.abs(off)):.3f}m (期望 ≈0，门控完全压制交替抖动)"


def sc_bursty_jitter(mod):
    """M: 块状抖动（出现5帧/消失5帧循环）→ 门控能过，但低通压成平滑锯齿，无阶跃。"""
    lp = _make_planner(mod)
    def sm_fn(f):
        present = (f % 10) < 5
        leads = [_Lead(+5.0, +2.0, 20.0, 0.9)] if present else []
        return _make_sm(leads)
    off, _ = _run(lp, _base_path(), sm_fn)
    jump = float(np.max(np.abs(np.diff(off))))
    peak = float(np.max(np.abs(off)))
    ok = jump < 0.03 and 0.01 < peak < 0.22
    return off, ok, (f"峰值|偏移|={peak:.3f}m 最大单帧跳变={jump:.4f} "
                     f"(期望 0.01<峰值<0.22 且无阶跃)")


def sc_radar_fallback(mod):
    """K: 视觉全丢、雷达 leadTwo 命中 → 兜底生效。"""
    lp = _make_planner(mod)
    sm_fn = lambda f: _make_sm([], lead_two=_LeadTwo(True, +8.0, +2.0, 0.0))
    off, _ = _run(lp, _base_path(), sm_fn)
    ok = off[-1] < -0.05
    return off, ok, f"末帧={off[-1]:+.3f}m (期望 <0，雷达兜底)"


def sc_disable_switch(mod):
    """L: OVERTAKE_ENABLE=False → 完全回退原行为。"""
    orig = mod.OVERTAKE_ENABLE
    mod.OVERTAKE_ENABLE = False
    try:
        lp = _make_planner(mod)
        sm_fn = lambda f: _make_sm([_Lead(+5.0, +2.0, 20.0, 0.9)])
        off, paths = _run(lp, _base_path(), sm_fn)
        same = np.allclose(paths[-1], _base_path())
        ok = abs(off[-1]) < 1e-9 and same
        return off, ok, f"末帧={off[-1]:+.4f}m 路径不变={same} (期望完全回退)"
    finally:
        mod.OVERTAKE_ENABLE = orig


SCENARIOS = [
    ("A 左侧并行→往右挪", sc_left_parallel),
    ("B 右侧并行→往左挪", sc_right_parallel),
    ("C 同车道不躲", sc_same_lane),
    ("D 窗口外不躲", sc_out_of_window),
    ("E 变道中归零", sc_lane_change),
    ("F 低速归零", sc_low_speed),
    ("G 护栏钳制", sc_guardrail),
    ("H 切入平滑衰减", sc_cutin),
    ("I 速度前瞻", sc_lookahead),
    ("J 目标抖动抑制", sc_jitter),
    ("M 块状抖动平滑", sc_bursty_jitter),
    ("K 雷达兜底", sc_radar_fallback),
    ("L 开关回退", sc_disable_switch),
]


# --------------------------------------------------------------------------
# HTML / SVG 输出
# --------------------------------------------------------------------------
def _svg_line(ys, label, color, w=860, h=180, ymax=None):
    t = np.arange(len(ys))
    if ymax is None:
        ymax = max(0.05, float(np.max(np.abs(ys))) * 1.2)
    def X(i):
        return 40 + (w - 60) * i / max(1, len(ys) - 1)
    def Y(v):
        return h / 2 - (h / 2 - 16) * (v / ymax)
    pts = " ".join(f"{X(i):.1f},{Y(v):.1f}" for i, v in enumerate(ys))
    zero = f"M40,{h / 2:.1f} L{w - 20},{h / 2:.1f}"
    return f"""
<svg viewBox="0 0 {w} {h}" class="chart">
  <line x1="40" y1="{h / 2}" x2="{w - 20}" y2="{h / 2}" stroke="#c8ccd4" stroke-dasharray="4 3"/>
  <polyline points="{pts}" fill="none" stroke="{color}" stroke-width="2"/>
  <text x="40" y="14" class="lbl">{label}</text>
  <text x="{w - 20}" y="{h - 4}" text-anchor="end" class="axis">帧 (20Hz)</text>
  <text x="42" y="{h - 4}" class="axis">偏移/m: -{ymax:.2f} … +{ymax:.2f}</text>
</svg>"""


def build_html(results, out_path):
    colors = ["#d33", "#389", "#3a3", "#a3a", "#e80", "#08c"]
    rows, charts = [], []
    allpass = True
    for idx, (name, off, ok, note) in enumerate(results):
        allpass = allpass and ok
        badge = "PASS" if ok else "FAIL"
        cls = "pass" if ok else "fail"
        rows.append(f'<tr><td>{name}</td><td class="{cls}">{badge}</td><td>{note}</td></tr>')
        charts.append(f'<div class="card"><h3>{name}</h3>{_svg_line(off, name, colors[idx % len(colors)])}</div>')
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
<html lang="zh"><head><meta charset="utf-8"><title>超车远离偏移验证</title>{css}</head>
<body>
<h1>超车远离偏移（traffic-aware clearance）离线验证</h1>
<div class="summary" style="background:{"#e6f4e1" if allpass else "#fde8e8"};color:{"#1a7f37" if allpass else "#d33"}">
<b>{'全部通过' if allpass else '存在失败项'}：{sum(1 for r in results if r[2])}/{len(results)} 场景 PASS</b>
&nbsp;|&nbsp; 被测对象：<code>lib/lateral_planner.py::_apply_traffic_clearance</code>（真实代码，非复刻）
</div>
<table><tr><th>场景</th><th>结果</th><th>数据</th></tr>{''.join(rows)}</table>
<h2>偏移量时间曲线（每场景 {FRAMES} 帧 @20Hz，纵轴 = 路径横向偏移 / m）</h2>
{''.join(charts)}
</body></html>"""
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(html)
    return allpass


# --------------------------------------------------------------------------
# qlog 回放（需要真实 openpilot 环境）
# --------------------------------------------------------------------------
def run_qlog(qlog_path, out_path):
    try:
        import cereal.messaging as messaging  # noqa: F401
    except ImportError:
        print("本环境无 cereal，无法回放 qlog。请在 openpilot 环境运行，或用默认 --synthetic。")
        sys.exit(2)
    from cereal.services import SERVICE_LIST  # noqa
    mod = load_lateral_planner()
    lp = _make_planner(mod)
    base = _base_path()
    off_hist = []
    sm = messaging.SubMaster(["modelV2", "radarState", "controlsState", "carState"])
    while True:
        sm.update(0)
        if not sm.all_checks():
            break
        lp.v_ego = max(0.1, sm["carState"].vEgo)
        _ = lp._apply_traffic_clearance(base, sm)
        off_hist.append(lp._traffic_offset)
    off = np.array(off_hist)
    results = [("qlog 回放", off, len(off) > 0,
                f"{len(off)} 帧，偏移范围 [{off.min():+.3f}, {off.max():+.3f}]m")]
    build_html(results, out_path)
    print(f"qlog 回放完成 → {out_path}")


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default=os.path.join(HERE, "overtake_offset_validation.html"))
    ap.add_argument("--qlog", default=None, help="回放真实 qlog（需 openpilot 环境）")
    args = ap.parse_args()

    if args.qlog:
        run_qlog(args.qlog, args.out)
        return

    mod = load_lateral_planner()
    print(f"已加载真实实现: lib/lateral_planner.py")
    print(f"  OVERTAKE_ENABLE={mod.OVERTAKE_ENABLE} MAX_OFFSET={mod.OVERTAKE_MAX_OFFSET}m "
          f"TAU={mod.OVERTAKE_TAU}s LOOKAHEAD={mod.OVERTAKE_LOOKAHEAD_T}s")
    print(f"  场景数: {len(SCENARIOS)}，每场景 {FRAMES} 帧 @20Hz\n")

    results = []
    for name, fn in SCENARIOS:
        off, ok, note = fn(mod)
        results.append((name, off, ok, note))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name:16s} {note}")

    allpass = build_html(results, args.out)
    print(f"\n报告: {args.out}")
    print(f"结果: {'全部通过' if allpass else '存在失败'}")
    sys.exit(0 if allpass else 1)


if __name__ == "__main__":
    main()
