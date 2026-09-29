#!/usr/bin/env python3
"""从干净基线 controlsd.py_bak_20260918_no_tl 原子重写 E2E 红绿灯(TL)功能。
每次替换都做断言(找不到锚点即报错), 写临时文件后 os.replace 覆盖, 避免外部编辑器回写造成的混杂状态。"""
import io, os

BASE = r"D:\Users\Desktop\红绿灯\controls\controlsd.py_bak_20260918_no_tl"
OUT  = r"D:\Users\Desktop\红绿灯\controls\controlsd.py"
TMP  = OUT + ".tmp"

with open(BASE, "r", encoding="utf-8") as f:
    s = f.read()

def apply(old, new):
    global s
    assert old in s, f"[ASSERT FAIL] 未找到锚点:\n{old[:120]}"
    # 防止同一锚点被应用两次
    assert s.count(old) == 1, f"[ASSERT FAIL] 锚点出现多次({s.count(old)}):\n{old[:120]}"
    s = s.replace(old, new, 1)

# E1 文件头"已实现"列表补第 4 条
apply(
'''#   1) 前方静态车辆识别（雷达 leadOne + 视觉 leadsV3 双源融合）
#   2) 弯道减速（模型曲率前瞻预判）
#   3) 高速跟车 / 前车起步跟随 / 积极加速''',
'''#   1) 前方静态车辆识别（雷达 leadOne + 视觉 leadsV3 双源融合）
#   2) 弯道减速（模型曲率前瞻预判）
#   3) 高速跟车 / 前车起步跟随 / 积极加速
#   4) E2E 红绿灯停起步（5~70km/h 强制实验模式，模型停止线/信号灯分支）''')

# E2 常量区：ACC/OP 接管防抖之后新增 TL 常量
apply(
'''ACC_TAKEOVER_IN_T = 0.3   # 切入 OP 纵向前，前车/弯道场景需持续的时间 (s)
ACC_TAKEOVER_OUT_T = 1.5  # 交回原车 ACC 前，场景需消失的时间 (s)''',
'''ACC_TAKEOVER_IN_T = 0.3   # 切入 OP 纵向前，前车/弯道场景需持续的时间 (s)
ACC_TAKEOVER_OUT_T = 1.5  # 交回原车 ACC 前，场景需消失的时间 (s)

# ===== E2E 红绿灯识别（低速强制实验模式窗口，按实际车速判定） =====
# 原理：E2E 纵向模型内置停止线/信号灯分支，但仅在实验模式(blended/e2e)下生效。
# 市区 5~70km/h 窗口内强制实验模式并让 OP 纵向接管：模型看到红灯/停止线即自行刹停，
# 变绿灯后 E2E 轨迹恢复 -> cruiseControl.resume 自动起步。
# 不依赖 hybrid_modeld 的 trafficLightState 字段（多数 fork 无此字段，会静默降级）；
# 写法对齐 controlsd.py11 已验证可用的机制。
TL_CONTROL_ENABLE = True        # E2E 红绿灯总开关；False 彻底关闭
TL_LOW_SPEED_MIN_KPH = 5.0      # 低于此车速不强制（停车/蠕行，避免误触发）
TL_LOW_SPEED_MAX_KPH = 70.0     # 低于此车速强制实验模式 + OP 纵向接管（秦PLUS市区红绿灯窗口）''')

# E3 场景标志位加回 traffic_light
apply(
'''    self._scene_flags = {
      'narrow_road': False,          # 窄路场景
      'curve': False,                # 弯道场景
      'stopped_lead': False,         # 静止前车场景
      'high_speed_approach': False   # 高速接近场景
    }''',
'''    self._scene_flags = {
      'traffic_light': False,        # 红绿灯场景（E2E 低速强制实验模式窗口）
      'narrow_road': False,          # 窄路场景
      'curve': False,                # 弯道场景
      'stopped_lead': False,         # 静止前车场景
      'high_speed_approach': False   # 高速接近场景
    }''')

# E4 state_control：ACC/OP 迟滞之后、弯道分支之前插入 TL 窗口判定
apply(
'''      if self._last_use_stock_acc:
        use_stock_acc = ACC_ON_KPH < v_ego_kph
      else:
        use_stock_acc = ACC_OFF_KPH < v_ego_kph

      # ========== 弯道预判减速（简化版）==========''',
'''      if self._last_use_stock_acc:
        use_stock_acc = ACC_ON_KPH < v_ego_kph
      else:
        use_stock_acc = ACC_OFF_KPH < v_ego_kph

      # ========== E2E 红绿灯：低速强制实验模式窗口（按实际车速判定） ==========
      # 5~70km/h 窗口内强制实验模式，让 E2E 模型的停止线/信号灯识别分支生效
      # （对齐 controlsd.py11 已验证写法，不依赖 trafficLightState 字段）。
      # 须放在弯道/视觉/雷达分支之前，保证 traffic_light 场景参与后续
      # other_scene_active 与接管防抖判定，experimental_mode 不会被清除。
      if TL_CONTROL_ENABLE and self.CP.openpilotLongitudinalControl and \\
         TL_LOW_SPEED_MIN_KPH <= v_ego_kph < TL_LOW_SPEED_MAX_KPH:
        self._scene_flags['traffic_light'] = True
        self.experimental_mode = True

      # ========== 弯道预判减速（简化版）==========''')

# E5 视觉分支 other_scene_active 纳入 traffic_light
apply(
'''      current_lead_status = False
      other_scene_active = self._scene_flags.get('curve', False)''',
'''      current_lead_status = False
      other_scene_active = self._scene_flags.get('traffic_light', False) or self._scene_flags.get('curve', False)''')

# E6 雷达静态前车 elif 纳入 traffic_light（避免 TL 窗口误清静态前车标志）
apply(
'''      elif not self._scene_flags.get('curve', False):
        self._static_lead_active = False''',
'''      elif not self._scene_flags.get('traffic_light', False) and not self._scene_flags.get('curve', False):
        self._static_lead_active = False''')

# E7 [关键] ACC/OP 接管防抖 takeover_req 纳入 traffic_light，否则窗口内 OP 纵向不接管
apply(
'''      takeover_req = current_lead_status or self._scene_flags['narrow_road'] or self._scene_flags['curve']''',
'''      takeover_req = current_lead_status or self._scene_flags['narrow_road'] or self._scene_flags['curve'] \\
          or self._scene_flags['traffic_light']''')

# E8 调试日志 [DBG] 增加 TL 段
apply(
'''        f"SS act={self._static_lead_active} d={self._static_lead_dRel:.1f} vr={self._static_lead_vRel:.1f} | "
        f"en={self.enabled} exp={self.experimental_mode} long={CC.longActive} "
        f"opLong={self.CP.openpilotLongitudinalControl}")''',
'''        f"SS act={self._static_lead_active} d={self._static_lead_dRel:.1f} vr={self._static_lead_vRel:.1f} | "
        f"TL act={self._scene_flags.get('traffic_light', False)} | "
        f"en={self.enabled} exp={self.experimental_mode} long={CC.longActive} "
        f"opLong={self.CP.openpilotLongitudinalControl}")''')

with io.open(TMP, "w", encoding="utf-8") as f:
    f.write(s)
os.replace(TMP, OUT)
print("OK: 已原子写入", OUT, "总行数", s.count(chr(10)) + 1)
