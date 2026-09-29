#!/usr/bin/env python3
"""E2E 红绿灯补丁(第二步): 增加停稳态锁存 _tl_engaged, 保证停稳/起步期间 blended 模式持续,
避免 long_mpc acc 模式清零 E2E 预测导致红灯冲出。从当前 controlsd.py 原子改写。"""
import io, os

PATH = r"D:\Users\Desktop\红绿灯\controls\controlsd.py"
TMP  = PATH + ".tmp"

with open(PATH, "r", encoding="utf-8") as f:
    s = f.read()

def apply(old, new):
    global s
    assert old in s, f"[ASSERT FAIL] 未找到锚点:\n{old[:160]}"
    assert s.count(old) == 1, f"[ASSERT FAIL] 锚点出现{s.count(old)}次:\n{old[:160]}"
    s = s.replace(old, new, 1)

# 1) 初始化锁存标志
apply(
'''    # ========== 场景检测标志位 ==========
    self._scene_flags = {
      'traffic_light': False,        # 红绿灯场景（E2E 低速强制实验模式窗口）
      'narrow_road': False,          # 窄路场景
      'curve': False,                # 弯道场景
      'stopped_lead': False,         # 静止前车场景
      'high_speed_approach': False   # 高速接近场景
    }''',
'''    # ========== 场景检测标志位 ==========
    self._scene_flags = {
      'traffic_light': False,        # 红绿灯场景（E2E 低速强制实验模式窗口）
      'narrow_road': False,          # 窄路场景
      'curve': False,                # 弯道场景
      'stopped_lead': False,         # 静止前车场景
      'high_speed_approach': False   # 高速接近场景
    }
    self._tl_engaged = False         # E2E 红绿灯锁存：窗口内进入后保持 blended 到停稳/绿灯起步''')

# 2) TL 窗口块改为带锁存版本
apply(
'''      # ========== E2E 红绿灯：低速强制实验模式窗口（按实际车速判定） ==========
      # 5~70km/h 窗口内强制实验模式，让 E2E 模型的停止线/信号灯识别分支生效
      # （对齐 controlsd.py11 已验证写法，不依赖 trafficLightState 字段）。
      # 须放在弯道/视觉/雷达分支之前，保证 traffic_light 场景参与后续
      # other_scene_active 与接管防抖判定，experimental_mode 不会被清除。
      if TL_CONTROL_ENABLE and self.CP.openpilotLongitudinalControl and \\
         TL_LOW_SPEED_MIN_KPH <= v_ego_kph < TL_LOW_SPEED_MAX_KPH:
        self._scene_flags['traffic_light'] = True
        self.experimental_mode = True''',
'''      # ========== E2E 红绿灯：低速强制实验模式窗口（按实际车速判定） ==========
      # 5~70km/h 窗口内强制实验模式，让 E2E 模型的停止线/信号灯识别分支生效
      # （对齐 controlsd.py11 已验证写法，不依赖 trafficLightState 字段）。
      # 须放在弯道/视觉/雷达分支之前，保证 traffic_light 场景参与后续
      # other_scene_active 与接管防抖判定，experimental_mode 不会被清除。
      # 锁存(_tl_engaged)：一旦在窗口内进入过，停稳/低速(<70km/h)期间保持 engaged，
      # 使 blended 模式持续到车停稳乃至绿灯起步。原因：long_mpc 在 acc 模式会把模型
      # E2E 预测清零(只跟 vCruise/前车)，若停稳后(<5km/h)释放实验模式，车会按 vCruise
      # 重新起步、冲过红灯；blended 才跟随模型轨迹，红灯保持停、绿灯随模型预测自动起步。
      tl_window = TL_CONTROL_ENABLE and self.CP.openpilotLongitudinalControl and \\
                  TL_LOW_SPEED_MIN_KPH <= v_ego_kph < TL_LOW_SPEED_MAX_KPH
      if tl_window:
        self._tl_engaged = True
      elif v_ego_kph >= TL_LOW_SPEED_MAX_KPH:
        self._tl_engaged = False
      tl_active = tl_window or (self._tl_engaged and (CS.standstill or v_ego_kph < TL_LOW_SPEED_MAX_KPH))
      if tl_active:
        self._scene_flags['traffic_light'] = True
        self.experimental_mode = True''')

with io.open(TMP, "w", encoding="utf-8") as f:
    f.write(s)
os.replace(TMP, PATH)
print("OK: 锁存补丁已写入", PATH)
