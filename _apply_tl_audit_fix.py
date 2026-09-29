#!/usr/bin/env python3
"""E2E 红绿灯审计修复: TL 窗口同步 step() 的 radarUnavailable&&dp_0813 关闭条件,
并让锁存 _tl_engaged 也受 TL_CONTROL_ENABLE 约束。从当前 controlsd.py 原子改写。"""
import io, os

PATH = r"D:\Users\Desktop\红绿灯\controls\controlsd.py"
TMP  = PATH + ".tmp"

with open(PATH, "r", encoding="utf-8") as f:
    s = f.read()

def apply(old, new):
    global s
    assert old in s, f"[ASSERT FAIL] 未找到锚点:\n{old[:200]}"
    assert s.count(old) == 1, f"[ASSERT FAIL] 锚点出现{s.count(old)}次"
    s = s.replace(old, new, 1)

apply(
'''      tl_window = TL_CONTROL_ENABLE and self.CP.openpilotLongitudinalControl and \\
                  TL_LOW_SPEED_MIN_KPH <= v_ego_kph < TL_LOW_SPEED_MAX_KPH
      if tl_window:
        self._tl_engaged = True
      elif v_ego_kph >= TL_LOW_SPEED_MAX_KPH:
        self._tl_engaged = False
      tl_active = tl_window or (self._tl_engaged and (CS.standstill or v_ego_kph < TL_LOW_SPEED_MAX_KPH))''',
'''      # 与 step() 一致：雷达缺失 + dp_0813 模型时原作者刻意关闭实验模式，TL 不应重新打开
      tl_window = TL_CONTROL_ENABLE and self.CP.openpilotLongitudinalControl and \\
                  (not (self.CP.radarUnavailable and self.dp_0813)) and \\
                  TL_LOW_SPEED_MIN_KPH <= v_ego_kph < TL_LOW_SPEED_MAX_KPH
      if tl_window:
        self._tl_engaged = True
      elif v_ego_kph >= TL_LOW_SPEED_MAX_KPH:
        self._tl_engaged = False
      tl_active = tl_window or (TL_CONTROL_ENABLE and self._tl_engaged and \\
                  (CS.standstill or v_ego_kph < TL_LOW_SPEED_MAX_KPH))''')

with io.open(TMP, "w", encoding="utf-8") as f:
    f.write(s)
os.replace(TMP, PATH)
print("OK: 审计修复已写入", PATH)
