"""
MUM-T 무장 발사 테스트 — 두 UAV가 이륙 후 직진하며 시간표대로 기총·미사일을 발사.

교전 시나리오와 달리 **조준(WEZ)·팀 지정과 무관하게 무조건 발사**한다.
목적: BT → /aircraft/setpoint → bridge → UDP 5010 → UWeaponComponent →
OnGunFiringChanged/OnMissileFired(BP 연출) 체인이 실제로 동작하는지 눈으로 확인.

- 기총: start_delay_s 후부터 gun_burst_s 켜고 gun_pause_s 끄는 점사 무한 반복
- 미사일: start_delay_s 후 즉시 1발, 이후 missile_interval_s 간격으로 총 missile_shots발
  (fire_id는 1부터 — 0은 미발사 초기값)
- 팀/표적 불필요: 허공 발사여도 UE가 미사일을 소모하고 연출 이벤트를 발화함.
  단 UAV 폰에 UWeaponComponent 부착 + BP 훅(트레이서/미사일 연출) 배선은 전제.
"""

import math
import time

from custom_msgs.msg import AircraftSetpoint

from modules.base_bt_nodes import (
    BTNodeList, Status, Node, Sequence, ReactiveSequence, Fallback, ReactiveFallback,
)
from modules.base_bt_nodes_ros import ActionWithROSTopic

# 기존 노드 재사용 (파일 수정 없이 import만)
from scenarios.mumt.bt_nodes import (
    Takeoff, _alt_m, _setpoint, _own_routing_name, _SETPOINT_TOPIC,
)
from scenarios.mumt_dogfight_1v1.bt_nodes import GatherCombatState

BTNodeList.ACTION_NODES.extend(["WeaponTestFire"])


class WeaponTestFire(ActionWithROSTopic):
    """직진 유지하며 시간표대로 발사. 항상 RUNNING (사용자가 관전 후 종료).

    XML 파라미터:
      own_name           : 기체 이름 (라우팅)
      heading_deg        : 유지 헤딩 (기본 90)
      target_speed_mps   : 유지 속도 (기본 200)
      start_delay_s      : 노드 시작 후 첫 발사까지 대기 (기본 3)
      gun_burst_s        : 기총 점사 길이 (기본 3)
      gun_pause_s        : 점사 사이 휴지 (기본 3)
      missile_interval_s : 미사일 발사 간격 (기본 8)
      missile_shots      : 발사할 미사일 수 (기본 3 = UE 기본 장탄)
    고도는 첫 틱의 자기 고도를 래칭해 유지 (Takeoff가 올려놓은 고도 그대로)."""

    def __init__(self, name, agent, own_name="",
                 heading_deg=90.0, target_speed_mps=200.0,
                 start_delay_s=3.0, gun_burst_s=3.0, gun_pause_s=3.0,
                 missile_interval_s=8.0, missile_shots=3):
        super().__init__(name, agent, (AircraftSetpoint, _SETPOINT_TOPIC))
        self._own_name = own_name or (getattr(agent, "agent_id", "") or "").strip("/")
        self._hdg      = float(heading_deg)
        self._speed    = float(target_speed_mps)
        self._delay    = float(start_delay_s)
        self._burst    = float(gun_burst_s)
        self._pause    = float(gun_pause_s)
        self._interval = float(missile_interval_s)
        self._shots    = int(missile_shots)
        self._t0       = None      # 노드 첫 틱 시각
        self._hold_alt = None      # 유지 고도 (첫 틱 래칭)
        self._fire_id  = 0         # 0=미발사, 1부터 발사
        self._last_msl = None      # 마지막 미사일 발사 시각

    def _build_message(self, agent, blackboard):
        own = blackboard.get("own_state")
        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now
        if self._hold_alt is None and own:
            self._hold_alt = _alt_m(own)

        t = now - self._t0 - self._delay          # 발사 시간표 기준 시각 (<0 = 대기)

        # 기총: 점사 duty cycle 무한 반복
        cycle = self._burst + self._pause
        gun = bool(t >= 0.0 and cycle > 0.0 and (t % cycle) < self._burst)

        # 미사일: t>=0 부터 interval 간격으로 shots발
        fired = False
        if (t >= 0.0 and self._fire_id < self._shots
                and (self._last_msl is None or now - self._last_msl >= self._interval)):
            self._fire_id += 1
            self._last_msl = now
            fired = True

        agent.ros_bridge.node.get_logger().info(
            f"[WeaponTestFire] t={max(t, 0.0):5.1f}s gun={'ON ' if gun else 'off'} "
            f"msl {self._fire_id}/{self._shots}{' ★발사' if fired else ''}")

        alt = self._hold_alt if self._hold_alt is not None else 1000.0
        msg = _setpoint(_own_routing_name(own, self._own_name),
                        self._hdg, alt, target_speed=self._speed)
        msg.gun_firing      = gun
        msg.missile_fire_id = self._fire_id
        return msg

    def _interpret_publish(self, msg, agent, blackboard) -> Status:
        return Status.RUNNING     # 테스트는 끝나지 않음 — 관전 후 Ctrl+C
