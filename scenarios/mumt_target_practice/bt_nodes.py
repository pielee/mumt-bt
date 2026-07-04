"""
MUM-T 표적 사격 테스트 — 표적기(고도 유지 직진) vs 공격기(격추).

킬체인 검증용: WEZ 명중 판정 → ApplyDamage(HP 감소) → HP 0 → Falling
(엔진 정지 + 하드오버 추락) → AGL 임계 → Crashed 까지를 반격 없는 환경에서 관찰.

- F16_UAV1 = 표적기 (Team=Enemy): 이륙 → 직진 고도 유지 (OrbitLeader turn_rate=0)
- F16_UAV2 = 공격기 (Team=FriendlyUAV): 이륙 → EngageTarget (pure pursuit + WEZ 발사)
  → 표적 destroyed 후 선회 대기

새 노드 없음 — 기존 시나리오 노드를 import로 재사용:
  scenarios.mumt            : Takeoff, OrbitLeader
  scenarios.mumt_dogfight_1v1 : GatherCombatState, ConditionEnemyAlive, EngageTarget
"""

from modules.base_bt_nodes import (
    BTNodeList, Status, Sequence, ReactiveSequence, Fallback, ReactiveFallback,
)

# import 부수효과로 각 모듈의 BTNodeList 등록도 함께 수행됨
from scenarios.mumt.bt_nodes import Takeoff, OrbitLeader, HoldSetpoint
from scenarios.mumt_dogfight_1v1.bt_nodes import (
    GatherCombatState, ConditionEnemyAlive, EngageTarget,
)
