# mumt_dogfight_1v1 — 1v1 UAV 교전 시나리오

F16_UAV1(적, Enemy) vs F16_UAV2(아군, FriendlyUAV). 양쪽이 이륙 후 서로를
pure pursuit로 추적하며 WEZ 안에서 기총·미사일을 발사한다. 명중 판정(원추/거리/대미지)은
UE의 `UWeaponComponent`가 하고, BT는 방아쇠 조건만 판단한다.

## 전제 (에디터에서 1회 설정)

- UE 레벨의 두 UAV 폰에 **UHealthComponent + UWeaponComponent 부착** 완료 상태
  - F16_UAV1 → `Team = Enemy`
  - F16_UAV2 → `Team = Friendly UAV`
- Phase 3 빌드 적용 (5010 무장 파싱 + 5006 hp/team/destroyed/missile_count 브로드캐스트)
- `custom_msgs` 재빌드 완료 (gun_firing/missile_fire_id 추가 후 colcon build 필수)

## 실행 커맨드

```bash
# 0) (msg 변경 후 1회) ROS 워크스페이스 재빌드
cd ~/dev/mumt_ros_ws && colcon build --packages-select custom_msgs mumt_ros_bridge
source install/setup.bash

# 1) UE 에디터에서 PIE 실행 (RL_30 등)

# 2) 브리지
ros2 run mumt_ros_bridge bridge_node

# 3) UAV1 (적) BT — 터미널 1 (py_bt_ros 루트에서)
python main.py --config scenarios/mumt_dogfight_1v1/configs/dogfight_uav1.yaml

# 4) UAV2 (아군) BT — 터미널 2
python main.py --config scenarios/mumt_dogfight_1v1/configs/dogfight_uav2.yaml
```

## 트리 구조 (양쪽 동일, own_name만 다름)

```
ReactiveSequence
├─ GatherCombatState          # own + 적 팀 생존 목록 (5006 team/destroyed 파싱)
└─ Sequence
   ├─ Takeoff                 # 기존 노드 재사용 (스폰 대비 상대 상승 판정)
   └─ Fallback
      ├─ Sequence
      │  ├─ ConditionEnemyAlive
      │  └─ EngageTarget      # pure pursuit + WEZ 방아쇠, 표적 destroyed → SUCCESS
      └─ OrbitLeader          # 적 전멸 → 선회 대기 (기존 노드 재사용)
```

## EngageTarget 튜닝 파라미터 (XML 속성)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `engage_speed_mps` | 250 | 추적 목표속도 (UE 오토스로틀) |
| `missile_range_m` | 8000 | 미사일 방아쇠 거리 |
| `missile_bearing_deg` | 15 | 미사일 방아쇠 방위오차 (UE WEZ 원추 15°와 동일) |
| `missile_cooldown_sec` | 10 | 미사일 발사 간 최소 간격 (3발 순간 소진 방지) |
| `gun_range_m` | 1500 | 기총 방아쇠 거리 |
| `gun_bearing_deg` | 5 | 기총 방아쇠 방위오차 (UE WEZ 원추 2.5°보다 넓게 — 명중 판정은 UE가 함) |

발사 프로토콜: `gun_firing`은 레벨(true인 동안 연사), `missile_fire_id`는 에지
(값이 바뀔 때마다 1발, **1부터 시작** — 0은 미발사 초기값이라 UE가 무시).

## 동작 확인 포인트

- 양쪽 BT 로그에 `[EngageTarget] tgt=... 거리=... 방위오차=...`가 뜨고,
  WEZ 진입 시 `gun=ON` / `★발사`가 표시된다
- `ros2 topic echo /mumt/aircraft_states`에서 피격 기체의 `hp` 감소,
  격추 시 `destroyed: true` 확인
- 격추된 기체는 UE에서 엔진 정지 + 하드오버로 추락(Falling), 승자는 OrbitLeader로 선회 대기
