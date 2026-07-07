# mumt_manned_formation — 유인기 편대비행 (StickController)

사람이 조이스틱으로 모는 **유인기(M_F16)**를 **UAV(F16_UAV1)**가 자동으로 따라붙는 시나리오.
BVRGym 제거 후 새 제어 체계(조준 제어기 + use_waypoint) 기반의 첫 편대 시나리오.

## 슬롯: 유인기 **앞 100m · 우측 40m · 동일 고도** (전방 편대)

조종사 시야 오른쪽 앞에 UAV가 자리잡는다. UAV는 리더를 못 보지만 5006 상태로 안다.

## 동작 순서

1. **자체 이륙**: PIE 시작 직후 UAV가 스폰 전방(동쪽 3km)+상방 800m 지점을 조준해
   지상에서부터 기수 들고 이륙·상승 (유인기가 지상에 있어도 안전)
2. **편대 합류**: 스폰 대비 150m 오르면 자동 전환 — 유인기 슬롯 추종 시작
   (BT 로그: `이륙 완료(+150m) → 편대 합류 전환`)
3. **편대 유지**: 유인기가 어디로 날든 슬롯을 따라옴 — 선회하면 슬롯도 리더 헤딩과
   함께 회전

## 제어 설계 (조준 제어기용 편대 법칙)

- **방향**: 슬롯 자체가 아니라 **캐럿**(슬롯에서 리더 헤딩 방향 800m 앞 점)을 조준
  → 정위치에선 기수가 리더와 정렬, 횡편차는 자연 수렴 (슬롯 조준 시의 기하 퇴화 회피)
- **전후 위치**: 속도로 제어 — `목표속도 = 리더속도 + 0.05×(슬롯까지 전후거리 m)`,
  closure ±60m/s 클램프, [70, 335] 한계 (상한=f16 실측 최고속도)
- **고도**: 슬롯 고도(=리더 고도)를 target_z로 — pitch 조준이 상하를 잡음

## ⚠️ 전방 슬롯의 물리적 한계

UAV 속도 상한은 **335 m/s** — JSBSim f16 실측 수평 지속최대(0.3~1.2km 고도에서
334.6~336.9 m/s, M0.98 천음속 한계)와 동일하게 설정. 즉 유인기가 낼 수 있는 모든
속도(풀스로틀 포함)까지 추종 가능하다. 단 유인기가 정확히 최고속도로 날면 접근
마진이 0이라 한 번 뒤처지면 못 따라잡는다 — 재합류가 필요하면 ~300 이하로.
급기동 시 UAV가 뒤처지는 건 순수추종의 구조적 지연(개선안은 폴리싱 항목).

## 실행

```bash
# (msg 재빌드가 아직이면) cd ~/dev/mumt_ros_ws && colcon build --packages-select custom_msgs mumt_ros_bridge
source ~/dev/mumt_ros_ws/install/setup.bash
# UE PIE 실행 (RL_2 — M_F16, F16_UAV1 배치 레벨)
ros2 run mumt_ros_bridge bridge_node
ros2 launch mumt_ros_bridge manned_joystick.launch.py     # 유인기 조종 (별도 터미널)
python main.py --config scenarios/mumt_manned_formation/configs/formation_uav1.yaml
```

순서 자유: UAV는 유인기가 지상에 있어도 먼저 이륙해 상공에서 슬롯으로 내려온다.
(단 GatherState가 own+leader 둘 다 보여야 동작 — M_F16 pawn이 레벨에 있어야 함)

## 튜닝 파라미터 (XML)

| 파라미터 | 기본 | 설명 |
|---|---|---|
| `front_m` / `right_m` / `up_m` | 100 / 40 / 0 | 슬롯 오프셋 (앞+/우+/상+) — 음수로 뒤/좌/하 |
| `carrot_m` | 800 | 조준점 전방 거리. 크면 완만·작으면 민첩(진동 위험) |
| `kp_speed` | 0.05 | 전후거리→속도 게인 |
| `closure_min/max_mps` | −60/+60 | 리더 대비 접근속도 한계 |
| `takeoff_forward_m/up_m` | 3000/800 | 이륙 목표점 (스폰 상대) |
| `takeoff_climb_m` | 150 | 편대 전환 상승량 (스폰 대비) |

## PIE 검증 포인트

- UAV 단독 이륙 → 동쪽 상승 → 유인기 이륙 후 우측 전방으로 미끄러져 들어와 고정
- 유인기 선회 시 슬롯이 함께 회전 — UAV가 바깥쪽에서 따라 도는지
- BT 로그 `슬롯거리=..m along=+..m Vtgt=..` 가 정위치에서 슬롯거리 ~0, Vtgt ≈ 리더속도
