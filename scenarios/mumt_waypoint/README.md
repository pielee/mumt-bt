# mumt_waypoint — StickController 웨이포인트 추종 시나리오

BVRGym 제어기를 걷어내고 새로 이식한 **Controller_CY(StickController)** 조준 제어기를
구동하는 첫 시나리오. BT는 heading/altitude 대신 **목표점**을 보내고, UE가 그 점을
NEU 미터로 변환해 기수를 조준(pitch로 상하까지)한다. 속도는 오토스로틀이 유지.

## 제어 방식 (기존과 다름)

```
FollowWaypoint → AircraftSetpoint{use_waypoint=True, target_x/y/z(UE cm), target_speed_mps}
  → bridge → UDP 5010 → FUavSetpoint → ApplyAutopilotToPawn
  → UE: (North=-y/100, East=x/100, Up=z/100) 변환 → StickController::GetStick → 조종면
  → 오토스로틀(target_speed_mps)이 throttle 별도 유지
```

- **Takeoff 노드 없음**: Takeoff는 heading/altitude를 보내는데 BVRGym이 사라져 UE가 무시.
  대신 첫 웨이포인트를 전방(동쪽)+상방에 두어 StickController가 지상에서부터 이륙.
- **웨이포인트 = 스폰 상대 (dNorth, dEast, dUp) 미터** (bt_nodes.py `DEFAULT_WAYPOINTS_NEU`).
  BT가 스폰 UE 위치에 더해 UE 월드 cm로 패킹 전송; UE가 역변환(왕복 항등).

## 전제 (빌드)

- UE: 제어기 교체 빌드 적용 (BVRGym 조종면 제거 + StickController 배선).
- ROS: `colcon build --packages-select custom_msgs mumt_ros_bridge` 후 **모든 터미널 re-source**
  (use_waypoint/target_x/y/z 필드 추가된 msg 반영 필수).
- 에디터: 대상 UAV 폰이 JSBSim 기체여야 함 (LocalEulerAngles 기반 자세 변환).

## 실행

```bash
cd ~/dev/mumt_ros_ws && colcon build --packages-select custom_msgs mumt_ros_bridge
source install/setup.bash
# UE 에디터 PIE 실행 (RL_2 등, F16_UAV1 배치된 레벨)
ros2 run mumt_ros_bridge bridge_node
python main.py --config scenarios/mumt_waypoint/configs/waypoint_uav1.yaml   # py_bt_ros에서
```

## 튜닝 파라미터 (XML)

| 파라미터 | 기본 | 설명 |
|---|---|---|
| `cruise_speed_mps` | 220 | 오토스로틀 목표 속도 |
| `accept_radius_m` | 400 | 웨이포인트 도달 반경(3D). 조준 제어기는 점을 정확히 통과하지 않으므로 넉넉히 |
| `loop` | True | 마지막 웨이포인트 후 처음으로 순환 |

경로 자체(웨이포인트 좌표)는 `bt_nodes.py`의 `DEFAULT_WAYPOINTS_NEU`에서 편집.

## PIE 검증 포인트

- UE 로그 `[Stick] F16_UAV1 wp=1 VP=(...) | Psi=... Ail=.. Elv=.. Rud=.. Thr=..`
- 이륙 후 동쪽으로 상승 → 박스 경로 순환. BT 로그 `[FollowWaypoint] wp k/4 거리=..`
- **부호 검증(중요)**: 좌/우 반대로 돌거나 기수가 거꾸로 들리면 UDPControlReceiver 액터
  Details의 `StickAileronScale`/`StickElevatorScale`/`StickRudderScale`을 −1로 뒤집어 확인.
  특히 North=-Y(좌우)와 rudder 부호가 1순위 의심 지점.
