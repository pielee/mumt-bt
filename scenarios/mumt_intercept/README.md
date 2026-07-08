# mumt_intercept — 요격 기능 체크 시나리오

**적기(F16_UAV1, Enemy)**: 이륙 → 스폰 동쪽 3km 중심·반경 2km 원을 180m/s로 선회
**아군(F16_UAV2, FriendlyUAV)**: 지상이면 이륙 → 적 생존 확인 → tail-chase 추격 →
적기 뒤 ~200m 수렴 → 기총/미사일 격추 → 적 전멸 시 선회 대기
**유인기(M_F16)**: 자유 비행 (팀 적대관계 수정으로 아군의 표적에서 제외)

## 전제

- 세 기체 팀 지정: UAV1=Enemy, UAV2=Friendly UAV (레벨 인스턴스 HealthComponent)
- 제어기 개선 UE 빌드 적용 (롤 2법칙 블렌드+선회 적분기 — 이것 없으면 선회 추격 불가)
- custom_msgs 빌드 상태 (이전과 동일 — msg 변경 없음)

## 실행

```bash
source ~/dev/mumt_ros_ws/install/setup.bash     # 모든 터미널
# UE PIE (RL_2)
ros2 run mumt_ros_bridge bridge_node
python main.py --config scenarios/mumt_intercept/configs/intercept_enemy.yaml     # 적기
python main.py --config scenarios/mumt_intercept/configs/intercept_friendly.yaml  # 아군
```

## 예상 진행

1. 두 UAV 각자 이륙 (동쪽 상승, 스폰 대비 150m에서 페이즈 전환)
2. 적기: 동쪽 3km 상공 800m에서 원 선회 시작
3. 아군: 적기 추격 — 멀면 최대 265m/s로 접근, 뒤 200m에 수렴
4. BT 로그 `[InterceptTarget] 거리=... 방위오차=...°` — 방위오차 ~7° 부근에서
   `★발사`(미사일)·`gun=ON`(기총) 교차 — HP 감소 → 격추(나선 추락)
5. 아군: `적 전멸 → SUCCESS` → 선회 대기(고도 1000m)

## 폐루프 검증값 (설계 근거)

| 항목 | 값 |
|---|---|
| tail-chase 정착 | 거리오차 9.2m RMS (standoff 200m 기준) |
| 사격 기하 | LOS 평균 7.5°, 방아쇠(5°) 충족 40%, 기총 실원추(2.5°) 27% |
| 적기 원 선회 | 반경오차 113m RMS, 연속 완주 |
| 편대 거동 회귀 | 무영향 (0.0/5.4m 유지) |

## 튜닝 파라미터

InterceptTarget: `standoff_m`(후방 거리), `kp_speed`, `chase_speed_max`(선회반경 기하
때문에 과속 금지 — 기본 265), 방아쇠 4종(gun/missile range·bearing), `missile_cooldown_sec`.
OrbitPoint: 원 중심(스폰 상대 N/E m)·반경·고도·속도·점 개수.
공통: 이륙 5종 + `min_agl_m`(고도 하한 가드).
