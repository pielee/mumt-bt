"""BT ModeManager 회귀 — formation/attack 모드 발행 + 이륙/홀드 direct + 방아쇠 + XML 바인딩.

ROS 없이 실행 가능한 스텁 하네스 (std_msgs/custom_msgs/modules 목킹).
실행: python3 tools/verify_bt_modes.py   (py_bt_ros 루트에서)
"""
import sys, os, types, enum, math, inspect, asyncio, time
import xml.etree.ElementTree as ET
sys.dont_write_bytecode = True
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)

# ControlV2 sequence 영속 저장을 테스트용 임시 파일로 격리 (실제 ~/.mumt 오염 방지, 결정론적)
import tempfile as _tf
os.environ["MUMT_CONTROLV2_SEQ_STORE"] = os.path.join(_tf.mkdtemp(), "cv2seq.json")

class String:
    def __init__(self, data=""): self.data = data
m=types.ModuleType("std_msgs"); mm=types.ModuleType("std_msgs.msg"); mm.String=String
sys.modules["std_msgs"]=m; sys.modules["std_msgs.msg"]=mm
class AircraftSetpoint:
    def __init__(self):
        self.aircraft_name=""; self.heading_deg=0.0; self.altitude_m=0.0; self.roll_ff_deg=0.0
        self.throttle_norm=0.0; self.target_speed_mps=0.0; self.launch_missile=False
        self.gun_firing=False; self.missile_fire_id=0
        self.guidance_mode=""; self.leader_name=""; self.target_name=""
        self.slot_front_m=0.0; self.slot_right_m=0.0; self.slot_up_m=0.0
        self.min_speed_mps=0.0; self.max_speed_mps=0.0; self.min_alt_m=0.0
        self.protocol_version=0; self.sequence_id=0; self.timestamp=0.0
        self.capture_tolerance_m=0.0; self.maintain_tolerance_m=0.0
        self.minimum_separation_m=0.0; self.maximum_closing_speed_mps=0.0
        self.control_mode=""; self.command_sequence=0; self.command_timestamp=0.0
c=types.ModuleType("custom_msgs"); cm=types.ModuleType("custom_msgs.msg"); cm.AircraftSetpoint=AircraftSetpoint
sys.modules["custom_msgs"]=c; sys.modules["custom_msgs.msg"]=cm
class BTNodeList:
    CONTROL_NODES=['Sequence','Fallback','ReactiveSequence','ReactiveFallback','Parallel']
    ACTION_NODES=['AssignTask']; CONDITION_NODES=['AlwaysFailure']; DECORATOR_NODES=[]
class Status(enum.Enum): SUCCESS=1; FAILURE=2; RUNNING=3
class Node:
    def __init__(self,name): self.name=name; self.type=None; self.status=None; self.is_expanded=False
    def halt(self): pass
class _Ctl(Node):
    def __init__(self,name,children): super().__init__(name); self.children=children
class Sequence(_Ctl): pass
class ReactiveSequence(_Ctl): pass
class Fallback(_Ctl): pass
class ReactiveFallback(_Ctl): pass
class Parallel(_Ctl): pass
bbn=types.ModuleType("modules.base_bt_nodes")
for n,o in [("BTNodeList",BTNodeList),("Status",Status),("Node",Node),("Sequence",Sequence),
            ("ReactiveSequence",ReactiveSequence),("Fallback",Fallback),("ReactiveFallback",ReactiveFallback),
            ("Parallel",Parallel)]:
    setattr(bbn,n,o)
class ConditionWithROSTopics(Node):
    def __init__(self,name,agent,mtt): super().__init__(name); self._cache={}
class ActionWithROSTopic(Node):
    def __init__(self,name,agent,ts): super().__init__(name)
bbnr=types.ModuleType("modules.base_bt_nodes_ros")
bbnr.ConditionWithROSTopics=ConditionWithROSTopics; bbnr.ActionWithROSTopic=ActionWithROSTopic
pkg=types.ModuleType("modules"); pkg.__path__=[]
sys.modules["modules"]=pkg; sys.modules["modules.base_bt_nodes"]=bbn; sys.modules["modules.base_bt_nodes_ros"]=bbnr
class _Log:
    def info(self,*a,**k): pass
    def warn(self,*a,**k): pass
class _N:
    def get_logger(self): return _Log()
    def create_subscription(self,*a,**k): return None
class _B:
    def __init__(self): self.node=_N()
class Agent:
    def __init__(self,a): self.ros_bridge=_B(); self.agent_id=a

import importlib
FG = importlib.import_module("scenarios.mumt_manned_formation.bt_nodes")
IC = importlib.import_module("scenarios.mumt_intercept.bt_nodes")
DF = importlib.import_module("scenarios.mumt_dogfight_1v1.bt_nodes")
FF = importlib.import_module("scenarios.mumt_formation_follow.bt_nodes")
MU = importlib.import_module("scenarios.mumt.bt_nodes")
FM = importlib.import_module("scenarios.mumt_formation.bt_nodes")
CV = importlib.import_module("scenarios.mumt.controlv2_seq")

PASS=0; FAIL=0
def check(l,cond,d=""):
    global PASS,FAIL
    if cond: PASS+=1; print(f"  ✓ {l}")
    else: FAIL+=1; print(f"  ✗ {l}  {d}")
_dh=lambda a,b:((a-b+180)%360)-180

def ac(name, e_m, n_m, alt_m, yaw=90.0, spd=200.0, **kw):
    d={"aircraft_name":name, "x":e_m*100.0, "y":-n_m*100.0, "z":alt_m*100.0, "yaw":yaw, "speed_mps":spd}
    d.update(kw); return d

print("== [1] FormationGuidance: 이륙(direct) → 홀드 → 편대(mode) ==")
fg = FG.FormationGuidance("FormationGuidance", Agent("F16_UAV1"), own_name="F16_UAV1",
                          leader_name="M_F16", front_m=-80, right_m=100, up_m=0,
                          takeoff_climb_m=150, min_agl_m=150)
bb = {"own_state": ac("F16_UAV1", 0,0,100, spd=0), "leader_state": ac("M_F16",0,0,100,spd=0)}
msg = fg._build_message(Agent("F16_UAV1"), bb)
check("이륙: direct(모드 빈값)", msg.guidance_mode=="")
check("이륙: 활주로 헤딩 90", abs(_dh(msg.heading_deg,90))<1e-6)
check("이륙: 고도 스폰+800", abs(msg.altitude_m-900)<1e-6, f"{msg.altitude_m}")
check("이륙: 속도 220", abs(msg.target_speed_mps-220)<1e-6)
check("RUNNING", fg._interpret_publish(msg,None,None)==Status.RUNNING)
# own은 이륙했지만 리더가 아직 지상 → 홀드(direct 유지) [PIE 2026-07-09 수정]
bb = {"own_state": ac("F16_UAV1", 0,500,100+150, spd=200), "leader_state": ac("M_F16",0,0,100,spd=30)}
msg = fg._build_message(Agent("F16_UAV1"), bb)
check("리더 지상 → 홀드(direct 유지)", msg.guidance_mode=="")
# F1 (2026-07-10): 홀드는 직진이 아니라 리더 방향 선회 대기 — own이 리더 북쪽 500m → 남(180°)
check("홀드: 리더 방향 헤딩(180)", abs(_dh(msg.heading_deg,180))<1.0, f"{msg.heading_deg}")
check("홀드: 저속 선회 150", abs(msg.target_speed_mps-150)<1e-6, f"{msg.target_speed_mps}")
# 리더 +80m 상승 → 편대 전환
bb = {"own_state": ac("F16_UAV1", 0,500,100+150, spd=200), "leader_state": ac("M_F16",0,300,100+80,spd=150)}
msg = fg._build_message(Agent("F16_UAV1"), bb)
check("리더 이륙(+80m) → formation 모드", msg.guidance_mode=="formation")
check("leader_name=M_F16", msg.leader_name=="M_F16")
check("슬롯 (-80,100,0)", (msg.slot_front_m,msg.slot_right_m,msg.slot_up_m)==(-80.0,100.0,0.0))
check("속도한계 (120,335)", (msg.min_speed_mps,msg.max_speed_mps)==(120.0,335.0))
check("고도가드 = 스폰+150", abs(msg.min_alt_m-250)<1e-6, f"{msg.min_alt_m}")
check("편대: 유도 숫자 미기입(heading=0)", msg.heading_deg==0.0)
last = msg
check("own 결손 → 래칭", fg._build_message(Agent("F16_UAV1"), {"own_state":None,"leader_state":None}) is last)

print("== [2] InterceptTarget: 이륙(direct) → attack 모드 + 방아쇠 ==")
def mk_it():
    it = IC.InterceptTarget("InterceptTarget", Agent("F16_UAV2"), own_name="F16_UAV2")
    it._airborne=True; it._spawn=(0.0,0.0,10000.0)   # spawn z=100m(cm)
    return it
own = ac("F16_UAV2", 0,0,800, yaw=90)
it = mk_it()
enemy_far = ac("Enemy1", 5000,0,800, yaw=90, missile_count=1, hp=100)
msg = it._build_message(Agent("F16_UAV2"), {"own_state":own, "enemies":[enemy_far]})
check("교전: attack 모드", msg.guidance_mode=="attack")
check("표적지정 Enemy1", msg.target_name=="Enemy1")
check("고도가드 = 스폰100+150", abs(msg.min_alt_m-250)<1e-6, f"{msg.min_alt_m}")
check("속도상한 265(추격)", abs(msg.max_speed_mps-265)<1e-6)
check("5km·정조준 → 미사일 id=1", msg.missile_fire_id==1)
check("5km → 기총 off", msg.gun_firing is False)
it2 = mk_it()
enemy_near = ac("Enemy1", 1000,0,800, yaw=90, missile_count=1, hp=100)
msg = it2._build_message(Agent("F16_UAV2"), {"own_state":own, "enemies":[enemy_near]})
check("1km·정조준 → 기총 ON", msg.gun_firing is True)
it3 = mk_it()
msg = it3._build_message(Agent("F16_UAV2"), {"own_state":own, "enemies":[ac("Enemy1",0,1000,800,yaw=0,missile_count=1)]})
check("측방적 → 기총 off (방위오차)", msg.gun_firing is False)
check("측방적도 attack 모드 유지", msg.guidance_mode=="attack")
it4 = mk_it()
it4._build_message(Agent("F16_UAV2"), {"own_state":own, "enemies":[enemy_near]})
msg = it4._build_message(Agent("F16_UAV2"), {"own_state":own, "enemies":[]})
check("적 전멸 → 기총 off + SUCCESS", msg.gun_firing is False and it4._interpret_publish(msg,None,None)==Status.SUCCESS)
itT = IC.InterceptTarget("InterceptTarget", Agent("F16_UAV2"), own_name="F16_UAV2")
msg = itT._build_message(Agent("F16_UAV2"), {"own_state":ac("F16_UAV2",0,0,0,spd=0), "enemies":[enemy_far]})
check("지상: 이륙 direct(모드 빈값)", msg.guidance_mode=="")
check("이륙: 활주로 90", abs(_dh(msg.heading_deg,90))<1.0)

print("== [3] EngageTarget(dogfight): attack 모드 ==")
et = DF.EngageTarget("EngageTarget", Agent("F16_UAV1"), own_name="F16_UAV1")
own_e = ac("F16_UAV1", 0,0,1000, yaw=90)
msg = et._build_message(Agent("F16_UAV1"), {"own_state":own_e, "enemies":[ac("EnemyX",3000,0,1200,yaw=270,missile_count=1)]})
check("attack 모드 + 표적 EnemyX", msg.guidance_mode=="attack" and msg.target_name=="EnemyX")
check("속도상한 = engage_speed(250)", abs(msg.max_speed_mps-250)<1e-6)
check("원거리 정조준 → 미사일", msg.missile_fire_id==1)
et._build_message(Agent("F16_UAV1"), {"own_state":own_e, "enemies":[]})
check("적 전멸 → SUCCESS", et._interpret_publish(msg,None,None)==Status.SUCCESS)

print("== [4] OrbitPoint 회귀: direct 유지 ==")
op = IC.OrbitPoint("OrbitPoint", Agent("F16_UAV1"), own_name="F16_UAV1")
op._airborne=True; op._spawn=(0.0,0.0,0.0)
msg = op._build_message(Agent("F16_UAV1"), {"own_state": ac("F16_UAV1",0,0,800)})
check("선회: direct(모드 빈값)", msg.guidance_mode=="")
check("선회: heading/alt 유효", 0.0<=msg.heading_deg<360.0 and msg.altitude_m>0)

print("== [5] XML 바인딩 (formation uav1_bt.xml) ==")
def conv(v):
    if isinstance(v,str):
        if v.isdigit() or (v.startswith('-') and v[1:].isdigit()): return int(v)
        try: return float(v)
        except ValueError: pass
    return v
bt_el = ET.parse(os.path.join(_ROOT,"scenarios/mumt_manned_formation/uav1_bt.xml")).getroot().find("BehaviorTree")
for el in bt_el.iter():
    if el.tag=="BehaviorTree": continue
    if el.tag in BTNodeList.CONTROL_NODES:
        check(f"제어 {el.tag}", hasattr(FG, el.tag)); continue
    cls = getattr(FG, el.tag, None)
    attrs = {k: conv(v) for k,v in el.attrib.items()}
    try:
        inspect.signature(cls.__init__).bind(None, el.tag, Agent("F16_UAV1"), **attrs)
        cls(el.tag, Agent("F16_UAV1"), **attrs)
        check(f"{el.tag} 등록+생성 OK", True)
    except TypeError as e:
        check(f"{el.tag} 바인딩", False, str(e))

print("== [6] mumt_formation_follow: SetLeader/SetFormationSlot/EnableFormationFollow/"
      "CheckFormationCaptured/CheckFormationMaintained/LeaveFormation ==")

# (a) SetLeader/SetFormationSlot이 blackboard에 기록
sl = FF.SetLeader("SetLeader", Agent("F16_UAV1"), leader_id="M_F16", follower_id="F16_UAV1")
bb6 = {}
st = asyncio.run(sl.run(Agent("F16_UAV1"), bb6))
check("SetLeader: SUCCESS", st==Status.SUCCESS)
check("SetLeader: blackboard leader_id/follower_id",
      bb6.get("leader_id")=="M_F16" and bb6.get("follower_id")=="F16_UAV1")

sfs = FF.SetFormationSlot("SetFormationSlot", Agent("F16_UAV1"),
                          offset_front_m=-100, offset_right_m=50, offset_up_m=0,
                          capture_tolerance_m=30, maintain_tolerance_m=50,
                          minimum_separation_m=30, maximum_closing_speed_mps=0)
st = asyncio.run(sfs.run(Agent("F16_UAV1"), bb6))
slot = bb6.get("formation_slot") or {}
check("SetFormationSlot: SUCCESS", st==Status.SUCCESS)
check("SetFormationSlot: blackboard 슬롯 dict",
      slot.get("offset_front_m")==-100.0 and slot.get("offset_right_m")==50.0
      and slot.get("capture_tolerance_m")==30.0 and slot.get("maintain_tolerance_m")==50.0
      and slot.get("minimum_separation_m")==30.0, f"{slot}")

# (b) EnableFormationFollow: formation 모드 + leader_name/슬롯/허용오차 + confirm_ticks 후 SUCCESS
eff = FF.EnableFormationFollow("EnableFormationFollow", Agent("F16_UAV1"),
                               min_speed_mps=120, max_speed_mps=335, min_agl_m=150, confirm_ticks=3)
bb_eff = dict(bb6)
bb_eff["own_state"] = ac("F16_UAV1", 0, 0, 100)
bb_eff["init_alt"]  = {"F16_UAV1": 100.0}
msg = eff._build_message(Agent("F16_UAV1"), bb_eff)
check("EnableFormationFollow: formation 모드", msg.guidance_mode=="formation")
check("EnableFormationFollow: leader_name=M_F16", msg.leader_name=="M_F16")
check("EnableFormationFollow: 슬롯 (-100,50,0)",
      (msg.slot_front_m,msg.slot_right_m,msg.slot_up_m)==(-100.0,50.0,0.0))
check("EnableFormationFollow: 허용오차",
      msg.capture_tolerance_m==30.0 and msg.maintain_tolerance_m==50.0
      and msg.minimum_separation_m==30.0)
check("EnableFormationFollow: min_alt_m=spawn+min_agl", abs(msg.min_alt_m-(100.0+150.0))<1e-6, f"{msg.min_alt_m}")
statuses = []
m = msg
for _ in range(3):
    statuses.append(eff._interpret_publish(m, Agent("F16_UAV1"), bb_eff))
    m = eff._build_message(Agent("F16_UAV1"), bb_eff)
check("EnableFormationFollow: RUNNING,RUNNING,SUCCESS(confirm_ticks=3)",
      statuses==[Status.RUNNING,Status.RUNNING,Status.SUCCESS], f"{statuses}")

# (c) CheckFormationCaptured: RUNNING → captured=True → SUCCESS, timeout → FAILURE
cfc = FF.CheckFormationCaptured("CheckFormationCaptured", Agent("F16_UAV1"), timeout_s=180)
bb_cfc = {"own_state": ac("F16_UAV1", 0, 0, 100, guidance={"captured": False, "maintained": False})}
st = asyncio.run(cfc.run(Agent("F16_UAV1"), bb_cfc))
check("CheckFormationCaptured: captured=False → RUNNING", st==Status.RUNNING)
bb_cfc["own_state"] = ac("F16_UAV1", 0, 0, 100, guidance={"captured": True, "maintained": True})
st = asyncio.run(cfc.run(Agent("F16_UAV1"), bb_cfc))
check("CheckFormationCaptured: captured=True → SUCCESS", st==Status.SUCCESS)

cfc2 = FF.CheckFormationCaptured("CheckFormationCaptured", Agent("F16_UAV1"), timeout_s=0.01)
bb_cfc2 = {"own_state": ac("F16_UAV1", 0, 0, 100, guidance={"captured": False})}
asyncio.run(cfc2.run(Agent("F16_UAV1"), bb_cfc2))       # 첫 틱 → 타이머 시작
time.sleep(0.02)
st = asyncio.run(cfc2.run(Agent("F16_UAV1"), bb_cfc2))
check("CheckFormationCaptured: timeout 초과 → FAILURE", st==Status.FAILURE)

# CheckFormationMaintained 보너스 회귀(명세 (a)-(f) 외 추가 안전망)
cfm = FF.CheckFormationMaintained("CheckFormationMaintained", Agent("F16_UAV1"),
                                  hold_s=0.0, break_grace_s=0.01)
bb_cfm = {"own_state": ac("F16_UAV1", 0, 0, 100, guidance={"captured": True, "maintained": True})}
st = asyncio.run(cfm.run(Agent("F16_UAV1"), bb_cfm))
check("CheckFormationMaintained: 유지 중 → RUNNING(hold_s=0)", st==Status.RUNNING)
bb_cfm["own_state"] = ac("F16_UAV1", 0, 0, 100, guidance={"captured": False, "maintained": False})
asyncio.run(cfm.run(Agent("F16_UAV1"), bb_cfm))
time.sleep(0.02)
st = asyncio.run(cfm.run(Agent("F16_UAV1"), bb_cfm))
check("CheckFormationMaintained: captured 이탈 grace 초과 → FAILURE", st==Status.FAILURE)

# (d) LeaveFormation: direct 모드 + offset heading, publish_ticks 후 SUCCESS
lf = FF.LeaveFormation("LeaveFormation", Agent("F16_UAV1"),
                       heading_offset_deg=45, alt_offset_m=0, speed_mps=220, publish_ticks=2)
bb_lf = {"follower_id": "F16_UAV1", "own_state": ac("F16_UAV1", 0, 0, 100, yaw=350.0, spd=200)}
msg = lf._build_message(Agent("F16_UAV1"), bb_lf)
check("LeaveFormation: direct 모드", msg.guidance_mode=="direct")
check("LeaveFormation: heading=yaw+offset(wrap 0-360)",
      abs(_dh(msg.heading_deg, (350.0+45.0)%360.0))<1e-6, f"{msg.heading_deg}")
check("LeaveFormation: altitude=own+offset", abs(msg.altitude_m-100.0)<1e-6)
check("LeaveFormation: speed=220", msg.target_speed_mps==220.0)
st1 = lf._interpret_publish(msg, Agent("F16_UAV1"), bb_lf)
check("LeaveFormation: tick1 RUNNING", st1==Status.RUNNING)
msg2 = lf._build_message(Agent("F16_UAV1"), bb_lf)
st2 = lf._interpret_publish(msg2, Agent("F16_UAV1"), bb_lf)
check("LeaveFormation: SUCCESS after publish_ticks", st2==Status.SUCCESS)

# (e) XML 바인딩 (scenarios/mumt_formation_follow/uav1_bt.xml)
print("== [6e] XML 바인딩 (mumt_formation_follow uav1_bt.xml) ==")
bt_el = ET.parse(os.path.join(_ROOT,"scenarios/mumt_formation_follow/uav1_bt.xml")).getroot().find("BehaviorTree")
for el in bt_el.iter():
    if el.tag=="BehaviorTree": continue
    if el.tag in BTNodeList.CONTROL_NODES:
        check(f"제어 {el.tag}", hasattr(FF, el.tag)); continue
    cls = getattr(FF, el.tag, None)
    attrs = {k: conv(v) for k,v in el.attrib.items()}
    try:
        inspect.signature(cls.__init__).bind(None, el.tag, Agent("F16_UAV1"), **attrs)
        cls(el.tag, Agent("F16_UAV1"), **attrs)
        check(f"{el.tag} 등록+생성 OK", True)
    except TypeError as e:
        check(f"{el.tag} 바인딩", False, str(e))

# (f) 전환된 mumt.MaintainFormation: guidance_mode="formation" + 슬롯 부호 매핑
print("== [7] mumt.MaintainFormation 편대모드 전환 ==")
mf = MU.MaintainFormation("MaintainFormation", Agent("F16_UAV"),
                          aft_offset_m=-80, lateral_offset_m=40, vertical_offset_m=0,
                          own_name="F16_UAV")
bb7 = {"own_state": ac("F16_UAV", 0, 0, 900), "leader_state": ac("M_F16", 0, 200, 1000),
       "init_alt": {"F16_UAV": 900.0}}
msg = mf._build_message(Agent("F16_UAV"), bb7)
check("MaintainFormation: formation 모드", msg.guidance_mode=="formation")
check("MaintainFormation: leader_name=M_F16", msg.leader_name=="M_F16")
check("MaintainFormation: slot_front_m=+80 (aft_offset_m=-80)",
      abs(msg.slot_front_m-80.0)<1e-6, f"{msg.slot_front_m}")
check("MaintainFormation: slot_right_m=40", abs(msg.slot_right_m-40.0)<1e-6)
check("MaintainFormation: RUNNING", mf._interpret_publish(msg,None,None)==Status.RUNNING)
last = msg
check("MaintainFormation: own/leader 결손 → 래칭",
      mf._build_message(Agent("F16_UAV"), {"own_state":None,"leader_state":None}) is last)

print("\n== [8] ControlV2 sequence 계약 (Phase I-A) ==")
import json as _cvjson

# (a) ControlV2Seq 단위: heartbeat 동일 / mode·leader·slot 변경 시 +1
q = CV.ControlV2Seq("UNIT_A")
s0 = q.sequence_for("formation", "M_F16", (-200.0, 100.0, 0.0))
check("Seq: 첫 명령 sequence >= 1", s0 >= 1)
check("Seq: 동일 명령 heartbeat → 동일 seq",
      q.sequence_for("formation", "M_F16", (-200.0, 100.0, 0.0)) == s0)
s1 = q.sequence_for("formation", "M_F16", (-120.0, -150.0, 30.0))     # slot 변경
check("Seq: slot 변경 → +1", s1 == s0 + 1)
check("Seq: slot 변경 후 heartbeat → 동일",
      q.sequence_for("formation", "M_F16", (-120.0, -150.0, 30.0)) == s1)
s2 = q.sequence_for("formation", "M_F17", (-120.0, -150.0, 30.0))     # leader 변경
check("Seq: leader 변경 → +1", s2 == s1 + 1)
s3 = q.sequence_for("legacy", "", (0.0, 0.0, 0.0))                    # mode 변경 (해제)
check("Seq: Formation→Legacy → +1", s3 == s2 + 1)
s4 = q.sequence_for("formation", "M_F17", (-120.0, -150.0, 30.0))     # 재진입
check("Seq: Legacy→Formation 재진입 → +1", s4 == s3 + 1)

# (b) 재시작 지속성: 같은 store 의 새 인스턴스가 단조 이어감 (UE 가 계속 도는데 BT 만 재시작해도
#     seq 가 되돌아가 replay 로 영구 거부되지 않도록)
q2 = CV.ControlV2Seq("UNIT_A")
check("Seq: 재시작 인스턴스가 마지막 seq 이상에서 이어감",
      q2.sequence_for("formation", "M_F16", (0.0, 0.0, 0.0)) > s4)

# (c) get_seq 공유 레지스트리: 같은 기체 → 같은 인스턴스 (Enable/Leave 노드가 seq 공유)
check("Seq: get_seq 동일 기체 → 동일 인스턴스",
      CV.get_seq("SHARED_UAV") is CV.get_seq("SHARED_UAV"))

# (d) mumt_formation.FormationFlight: control_mode=formation heartbeat (사용자 시나리오)
ag = Agent("F16_UAV1")
ws = FM.MUMTWorldState(ag, "F16_UAV1")
ws._cb(String(_cvjson.dumps({"aircraft": [
    {"aircraft_name": "F16_UAV1", "x": 0.0,     "y": 0.0, "z": 50000.0, "yaw": 90.0, "speed_mps": 200.0},
    {"aircraft_name": "M_F16",    "x": 10000.0, "y": 0.0, "z": 50000.0, "yaw": 90.0, "speed_mps": 200.0},
]})))
bbF = {"_mumt_ws_F16_UAV1": ws}
ff = FM.FormationFlight("FormationFlight", ag, lateral_sign=-1.0, uav_name="F16_UAV1")
mF1 = ff._build_message(ag, bbF)
check("FormationFlight: control_mode=formation", mF1.control_mode == "formation")
check("FormationFlight: leader_name=M_F16", mF1.leader_name == "M_F16")
check("FormationFlight: command_timestamp=0 (bridge 스탬프)", mF1.command_timestamp == 0.0)
check("FormationFlight: command_sequence >= 1", mF1.command_sequence >= 1)
mF2 = ff._build_message(ag, bbF)   # 같은 leader/slot → heartbeat
check("FormationFlight: 동일 slot/leader heartbeat → 동일 sequence",
      mF2.command_sequence == mF1.command_sequence)
check("FormationFlight: guidance_mode=formation 유지(안전 baseline)", mF2.guidance_mode == "formation")

# (e) mumt_formation_follow: Enable=formation, Leave=legacy (해제 시 seq 증가)
agF = Agent("F16_UAV1")
bbE = {"leader_id": "M_F16", "follower_id": "F16_UAV1",
       "own_state": {"aircraft_name": "F16_UAV1", "x": 0.0, "y": 0.0, "z": 50000.0, "yaw": 90.0},
       "formation_slot": {"offset_front_m": -100.0, "offset_right_m": 50.0, "offset_up_m": 0.0}}
eff = FF.EnableFormationFollow("EnableFormationFollow", agF)
mE = eff._build_message(agF, bbE)
check("EnableFormationFollow: control_mode=formation", mE.control_mode == "formation")
check("EnableFormationFollow: command_sequence >= 1", mE.command_sequence >= 1)
lf = FF.LeaveFormation("LeaveFormation", agF)
mL = lf._build_message(agF, bbE)
check("LeaveFormation: control_mode=legacy", mL.control_mode == "legacy")
check("LeaveFormation: 해제 seq > enable seq (역행 없음)", mL.command_sequence > mE.command_sequence)

print(f"\n결과: PASS={PASS} FAIL={FAIL}")
sys.exit(1 if FAIL else 0)
