"""ControlV2 command_sequence 관리 (Phase I-A).

UE ControlV2 는 command_sequence <= 마지막 적용 seq 를 replay 로 폐기한다. 그래서 sequence 는:
  - 같은 운용 명령(control_mode/leader/slot)의 heartbeat  → 값을 유지 (idempotent, re-prime 없음)
  - leader / slot / mode 가 바뀌면                         → +1
  - 프로세스(BT/bridge) 재시작 후에도 단조 증가             ← UE 가 계속 도는 채 BT 만 재시작하면
    seq 가 되돌아가 이후 모든 명령이 영구 replay 로 거부된다. 이를 막으려 기체별 카운터를
    파일에 영속화해 재시작 시드로 쓴다. wall-clock 을 정수 sequence 로 쓰지 않는다(단조성/충돌 위험).

영속 저장이 실패해도(권한/디스크) 세션 내 단조성은 메모리 카운터로 보장한다.
저장 위치는 MUMT_CONTROLV2_SEQ_STORE 환경변수로 재정의 가능(테스트용).
"""
import json
import os
import threading

_DEFAULT_STORE = os.path.join(os.path.expanduser("~"), ".mumt", "controlv2_seq.json")
_LOCK = threading.Lock()


def _store_path():
    return os.environ.get("MUMT_CONTROLV2_SEQ_STORE", _DEFAULT_STORE)


def _load_store():
    try:
        with open(_store_path(), "r") as f:
            data = json.load(f)
        return {str(k): int(v) for k, v in data.items()} if isinstance(data, dict) else {}
    except Exception:
        return {}


def _persist(name, seq):
    try:
        with _LOCK:
            store = _load_store()
            store[name] = max(int(seq), int(store.get(name, 0)))
            path = _store_path()
            os.makedirs(os.path.dirname(path), exist_ok=True)
            tmp = path + ".tmp"
            with open(tmp, "w") as f:
                json.dump(store, f)
            os.replace(tmp, path)
    except Exception:
        pass  # best-effort; 세션 내 단조성은 메모리 카운터로 유지


class ControlV2Seq:
    """한 기체의 command_sequence 를 관리한다."""

    def __init__(self, aircraft_name):
        self._name = str(aircraft_name)
        # 재시작 시드: 파일에 남은 마지막 값에서 이어감 (없으면 0 → 첫 명령이 1)
        self._seq = int(_load_store().get(self._name, 0))
        self._key = None  # 마지막 (control_mode, leader, slot) 튜플

    @property
    def value(self):
        return self._seq

    def sequence_for(self, control_mode, leader, slot):
        """이 (control_mode, leader, slot) 명령에 쓸 sequence. 바뀔 때만 +1 후 영속화."""
        key = (str(control_mode), str(leader),
               tuple(round(float(s), 3) for s in slot))
        if key != self._key:
            self._seq += 1
            self._key = key
            _persist(self._name, self._seq)
        return self._seq


# 프로세스 공유 레지스트리: 같은 기체를 다루는 여러 노드(Enable/Leave/Formation)가 하나의
# 단조 카운터를 공유해야 enable→leave→re-enable 전이에서 sequence 가 역행하지 않는다.
_REGISTRY = {}


def get_seq(aircraft_name):
    """이 프로세스에서 그 기체의 ControlV2Seq(공유 인스턴스)를 돌려준다."""
    name = str(aircraft_name)
    inst = _REGISTRY.get(name)
    if inst is None:
        inst = ControlV2Seq(name)
        _REGISTRY[name] = inst
    return inst
