# RAON-RT 병합 트랙 — 통합 기록 (merge.md)

> **이 문서의 목적**: `~/RAON-RT-Revision` (KIST/RAIMLAB RAON-RT 프레임워크)을 이 저장소의
> perception 파이프라인과 병합해 **"파이프라인이 인식한 물체 위로 Indy7 TCP를 접근"**시키는
> 트랙의 정본. 결정·진행·게이트·버그·재개 방법을 이 한 파일로 파악한다.
>
> **작성**: 2026-07-26 (Phase 0~2a + pick_logic v2까지 완료된 시점) · 세션 기록은
> `~/.claude/projects/-home-ubuntu/memory/raon-vs-merge-plan.md`가 병행 보존.
>
> **참조 정본**:
> - RAON-RT 코드 구조/버그 전수: `~/RAON-RT_guide_for_CLAUDE.md` (348KB, §1~§14 — §13 통합지침·§14.2 차단요소)
> - 로봇제어 코드: GitHub `seriouslysilly21612/RAON-RT-Revision` **브랜치 `kv260-merge`** (로컬 `~/RAON-RT-Revision`)
> - 이 저장소(ros2_ws)는 perception 쪽 절반만 담당 — 두 저장소는 ROS2 토픽으로만 만난다.

---

## 0. 30초 요약 (2026-07-27 현재)

- **목표**: perception 파이프라인은 그대로 살린 채, `/pick_target_base`(base_link 좌표)를
  RAON-RT 제어 앱이 구독 → 오퍼레이터가 터미널 메뉴로 물체 선택 → `z+margin`, top-down
  자세로 **look-then-move 접근** (CST 토크 + RBDL, 1 kHz).
- **완료**: Phase 0(기반: RT-POSIX·EMasterApp aarch64) → Phase 1(버스 검증: 7슬레이브 PDO
  비트단위 일치) → Phase 2a(ViSP 제거 앱 빌드 + §14.2 픽스) → **pick_logic v2**(선택·안정성·
  person 가드, 합성테스트 6/6) → **Gate 2b**(ROS2 브릿지 + 대화형 메뉴 'p'/숫자/'v',
  합성테스트 5/5 ×2연속) → **Phase 3**(서보-오프 통합 런: 300 s 무단절 OP·정상상태 유실 0·
  파이프라인 동시·in-app 메뉴/param/'v'·PDO=SDO 조인트 대조·person_guard 실환경 실증) →
  **Phase 4**(서보온 게이트1 grav-comp + 게이트2 'a' 실모션) → **refine 160→10.9 mm** →
  **Phase 5 전체**(손-눈 캘리브 rMc residual 1.8 cm → TF/박스 정본화 → **'v' 접근 데모
  전 클래스 성공**: 접근→'b' homing→다음 물체 사이클. 1차 시도 테이블 충돌(E13)은
  soft-R IK+Δq 게이트+T 스케일링으로 해소) → **2026-07-27 주간**: F/T 페이로드 URDF
  반영(공구축=−X 함정 E16), sticky-float(속도게이트 완화판 오퍼레이터 OK), 서보-오프
  goal 가드(E15), radial 게이트, refine oriWeight=0 회귀픽스(E18) → **ready-seed IK**
  (`19b15ba`: q_ready 고정 시드로 접근 해가 시작 자세와 무관하게 결정적 + 2π 폴딩 +
  자동 스테이징 — **실기 검증 대기**).
- **20~22시 실기 — E21/E22/E23 연쇄 복구 완결**: v3 정상(괴랄 궤적 소멸) → 도달점이 물체를
  ~11 cm 벗어남 = **E21 로봇 베이스 물리 이동**(프로브 fit yaw +5.14°/t(+4.5,+10.4) cm →
  launch TF 재베이스) → 게이트 상수도 옛 프레임 유물(E22, 박스·r 재베이스) → 원거리 soft-R
  정체(E23, w 사다리+60° tilt 캡) → **전 5클래스 접근 성공**(refine 4.9~10.6 mm; 원거리
  3종은 tool 41~48° tilt로 도달 — 수직-hover 한계 표시).
- **22~23시 D10 3+1 격리 배치**: cmdline `isolcpus=3 nohz_full=3 rcu_nocbs=3 irqaffinity=0-2`
  적용·재부팅, cfg `[TASKn] CPU=` 키로 1 kHz 페어(RT97/RT95)를 CPU3 단독 점유, 나머지 15개
  비RT 스레드 0-2 — /proc 토폴로지 확인 완료, EtherCAT OP·2000 f/s·Lost 2(재베이스). 순수
  지터 A/B는 미완(앱과 gate0가 같은 격리코어를 다퉈 측정 오염).
- **2026-07-28 새벽 — E24 손목 0점 사고 해결**: `h` 순간 팔이 뒤로 넘어가고 전 물체가
  119~139° tilt로 거부 → 원인은 AXIS4/5의 `HOMING_METHOD=1`이 **cfg의 POS_BEFORE_EXIT를
  시작 각도로 디코드**(팔 위치와 무관)하는 것. J4 81.7° 오차 → 중력보상이 손목을 능동적으로
  넘김 → 무너진 자세로 HOME 기록되며 앵커까지 오염. **AXIS4/5 → HOMING_METHOD=3**으로
  파일 유래 0점 제거, 엔코더 카운트 고고학으로 잔차 ≤2.8° 검증, 실기 전 5클래스 재현(§7 E24).
  잔여: `w=0` 폴백 브랜치 사고(E25), refine 진동(E26).
- **다음**: 파지 준비 — top-down 고정 R(기울기 정식 해결) + TCP 재정의(끝면 기준, 공구 스택
  ~5 cm 실측 반영) + <5 mm 정밀도, 물체 6D pose는 나중 — §9. task-space servo는 사고
  부검(E17) 전제조건 하 백로그.
- **사건 1건 해소(2026-07-26)**: 첫 Phase 3 시도에서 앱 `bad_alloc`(memlock 한도 유한 세션 +
  `mlockall(MCL_FUTURE)`+DDS arena) → teardown 중 커널 하드 락업 → 보드 재부팅. §7 E6~E8.
- **로봇**: Indy7이 eth0에 직결, 제어전원 인가 시 7슬레이브 PREOP 상시 응답. 모션은 아직 0.

## 1. 목표와 최종 아키텍처

### 1.1 목표의 변천 (오해 방지용 히스토리)

| 시점 | 목표 후보 | 결말 |
|---|---|---|
| 계획 초기 | RAON-RT의 ViSP AprilTag visual servoing 재현 | **폐기** — `VisualServo`가 librealsense로 카메라를 직접 열어 perception 파이프라인과 **배타 점유 충돌**. 병합 목적(파이프라인 검출→로봇)과 모순 |
| 최종 확정 | **인식 물체 위 TCP 접근 (look-then-move)** | 파이프라인 유지 + `/pick_target_base` 구독. AprilTag/ViSP는 **손-눈 캘리브 1회용**으로만 (그마저 이미지 처리는 데스크톱 오프라인) |

이동 중 연속 추적(진짜 closed-loop)은 백로그 — 컨트롤러에 `SetTargetPose_Jacobian` 경로가
이미 있어 15 Hz 갱신만 물리면 업그레이드된다.

### 1.2 아키텍처 (병합 접점 = ROS2 구독 하나)

```
[ros2_ws perception — 무수정 유지]
  camera → detector(DPU) → pick_logic v2 → t3d(camera frame) → base node(TF) 
                                │                                   │
                                │ desired_class (LIVE param)        ▼
                                │ ← ros2 param set ────  /pick_target_base (PickTarget3D, base_link, m)
                                                                    │ 구독
[RAON-RT App/Indy7 (kv260-merge)] — CROS2PickBridge                 ▼
  'p' 키: /detections 집계 메뉴 → 숫자로 물체 선택 → param set ──┘
  'v' 키: N프레임 통계 게이트(std·워크스페이스 박스·radial) → goal=(x, y, z+margin)
          → SolveReadyIK(q_ready 고정 시드, 2026-07-27~) → 직행 or q_ready 경유 스테이징
          → quintic 궤적 → CTC → CST 토크 → EtherCAT → Indy7
          (자세 = q_ready 가족의 soft-R; top-down 고정 R·물체 6D는 백로그)
```

### 1.3 좌표 변환의 소재 (자주 헷갈리는 지점)

**변환은 전부 ros2_ws 파이프라인에서 한다** (`pick_target_base_node` + 정적 TF).
RAON-RT는 이미 base_link로 변환된 좌표를 소비만 한다 — RAON-RT의 rMc/VS 변환 기계는 안 쓴다.
실제 남은 일은 코드 이동이 아니라:

1. ~~placeholder TF 정본화~~ **완료(2026-07-27)**: rMc(base→color 광학) t=(0.7612, -0.0997,
   0.9262), 광축이 base -Z에서 5.6° 기울어진 top-down. launch의 정적 TF는
   `base_link→camera_link` = rMc·inv(camera_link→color_optical, realsense 라이브 TF 실측)
   — RPY가 pitch 90° gimbal 특이점 근처라 **의도적으로 quaternion 표기**
   (재합성 오차 0.0004 mm 확인). 원데이터·solver: RAON `App/CalibUtils/kv260/`.
   **E21 보정(2026-07-27 저녁)**: 사고로 로봇 베이스가 밀림(yaw +5.14°, t (+4.5, +10.4) cm
   — 손-프로브 2점 fit, §7 E21) → 현행 정본 = **t (0.811666, 0.087684, 0.924974),
   q (0.72462829, 0.00008362, -0.68706199, 0.05347582)** (`tools/base_shift_fit.py`로 합성).
2. rMc의 "r"은 RAON-RT FK의 로봇 베이스 → 파이프라인 base_link = RBDL 모델 베이스로 자동 정합
   (검증: 캘리브 태그 z-거리와 FK 플랜지 높이 일치, apple 실좌표 기하 일치).
3. reachability 게이트는 별도 노드 파라미터가 아니라 **브릿지 워크스페이스 박스가 전담**
   (실측치 §5 Phase 5 행). **z 의미 주의**: `/pick_target_base`의 z는 물체 **윗표면**(depth가
   닿는 면)이지 부피 중심이 아니다 — 접근 z+0.15는 표면 기준이라 그대로 안전.
   **테이블면은 base z≈+0.10** (브레드보드가 로봇 base 0면보다 높음, depth 실측).

## 2. 확정 결정 로그

| # | 결정 | 근거/비고 |
|---|---|---|
| D1 | 제어 = **CST 토크 + RBDL** `CControllerFullDynamicsRT` 재사용 | 구 "CSP 직결" 계획 대체. Indy7=NRMK CORE라 토크변환 정상 경로(비-NRMK 항등변환 버그 비해당) |
| D2 | 구조 = **독립 make + ROS2 브릿지** | RAON-RT는 colcon 밖. ros2_ws엔 아무것도 안 들어감(브릿지가 my_interfaces를 링크) |
| D3 | 코드 정본 = **Revision** | 원본(lab git)은 Revision의 순수 부분집합(원본에만 있는 건 .git뿐). 가이드 162항목 검증도 Revision 기준 |
| D4 | rtposix = **RT-POSIX 원본 빌드** (shim 불필요) | `git.raimlab.com/RAIMLAB/RT-POSIX` = RT-AIDE 정품 소스(같은 저자). aarch64 이식 OK |
| D5 | sim-first **철회** → 실기 중심 | Indy7이 eth0 직결로 판명 → "서보-오프 통합 런"이 sim이 하려던 검증을 모션 0으로 대체+와이어까지 검증 |
| D6 | 물체 선택 = **대화형 터미널 메뉴** | 오퍼레이터가 로봇측 터미널에서 보기 중 선택 → `desired_class` LIVE 파라미터로 전달 |
| D7 | person 안전 = **근접(대형 bbox)만 차단+래치** | 배경 통행인엔 무반응, 워크스페이스 접근 시 타깃 무효화 |
| D8 | 슬레이브 DC = **저자 구성 그대로 OFF(SM-sync)로 브링업** | INDY7.cfg에 `DC_SUPPORT` 키 자체가 없음(기본 0) = 저자의 검증된 구동 상태. DC 실험은 cfg 주석 해제로(코드 준비됨) |
| D9 | 사이클 = **1 ms 유지** | `m_dt=0.001` 하드코딩과 결합돼 있고, 동급 GEM 실측 "1 ms 안정" 근거. 2 ms 완충안은 폐기 |
| D10 | 3+1 코어 격리 = ~~보류~~ → **착수(2026-07-27 밤, 오퍼레이터 지시 — 그리퍼 대기 중 인프라 작업)** | RT-POSIX가 전 태스크를 CPU0에 기본 pin(`posix_rt.c:257`) → isolcpus만으론 무효, 앱 pin 코드와 한 세트. **실증 확인(2026-07-27)**: 접근 데모 전 구간 비격리(cmdline `skew_tick=1`만), Lost 0·지터 이상 무. **구현(`a6e2097`)**: cfg `[TASKn] CPU=` 키(파서+`InitRTTasks` pin, 미지정=-1=기존 CPU0) — TASK0/1(1 kHz 페어)만 CPU3, 키보드/터미널/로거는 CPU0 잔류. gate0에 cpu 인자. **비격리 베이스라인(파이프라인 부하, 20 s)**: CPU3 avg 1.0/max 33.9 µs, CPU0 avg 1.2/**max 104 µs**. cmdline 전환은 `/etc/default/flash-kernel` → `"skew_tick=1 isolcpus=3 nohz_full=3 rcu_nocbs=3 irqaffinity=0-2"`(+`flash-kernel`+재부팅; 커널 NO_HZ_FULL=y·RCU_NOCB=y 확인, 비전 ~1.8코어라 0-2 3코어 충분 — 7/9의 2+2 실패와 다름). GEM IRQ의 CPU3 이전 여부는 격리 후 실측으로. **재부팅+격리 A/B 대기** |
| D11 | 캘리브 이미지 = **파이프라인 토픽에서 캡처** | librealsense 직접 열기 금지(충돌). intrinsics도 camera_info에서 → `save_camera_params` 불필요 |
| D12 | 캘리브 체인 = **ViSP 완전 배제, 태그 pose는 OpenCV로** (2026-07-26) | `visp-compute-apriltag-poses`는 x86-64 바이너리+소스 미포함+원저자 홈 rpath → 어디서도 실행 불가. `cv2.aruco`(APRILTAG_36h11)+`solvePnP`로 동일 YAML 산출. intrinsic은 공장값(camera_info) 1순위 — 파이프라인 3D와 같은 카메라 모델이어야 rMc 정합(불일치가 rMc에 흡수되는 계통오차 방지). 원저자 camera.xml은 640×480이라 어차피 재사용 불가(우린 848×480). 잔차 불량 시에만 `cv2.calibrateCamera` 재캘리브로 escalation |
| D13 | 접근 IK 시딩 = **init 계산 q_ready 고정 시드 + 자동 분기** (2026-07-27, 오퍼레이터 선택) | 근본 원인: RBDL IK는 local 반복법이라 **시드가 유일한 브랜치 선택 장치**인데 시드=현재 자세였음 → 성공이 "오퍼레이터가 팔을 어디 놔뒀나"에 종속(시도마다 자세도 상이). 시드는 계산상 출발점일 뿐 물리 자세와 분리 가능 → init에서 워크스페이스 박스 중심을 풀어 q_ready 산출·코너 8점 검증, 매 접근을 q_ready(+그 자세의 soft-R)에서 해석 = **같은 목표, 같은 해**. 이동 정책(사용자 선택지 4개 중): **자동 분기** — Δq≤2.0 직행, 초과 시 q_ready 경유 자동 스테이징. 대안 검토: 해석적 IK(브랜치 전수 열거)가 근본 치료지만 기계변환 URDF의 프레임 뒤틀림(E16)이라 수식 유도 고위험 → 백로그 |

## 3. 저장소·브랜치 체계

| 트리 | 정체 | 규칙 |
|---|---|---|
| `~/RAON-RT-Revision` | **작업 저장소** — GitHub `seriouslysilly21612/RAON-RT-Revision` clone (Revision 개발 실히스토리) | 작업은 **`kv260-merge` 브랜치만**, `main` 무접촉, push는 이 GitHub로만 |
| `~/RAON-RT` | lab 원본 clone (`git.raimlab.com`, dev) | **읽기 전용** — push 절대 금지. 히스토리 참조용 |
| `~/RAON-RT-Revision-main` | zip 스냅샷 | 무수정 참조본 |
| `~/RT-POSIX` | RT-AIDE rtposix 소스 clone | 로컬 패치 1건 적용(빌드용) — 패치 파일은 kv260-merge에 보존 |
| `~/ros2_ws` | 이 저장소 | perception 쪽 변경(pick_logic v2 등)만. RAON-RT와 토픽으로만 결합 |

`kv260-merge` 커밋 트레일 (main 이후):
`59fc801` 브랜치 셋업 → `2cdba3a` x86 빌드잔재 untrack → `9c3a28a` Gate0 테스트+rtposix 패치 →
`973a57e` gitignore → `f59f49b` gate0 warm-up 스킵 → `b3b8209` **Phase 2a** (ViSP 제거+픽스+빌드) →
`f7c27fc` **Gate 2b** (ROS2 pick bridge + 대화형 흐름 + 합성테스트) → `1ed4881` **Phase 3**
(스모크 스크립트 + E8 키보드 가드 + 사건 방어책) → `2c8fae5` **Phase 4 준비** (런타임 enable
픽스 + 'h'/'j' 서보 arm/disarm 인터록) → `1334658` run.sh 런처 → `d94c1c1` B6+게이트2 → `0668ed2` **refine 10.9 mm** →
`8093e8a` **Phase 5 캘리브** (grabber/cpo 툴 + 16자세 데이터셋 + rPc + 박스 실측치) →
`61a4a86` **E13 안전 IK + HOME + 브릿지 세션화** (접근 데모 완주 커밋).
ros2_ws(main, 로컬): `a9644dc` **TF 정본화 + person_guard off** → `1470d98` docs.

## 4. 시스템 배치 현황 (보드, 전부 검증됨)

| 항목 | 위치 | 상태 |
|---|---|---|
| IgH EtherLab (stable-1.6) | `/opt/etherlab` + `ethercat` CLI | 마스터 기동 검증. `ethercat.conf`=eth0 MAC+generic |
| librtposix (aarch64, 패치) | `/opt/rt_posix/{include,lib}` | `--coverage` 제거+`-fPIC`+stack-min 클램프 반영 |
| libEMasterApp (PDO 픽스 포함) | `/opt/emaster_app/{include,lib}` | `sudo make install`로 갱신됨(20:03) — **런타임은 이걸 로드** |
| RBDL 3.1.3 (rbdl-orb+urdfreader) | `/usr/local/lib` | ldconfig 등록 |
| ld 검색 경로 | `/etc/ld.so.conf.d/raon-rt.conf` | 3경로 등록 |
| RT 권한 | `/etc/security/limits.d/99-realtime.conf` | ubuntu rtprio 98 + memlock unlimited (재로그인 후 발효) |
| EtherCAT 디바이스 udev | `/etc/udev/rules.d/99-ethercat.rules` | `/dev/EtherCAT0` 0664/ubuntu — 일반유저 CLI 가능 |

**앱 빌드 명령** (App/Indy7에서):
```bash
make clean   # 필수 1회 — 저자의 x86 obj/*.d 잔재가 남아 있으면 빌드 즉사
make         # 정식 (RBDL=/usr/local, EMasterApp=/opt)
# 개발 중 /opt 미갱신 상태로 돌릴 땐:
make RBDL_DIR=~/rbdl-stage/usr/local ECAT_INCLUDE=../../include/EMasterApp ECAT_LIB=../../lib/EMasterApp
```
⚠️ 링크는 어디서 하든 **런타임 로더는 `/opt/emaster_app` 것을 집는다** — EMasterApp 소스를
고치면 실행 전 `cd ~/RAON-RT-Revision && sudo make install` 필수.

## 5. Phase/Gate 진행 현황

| Phase | Gate | 결과 |
|---|---|---|
| **0 기반** | Gate 0: RT(SCHED_FIFO) 1 ms×5000 지터 | ✅ **avg 1.3 µs / max +35 µs** (984 µs 아웃라이어는 stale-first-deadline 시작 아티팩트로 규명 — 테스트에 warm-up 10사이클 스킵 반영) |
| **1 버스 검증** | Gate 1: 스캔↔cfg 대조 (모션 0) | ✅ 7/7 슬레이브(6×Drive `0x089a/0x30000000` + EOAT `0x10000007` 실존→cfg 무수정), **PDO 맵 10엔트리 비트단위 일치**, DC 64bit, Lost frames 0 |
| **2a 앱 포팅** | Gate 2a: aarch64 빌드 | ✅ `bin/Indy7Ctrl.out` — ViSP-free, 의존성 전해소 |
| **2a' 파이프라인** | pick_logic v2 합성테스트 | ✅ **6/6 PASS ×2연속** + 프로세스 누수 0 (`tools/vision/test_pick_logic_v2.py`) |
| **2b 브릿지** | Gate 2b: 브릿지 합성테스트 (카메라·로봇 불필요) | ✅ **5/5 PASS ×2연속** + 누수 0 (`tools/test_gate2b_bridge.py`) — 메뉴/param 왕복(`ros2 param get`=apple 실증)/lock/std게이트(≈1.5 mm<8 mm)/goal z=0.27 m(0.12+마진 0.15) |
| **3 서보-오프 런** | 수 분 무단절 + 조인트 판독 + 파이프라인 동시 + in-app 'p'/'v' | ✅ (2026-07-26, `tools/test_phase3_smoke.py`) app-only **4/4** → 풀 런 **6/6** → **300 s 홀드**: 정상상태 Lost frames 0(활성화 천이 2개는 베이스라인 제외), 에러 0, 7슬레이브 OP 유지. **라이브 메뉴에 5클래스 전부** 표시→apple 선택→실 pick_logic param 반영→'v'는 person_guard(area 0.08≥0.06)로 정당 거부 = **D7 첫 실환경 실증**. 조인트 판독 = DeInit **PDO값과 SDO 대조 전축 수 카운트 일치**(servo-off 브레이크로 손-이동 검사는 불가·불필요). SIGINT DeInit 경로(위치저장·마스터 해제·브릿지 다운·PREOP 복귀) 반복 검증. 코어 격리 A/B는 **비격리로 이미 유실 0이라 보류** — Phase 4 실토크에서 지터 보이면 재개(D10 갱신) |
| **4 서보온 — 게이트1 grav-comp** | 오퍼레이터 게이트 서보온('r'→'g'→'h') | ✅ (2026-07-26) **6축 동시 0x0237(OPERATION ENABLED), grav-comp 34 s 유지, 손밀기 컴플라이언스 확인, 'j' disarm 클린**. 사전에 E-stop 실효성 확인(버스 전원 차단→해제 후 fault 0 복구, E10). 실행은 `App/Indy7/run.sh` |
| **정확도 개선 (refine)** | CTC droop 반복 보정 | ✅ (2026-07-27, `0668ed2`) **160 mm → 10.9 mm** (4 pass 단조수렴 60→25→19→17→10.9). 감쇠 0.65 편향 + 속도게이트(|qd|<0.02, 0.3 s) 측정 + tol 12 mm(=stiction 바닥). 'a'와 비전 접근 모두 자동 적용. <5 mm는 게인/closed-loop 백로그 |
| **4 서보온 — 게이트2 'a' 위치추종** | 하드코딩 목표 실모션 | ✅ (2026-07-26) 사전에 **손-유도 프로브로 카메라 위치 실측**: base (+0.70, -0.13, 0.98) → **테이블/카메라 = +X 확정**(워크스페이스 박스 입력). 'a' 목표는 반대편(-X)이라 GO → 3 s quintic 정상 실행·유지·'g' 소프트중단 검증. **정밀도 발견: 정상상태 오차 ~16 cm**(도달 (-0.330,-0.194,0.866) vs 목표 (-0.181,-0.181,0.931)) — 무적분 CTC + 하모닉 마찰의 전형. **Phase 5 전 개선 필수**(게인/적분/도달 후 재타겟 반복 — 궤적 CSV는 B6 픽스로 분석 가능해짐) |
| **5-1 캘리브 캡처** | 12~15자세, 회전 다양성 | ✅ (2026-07-27) **16쌍 lockstep**(grabber Enter→앱 's', 태그 검출시에만 저장). 재검출 16/16, 쌍별 회전각 중앙값 40.8°·92%≥15°, FOV x225~736/y45~426, 이동 스프레드 33×56×31 cm. 태그 = 36h11 id0, 검은 사각형 **65 mm**(흰테두리 포함 80 — 8:10 규격비 일치로 측정 교차확인) |
| **5-2 AX=ZB solve** | rMc + residual | ✅ (2026-07-27, `8093e8a`) `calib_cpo.py`(IPPE_SQUARE, reproj 0.12~0.55 px, 모호성비 ≥2.8) → 원저자 `eye_to_hand_calib.py` 무수정 실행. **rMc t=(0.7612,-0.0997,0.9262), residual 1.81 cm/1.66°**. 손-프로브(0.698,-0.132,0.981)와 9 cm 정합(프로브는 ±10 cm짜리 참고치). eMo(플랜지→태그)도 동시 해결 — 태그 장착 실측 불필요 |
| **5-3 TF+박스 정본화** | 파이프라인 반영 + 실좌표 검증 | ✅ (2026-07-27, ros2_ws `a9644dc`) launch TF quaternion 교체(§1.3), 브릿지 박스 실측치 x[0.30,0.85] y[-0.50,0.45] z[0.10,0.50](카메라 아래 >0.4 m 클리어). **실검증: apple → (0.841, -0.102, 0.179)** — x/y 이미지 기하 일치, z = 테이블(+0.10) + 사과 높이 표면값과 4 mm 일치. person_guard는 우측 케이블 FP(conf≤0.41, 36%)로 상시 래치 → **오퍼레이터 결정으로 off**(E12) |
| **5-4 접근 데모** | 'v' 라이브 lock→approach | ✅ (2026-07-27 심야, `61a4a86`) **1차 시도에서 테이블 충돌 사고(E13)** → 원인(점-구속 IK의 nullspace 방랑 + 무게이트 quintic) 제거 후 재시도: 수직 시작=게이트 거부(정상), 기울인 시작=orange/tennis_ball/banana 접근 성공, **물체 중앙 배치 후 접근→homing('b')→다음 물체 사이클로 전 클래스 성공**. apple(0.85 m)=reach 한계 거부, mustard(x 0.906)=박스 게이트 거부(정상). z마진 15 cm 상수 — 관측 편차는 표면기준+refine tol 12 mm+미모델 공구길이(내일 반영) 합성 |
| **HOME 기능** | 메뉴 기록 + 'b' 복귀 | ✅ (2026-07-27, `61a4a86`) 'p' 메뉴 마지막 항목이 **현재 관절값 스냅샷**(IK 없음 = 브랜치 점프 원천 불가), 'b'가 속도상한 quintic으로 복귀. refine/접근 자동 취소. **07-27 주간**: HOME 미기록 시 'b'는 계산된 q_ready로 복귀(`19b15ba`) |
| **grav-comp 보완 (F/T+sticky)** | 말단 페이로드 모델 + 침하 정지 | ✅ (2026-07-27 주간) URDF tcp에 **285 g**(RFT76-HA01 200 g + 어댑터판 등) 반영 — 1차에 +Z로 넣어 악화, **공구축=−X 함정**(E16) 교정 후 CoM (−0.021,0,0). 전방 뻗은 자세 잔류 sag는 모델 바닥(태그판 비자성=알루미늄 확인, 질량 추정 유지) → **sticky-float**가 완충: dead-band 앵커 스프링(0.15·Kp·0.03 rad 상한) + **속도게이트 완화판**(|qd|>0.08 rad/s면 앵커 추종=순수 grav-comp 감각, Kd 0.08→0.03) — **오퍼레이터 "이정도면 OK"**(`5140a83`). 'k' 토글 |
| **ready-seed IK** | 접근 해의 시작자세 독립화 | 🔨 (2026-07-27, `19b15ba`→**v2 `f9a411b`**) **구현+빌드 완료, 실기 재검증 대기.** init에서 박스 중심 부트스트랩 → r-클램프 코너 8점을 런타임과 동일 방식(soft-R=q_ready의 R)으로 전수 검증 → `[SEED]` 부팅 로그. 매 'v'를 q_ready 시드로 해석(결정적), **2π 폴딩+관절한계 검사**(J3 10.42 rad류 감김 해 → −2.15 rad 등가해로 구제, legacy 경로에도 적용), Δq≤2.0 직행 / 초과 시 **자동 스테이징**(q_ready 경유, settle 0.2 s→goal 레그. 모드/서보 변동 시 즉시 취소=이연 모션 금지). 부트스트랩 실패 시 live-시드 폴백. **v1 필드 결함(E19)**: 합성 사다리가 손목 접힌 기형 브랜치(J4 2.61/J5 1.65)를 q_ready로 뽑아 "괴랄한 회전" → **v2 = 오퍼레이터 자세 앵커**: HOME 기록 시 그 자세로 즉시 re-base(`SetReadyAnchor`)+`$HOME/.indy7_ready_seed` 영속화(RT 메일박스→브릿지 워커 파일 IO), 다음 부팅은 파일 자세를 1순위 시드로. 사다리엔 \|J4\|,\|J5\|>2.0 거부 가드. 부팅 로그에 `[anchored to operator posture]`/`[synthetic — record HOME to re-base]` 표기 |

## 6. 코드 변경 요약

### RAON-RT (kv260-merge)

| 파일 | 변경 |
|---|---|
| `App/Indy7/Indy7Ctrl.{h,cpp}` | VisualServo 완전 제거(멤버·TASK5·구 'v'키·RT루프 주입블록·proc), `SetAsDCRef(slave0)` 가드콜 추가, calib 출력 경로 `App/CalibUtils/kv260/` · **Gate 2b**: 브릿지 수명주기(Init 비치명 실패 허용/DeInit), 'p'/숫자/'v' 키 핸들러(원자플래그만), RT 루프 goal 소비 SM('n'키 검증 시퀀스 재사용, ISO/RECT 상호배제·이동중 재트리거 금지) · **refine SM**(감쇠 편향+속도게이트, oriWeight=0) · **HOME**('p' 기록+'b' 복귀) · **서보-오프 goal 폐기 가드**(E15) · **ready-seed 접근**(`TryReadyApproach` 직행/스테이징 + `eAPPROACH_STAGING` 레그2: 모드·서보 불변시에만 발화) · init에서 박스 코너 프로브로 `ComputeReadySeed` 호출 |
| `App/Indy7/CalibCapture.{h,cpp}` | **ViSP-free 재작성** — Eigen `AngleAxisd`로 theta-u 변환, `vpPoseVector::saveYAML` 포맷 호환(→ `eye_to_hand_calib.py` 무수정 소비) |
| `App/Indy7/FullDynControllerRT.{h,cpp}` | x86 SSE 헤더 `__SSE__` 가드 (aarch64 빌드 차단 해소) · **E13 3중 방어**(soft-R IK·Δq 게이트·T 스케일링, FK 잔차 합격판정) · `IsTrajectoryRefDone()`(B7) · **sticky-float 홀드**(dead-band 앵커+속도게이트, 'k') · **ready-seed IK**(`ComputeReadySeed`/`SolveReadyIK`/2π 폴딩+관절한계/`ScaledTrajTime`, D13) |
| `App/Indy7/indy7.urdf` | tcp 링크에 F/T 페이로드 **285 g @ CoM (−0.021,0,0)** — 공구축이 tcp 프레임 **−X**라는 경고 주석 포함(E16) |
| `App/Indy7/Makefile` | ViSP/librealsense/OpenCV/PCL 제거, `RBDL_DIR` override 추가 · **Gate 2b**: humble+my_interfaces 배선(E5의 include **화이트리스트** 방식), rpath 내장(ROS env source 불필요), `make gate2b_test` 타깃 |
| `App/Indy7/ROS2PickBridge.{h,cpp}` | **신규(Gate 2b)** — `/pick_target_base`·`/detections` 구독 + `AsyncParametersClient`로 `desired_class` LIVE 설정. 스레딩 계약: ROS2 I/O·메뉴·통계게이트는 브릿지 자체 non-RT 스레드(spin+worker), RT는 wait-free 원자 API+SPSC goal 슬롯만. `SignalHandlerOptions::None`(앱 SIGINT 보존). 게이트: N=15, 축별 std<8 mm, 워크스페이스 박스 실측치+**radial 게이트 r≤0.80**, z마진 0.15 m · E14 세션 위생(lock 방송 세션화·lost 1 s 디바운스·desired_class 자동 초기화) · 메뉴 HOME 기록 항목 · 박스 상수 public(ready-seed 프로브 파생용) |
| `tools/gate2b_bridge_test.cpp` | 브릿지 단독 하네스(EtherCAT/로봇 불필요) — stdin 명령 p/숫자/v/q, RT 소비자 대역 poller |
| `tools/test_gate2b_bridge.py` | 합성 검증 드라이버 — 실브릿지+실 pick_logic 노드 vs 합성 `/detections`·`/pick_target_base` 피더. C1 메뉴 / C2 param ack / C3 `ros2 param get` 실증 / C4 lock / C5 게이트 통과 goal(z=0.27). E1~E3 방어 패턴 이식 |
| `tools/test_phase3_smoke.py` | **Phase 3 정본 스크립트** — P0 preflight(rtprio/memlock/슬레이브/중복실행) → P1 파이프라인 기능적 대기 → P2 앱+OP → P3 홀드(`--hold N`, E9 규칙) → P4 in-app 'p'/선택/'v' → P5 축별 SDO → P6 SIGINT DeInit 검증. `--app-only` 격리 스테이지. 앱 실행에 `MALLOC_ARENA_MAX=2`+`stdbuf -oL` |
| `App/Indy7/Indy7Ctrl.cpp` (keyboard) | E8 개행 가드 — '\n'/'\r'는 키로 취급 안 함 |
| `App/Indy7/INDY7.cfg` | **전 축 `AUTO_SERVO_ON=0`**, `ENABLE_CONTROLLER_AT_STARTUP=0`, 5태스크(VS 태스크 제거), 경로 로컬화, DC 실험용 주석 템플릿 |
| `EMasterApp/Device/EcatSlaveBase.{h,cpp}` | `RegisterPDOEntry` **UINT32→INT64** (내부 `<0` 체크도 unsigned라 이중 사망 상태였음) |
| `EMasterApp/Device/Slave{CIA402Base,NRMKEndTool}.cpp` | **죽어있던 PDO 실패 가드 27개 소생** — signed 임시변수 캡처(대입식이 unsigned면 반환형만 고쳐도 무효) |
| `CRobot/AxisCIA402.h` | `GetEcatSlave()` 접근자 (DC ref 지정용) |
| `tools/gate0_rtposix_test.c` | 1 ms 주기 지터 스모크(RT/NRT 폴백, warm-up 스킵) |
| `tools/patches/rtposix-aarch64-stackmin.patch` | RT-POSIX 스택 버그 패치 보존본 |
| `tools/calib_grab.py` | Phase 5 캡처 grabber — 파이프라인 압축토픽 구독, **태그 검출시에만 저장**(이미지 N ↔ 앱 pose_rPe_N lockstep 강제), camera_info 1회 저장(D12: 파이프라인과 동일 카메라 모델) |
| `tools/calib_cpo.py` | 이미지→pose_cPo (36h11 + solvePnP IPPE_SQUARE + 모호성비 리포트). 코너 정밀화 **per-image 폴백**(B8) |
| `App/CalibUtils/kv260/` | 16자세 데이터셋 + solve 결과(rPc.yaml/txt, 시각화) — 원저자 예제 데이터(상위 디렉토리)와 격리 |

### ros2_ws (main)

| 파일 | 변경 |
|---|---|
| `src/pick_logic_pkg/.../pick_logic.py` | **v2**: L2 스코어 선택(conf/중앙/크기 가중) + `desired_class` LIVE 파라미터('':자동) + L3 안정성 3프레임·히스테리시스·5프레임 생존 + L4 근접 person 래치. 신규 reject 사유 3종. 게이트·metrics·yield CSV 계약 불변 |
| `config/pick_logic.yaml` | v2 파라미터 블록(주석 포함). **2026-07-27 `person_guard_enable: false`**(오퍼레이터 결정, E12 — 재활성 대안 주석 포함) |
| `launch/pick_place_vitis_ai.launch.py` | `base_to_camera_tf` placeholder → **캘리브 실측치**(quaternion, §1.3) |
| `tools/vision/test_pick_logic_v2.py` | 카메라-불필요 합성 검증(실노드+합성 DetectionArray, 시나리오 6종) |

## 7. 발견한 버그·함정 대장

**RAON-RT/RT-POSIX 소스 결함** (우리가 픽스/회피한 것):

| # | 결함 | 처치 |
|---|---|---|
| B1 | RT-POSIX `DEFAULT_STKSIZE` 64K < aarch64 `PTHREAD_STACK_MIN` 128K → 기본스택 태스크 생성 전멸(EINVAL) | 소스 클램프 패치(+보존). RAON 본체는 항상 2 MB라 잠복해 있었음 |
| B2 | RT-POSIX `wait_next_period` overrun 판정 부등호 반전(미래 deadline을 overrun으로 셈) | 카운터 불신 — 판정은 dt 실측으로 (미패치, 기록만) |
| B3 | `RegisterPDOEntry` unsigned 반환 → 실패 가드 27개 + 내부 에러로그 전부 죽은 코드, 실패 시 offset 0xFFFFFFFF로 다음 사이클 OOB | INT64 + signed 캡처로 소생 (§14.2 #1) |
| B4 | 저장소에 저자 x86 빌드잔재(obj/*.d, bin, lib) 커밋됨 → aarch64에서 make 즉사 | 브랜치에서 untrack + `make clean` 선행 규칙 |
| B5 | `INDY7.cfg`에 DC 키 부재 → 슬레이브 DC 전부 OFF가 저자의 실구동 상태 | 그대로 브링업(D8), DC 켤 준비만 완료 |
| B6 | `make_csv`가 저자 홈 절대경로(`/home/raimlab/...`) 하드코딩 → 궤적 CSV 저장 전부 실패 | 로컬 경로로 픽스(`d94c1c1`) — 추종오차 분석 도구 복구 |
| B7 | `IsTrajectoryDone()`은 궤적 종료에 **settle 조건(전 조인트 0.15 rad 이내)까지 요구** → 정상상태 droop(마찰) 하에선 영원히 미완료. 저자 rect SM이 done 대신 사이클 카운터를 쓴 이유 | `IsTrajectoryRefDone()`(참조 생성 완료 기준) 신설 — refine·접근 완료 게이트는 이걸 사용(`0668ed2`) |

**개발환경 함정** (재발 방지 규칙):

| # | 함정 | 규칙 |
|---|---|---|
| E1 | `ros2 run` 래퍼에 SIGTERM → 노드 고아화. **유휴 rclpy spin은 SIGTERM을 삼킴**(EINTR 재대기, 콜백 없으면 python 핸들러 실행 기회 없음) | 하네스는 `start_new_session` + 그룹 SIGTERM 후 **무조건 그룹 SIGKILL** 마무리. (launch 환경은 트래픽이 executor를 깨워 무관) |
| E2 | SIGKILL된 노드는 미등록해제 → **ros2 daemon 그래프 캐시 오염** — half-dead 노드가 graph-count 대기를 속이고 `param set`이 죽은 노드로 감 | 준비판정은 **기능적 핸드셰이크**(빈 프레임→응답 수신)로. preflight에서 `ros2 daemon stop` + param set 재시도 |
| E3 | `pgrep/pkill -f`는 패턴이 **자기 셸 명령줄에 있으면 자살** | 킬 전 `/proc/PID/comm` 검사(python3/노드명만), 확인 명령은 별도 호출로 분리 |
| E4 | pick_logic 실행파일명은 `pick_logic` (노드명 `pick_logic_node`와 다름) | — |
| E5 | **Humble include의 CycloneDDS 지뢰**: `include/dds/features.h`·`include/idl/{string.h,endian.h}`가 glibc 표준 헤더를 섀도잉 — 저자 Indy7_ROS2 Makefile의 `find -maxdepth 1` -I 샷건이 이 디렉토리들을 포함해 libstdc++ 컴파일이 `__throw_length_error` 류로 붕괴(저자의 `-include cstdio/cstring/cstdlib`는 이것의 밴드에이드) | -I는 실제 include하는 패키지만 **화이트리스트** (`ROS_PKGS` 변수) — 섀도 디렉토리 원천 배제, 누락 시 명시적 에러로 드러남 |
| E6 | **에이전트/VSCode 셸은 PAM limits 미적용**(limits.d 이후에 뜬 세션이라도 데몬 계열은 안 탐): rtprio=0 → RT-POSIX 태스크 생성 EPERM 전멸, memlock 유한 → E7 | `sudo prlimit --rtprio=98:98 --memlock=unlimited:unlimited --pid <claude PID>`로 조상 프로세스에 주입(자식 상속) 또는 재부팅 후 새 세션. 스크립트 preflight가 둘 다 검사 |
| E7 | **`mlockall(MCL_FUTURE)` + ROS2/DDS = 잠금 폭발**: 스레드별 glibc malloc arena(가상 64 MB)·8 MB 스택이 전부 물리 잠금 → memlock 490 MB 한도 초과 시 **할당 자체가 실패**(`std::bad_alloc`, OP 중 앱 사망). Gate 2b 하네스는 mlockall이 없어 잠복 | memlock **unlimited 필수** + 앱 실행 환경 `MALLOC_ARENA_MAX=2`. **사건**: 이 크래시가 유발한 동시 teardown(마스터 해제+DPU+realsense) 중 **커널 하드 락업 1회**(저널 무flush 단절, 21:56:11) → 보드 재부팅. 트리거(앱 비정상사망) 제거로 재발 방지; 재발 시 시리얼 콘솔 필요. 슬레이브는 SM watchdog으로 SAFEOP 낙하 = 버스 측 안전거동 정상 |
| E8 | **파이프 stdin 키 레이스**: `"p\n"` 두 글자가 한 번에 도착하면 keyboard 태스크가 `m_cKeyPress`를 'p'→'\n'으로 즉시 덮어써 1 kHz 소비자가 'p'를 못 봄(실터미널은 사람 속도라 저자 미조우) | 앱 keyboard 태스크에 '\n'/'\r' 무시 가드 + 스크립트는 개행 없이 단일 문자 전송 |
| E9 | IgH `ethercat master`의 **Lost frames는 감소 가능한 파생 통계**(지연 프레임이 뒤늦게 계상되면 ↓). PREOP→OP 활성화 천이 자체가 프레임 2개 정도 유실(정상) | 유실 판정은 **OP 도달 후 재베이스라인** + **증가 시에만 FAIL** |
| E10 | **E-stop = 로봇측 버스 전원 차단**: eth0 Link DOWN + 슬레이브 0 → 앱은 "There are 0 Responding Slaves!"로 초기화 실패. 버튼이 물리 래치라 해제 전까지 지속 | 버튼 비틀어 해제 → 슬레이브 7 복귀·전축 0x0220 fault-free 확인(2026-07-26 실측) 후 재실행. E-stop 실효성 테스트로는 오히려 정석 경로 |
| E11 | **rosidl typesupport는 런타임 dlopen** — 실행파일의 RUNPATH(-Wl,-rpath)가 dlopen 검색에 적용되지 않아 env 미소스 실행 시 `libmy_interfaces__..._fastrtps_cpp.so` 로드 실패(브릿지만 비활성, 앱은 진행) | 앱 실행은 항상 `App/Indy7/run.sh`(ROS env source + MALLOC_ARENA_MAX=2 + rtprio/memlock preflight) |
| E12 | **person_guard 상시 래치 FP(2026-07-27)**: 테이블 우측 가장자리의 케이블/그림자 세로 영역이 conf 0.32~0.41 person으로 **36% 프레임**에서 오검출(area 0.08~0.10 ≥ 0.06) → 사람이 없어도 2 s 래치가 계속 갱신되어 타깃 전면 무효. 증거 프레임 확보 | **오퍼레이터 결정으로 guard off**(yaml). E-stop이 주 안전장치. 재활성 시 `person_min_confidence: 0.5`면 이 FP는 걸러짐(실측 FP max 0.41 — 근접 실사람은 통상 그 이상) |
| B8 | `CORNER_REFINE_APRILTAG`이 기본 검출기가 잡는 마커를 **떨어뜨릴 수 있음**(image0012). cPo 하나가 비면 기존 solver의 `sorted(glob)` zip 페어링이 **그 뒤 쌍 전부를 조용히 어긋나게 함**(사전순 정렬이라 1,10,…,16,2,… 순서 — 인덱스 결번 시 재앙) | `calib_cpo.py`가 정밀화 방법을 per-image 폴백(APRILTAG→SUBPIX→기본)으로 시도해 결번 자체를 방지 + 16/16 강제 |
| **E13** | **접근 1차 시도 테이블 충돌(2026-07-27 01:32)**: `SetTargetPosePositionOnly`가 **점 구속 하나뿐인 3-DOF 구속 IK**라 RBDL이 1000 iter 동안 3차원 nullspace를 방랑 — "현재 자세 유지"는 주석뿐, 해가 현재 관절에서 임의로 먼 구성으로 수렴 가능. 관절공간 quintic이 그 델타를 3 s에 쓸며 **TCP가 테이블 관통 호를 그림**. quintic 최고속 구간(t≈2 s)에서 Axis4 드라이브 FAULT(0x0000), 공구 스크래치 경미. goal 자체는 완벽했음(banana 0.652,-0.058,0.269 = 테이블+17 cm) — 인식이 아니라 모션 생성 결함. 'a'가 멀쩡했던 건 full 6-DOF 구속(SetTargetPose)이었기 때문 | 3중 방어(`61a4a86`): ① orientation **soft** constraint(w=0.3; hard로 걸면 손으로 잡은 시작 자세에 과민 — 합격판정은 RBDL 수렴플래그가 아니라 **FK 잔차<2 mm**로, soft 잔차는 플래그를 영원히 false로 두므로) ② **Δq 게이트**: 한 관절이라도 2.0 rad 점프 시 REFUSE ③ **T 스케일링**: quintic 피크 관절속도 ≤0.6 rad/s로 T 자동 연장(≤10 s). 부수: 접근/refine SM의 중복 StartJointTrajectory 제거(스케일 T 덮어쓰기 방지), StartRefine이 Pose 전체 수령(구버전은 rotation identity 잠복) |
| E14 | **desired_class는 pick_logic의 LIVE 파라미터라 앱보다 오래 산다**: 앱 재시작 후에도 이전 세션의 orange가 남아 부팅 즉시 stale 타깃 lock 방송. lock-watch가 세션 개념 없이 상시 방송 + 검출 깜빡임 무디바운스 → 초당 LOCKED/lost 스팸 | 브릿지(`61a4a86`): 방송을 **메뉴 선택~GOAL READY 사이로 한정** + lost 1 s 디바운스 + **worker 기동/DeInit 시 desired_class 자동 초기화**(DeInit은 500 ms 1회 best-effort — teardown 신속 유지 E7) |
| E15 | **j→v 브레이크 트랩(2026-07-27)**: 브레이크 잡힌('j') 상태의 'v'는 접근+refine 전체가 얼어있는 팔을 상대로 돌아 거대한 "droop"으로 읽힘 — "전 물체 접근 실패"의 실체가 이것(오퍼레이터가 진단). 이연(deferred) 처리하면 'h' 순간 예상 못 한 모션이 튀는 게 더 위험 | 접근/'b'에 **IsServoOn 전축 가드** — goal은 **폐기**(이연 금지) + "h 먼저" 안내. 같은 원칙이 스테이징 레그2 취소 규칙에도 적용(모드/서보 변동 시 stash 폐기) |
| E16 | **기계변환 URDF의 프레임 축 함정**: tcp 프레임의 공구축이 +Z가 아니라 **−X** (관절 프레임 순열). F/T 페이로드를 관례대로 z=−0.060에 달았더니 질량이 옆으로 매달려 **수직 유지가 오히려 악화**(오퍼레이터가 즉시 감지). 캘리브 부산물 eMo x=−29.4 mm(플랜지면 −10.9 + 스택 18.5)로 −X 확정 — **TCP 원점은 태그면 안쪽 29 mm**(호버 실클리어런스 ≈ 150−29=121 mm) | CoM (−0.021,0,0)으로 교정 + URDF에 축 경고 주석. 교훈: 이 URDF에서 "tool 방향" 가정 금지 — 프레임 관련 삽입은 FK/캘리브 부산물로 축부터 실증 |
| E17 | **터미널 Jacobian servo 사고(2026-07-27)**: "한 번에 접근" 시도로 접근 말단을 누적형 DLS servo(적분 복원)로 교체 → 손목 특이점 근처에서 DLS가 **J5를 9.39 rad 감아** 사과는 이상한 곳으로, 나머지는 전부 REFUSED. 감김 워치독·태스크 진척 워치독·손목 nullspace 처리 없이 누적 적분만 넣은 게 원인 | **전면 리버트**(git checkout, URDF 픽스는 보존 확인). 재시도 전제조건 3종을 부검으로 명문화: ① 관절 감김 워치독 ② 태스크 진척 워치독(오차 미감소 시 중단) ③ 손목 특이점 nullspace 처리. 당분간 refine(12 mm, 2~6 pass)이 프로덕션 — "여러 번 시도"는 속도 IK 탓이 아니라 **stiction 바닥의 반복 보정**이 정체 |
| E18 | **soft-R×refine 회귀(2026-07-27)**: E13 방어로 넣은 soft-R(w=0.3)이 refine 재타겟에도 걸려 **~10 cm급 편향에서 위치/자세 gradient 평형으로 IK 정지**(apple 155 mm 잔차 → pass2 97.8 mm 정체). refine 편향은 소규모 국소 이동이라 R 유지가 불필요 | refine 재타겟은 `oriWeight=0.0`으로 호출(접근 1차 명령만 soft-R). 같은 평형 메커니즘이 접근 실패(residual 133.7 mm)의 원인이기도 → D13 ready-seed가 시드 차원에서 제거 |
| E19 | **합성 부트스트랩의 기형 자세(2026-07-27 저녁, ready-seed v1)**: 시드 사다리+순수 위치 IK가 뽑은 q_ready가 **도달·한계·Δq 전부 합격인데 물리적으로 기형**(J4 2.61/J5 1.65 rad — 손목이 몸통 쪽으로 접힘, 5/8 코너도 통과). 스테이징이 그 자세로 이동하니 "로봇 근처에서 괴랄하게 회전"으로 관측, 자가충돌 위험도 있었음. 교훈: **기하 조건만으로는 자세의 sane함을 판정할 수 없다** — 사람이 시연한 브랜치가 유일하게 신뢰 가능한 앵커 | v2(`f9a411b`+`f3c4569`): ① HOME 기록=즉시 앵커 re-base+파일 영속화(다음 부팅 1순위 시드) ② 사다리 후보 \|J4\|,\|J5\|>2.0 즉시 거부 ③ 부팅 로그에 앵커/합성 출처 표기 ④ **기록 즉시 커버리지 검증**: 8프로브를 사이클당 1해석(amortized — 일괄 9해석은 1 kHz 루프 정지)으로 재검증 → `anchor coverage: n/8` + 미달 좌표 로그. 사람의 역할은 "정상 브랜치 1비트"뿐, 도달성 판정은 계산이 담당. 운영 수칙: **세션 첫 행동 = 자연스러운 자세에서 HOME 기록**(전 생애 1회 — 파일 영속, 접근 시 그 자세에 있을 필요 없음). **후속: 손목 가드로도 부족 판명(2차 합성도 J2 −2.80 팔꿈치 접힘) → v3(E20)에서 사다리 완전 제거 — q_ready는 오직 시연 자세**. **v3 실기(20시)**: 파일 앵커 부팅([operator anchor] 6/8, 감김 turn 로그 정상), 괴랄 궤적 소멸, apple/mustard/tennis 전부 직행(dq 0.17~0.79) refine 수렴 9.6~11.9 mm — **모션 스택 정상**(도달점이 물체를 벗어난 건 E21 베이스 이동) |
| **E20** | **멀티턴 카운터 감김 = 랩 불일치 폭탄(2026-07-27 저녁)**: 기록된 HOME이 **J1 −6.21 / J4 −18.93 rad**(물리 자세는 0 근처 — E17 서보 감김 + E-stop 버스전원 사이클이 멀티턴 인코더를 재참조하며 통째 회전수가 카운터에 잔류, **랩은 기록/재생 시점 간 달라질 수 있음**). FK·중력보상·IK는 2π 주기라 제어는 멀쩡히 돌지만: ① 한계검사가 감긴 값을 거부 → 앵커 실패·refine 전체 오거부 ② canonical 목표와 감긴 카운터를 섞으면 **관절 목표가 실제 다중 회전 명령이 됨** — 'b' 괴랄 궤적의 정체, staged 'v'였다면 J4 ~20 rad 회전 시도 | v3(`0662b98`) 원칙: **판정은 물리 자세(0-접힘)로, 실행은 라이브 카운터 좌표계로** — 모든 관절 목표를 명령 직전 현재 카운터 쪽으로 접어 **관절당 이동 ≤π 보장**. `CheckLimitsPhysical`/`FoldTowardRef` 분리, 적용처=SetTargetPosePositionOnly(refine 오거부 해소)·SolveReadyIK 출력·스테이징 레그·'b' 재생·TickAnchorVerify. SetReadyAnchor는 감긴 입력을 접어서 수용(+관절별 감김 turn 수 로그), 저장은 canonical만. 합성 사다리 제거(앵커 없으면 live-시드 폴백+안내) |
| **E21** | **로봇 베이스 물리 이동(2026-07-27 20시)**: v3 실기에서 접근이 FK상 ±1 cm 도달인데 실물은 물체를 ~11 cm 벗어남 + banana/orange가 갑자기 r-게이트 탈락. **손-프로브 판별**(사과 꼭대기 접촉 's'): 로봇 FK (0.744, −0.138) vs 카메라 보고 (0.681, −0.235) → **Δ(+6.3, +9.7) cm, z는 8 mm 일치** = 테이블 평면 강체 이동(기울지 않음). 원인 = E17/E19 사고 시 팔 요동으로 베이스가 밀림(오퍼레이터도 육안 확인). 캘리브 rMc는 "옛 베이스" 기준이라 전 좌표가 계통 오프셋 | ① **베이스 볼트/클램프 고정**(재발 방지 — 안전) ② 멀리 떨어진 3점(apple+tennis+banana류) 손-프로브 ↔ LOCKED 좌표쌍으로 **2D 강체(yaw+t) 최소자승 fit** → launch TF에 합성(빌드 불필요, 파이프라인 재시작만). 잔차 1~2 cm 초과 시(카메라도 움직인 비강체) 16-pose 정식 재캘리브(도구 준비돼 있어 20~30분). 교훈: **접근이 "자신 있게 빈 공간"으로 가면 모션이 아니라 좌표(외부 캘리브) 붕괴부터 의심** — 판별은 손-프로브 5분. **TF 보정 후 apple/tennis 접근 성공으로 복구 검증(21시대).** **복구 완료(같은 날 저녁)**: 데이터 함정 2회를 강체성 검사가 잡아냄(① 락과 프로브 사이 물체 이동 — 거리 불변량 위반 최대 13 cm ② 접촉 해제 후 's' — TCP z가 꼭대기+30 cm, 세 프로브 모두 공구좌표 동일점(~37 cm) 접촉으로 역산됨) → sticky-float로 접촉 유지 프로브 재취득 → **2점(apple+tennis) fit 잔차 0.2 mm·상호거리 0.4 mm 일치, banana 검증 2.1 cm(길쭉한 물체의 bbox중심 vs 접촉점 모호성), 3점 fit과 ≤1 cm 일치** → yaw +5.14°/t(+4.5,+10.4) cm 확정, launch TF 교체(§1.3). 베이스 볼트 고정 완료. 주의: 보정 후 banana는 신좌표 r=0.84로 r-게이트 정당 거부 — 5 cm 안쪽으로 옮겨야 접근 가능(→ E22에서 게이트 자체를 재보정) |
| **E22** | **게이트 상수도 옛 프레임 유물(E21 후폭풍, 21시대)**: TF 보정 후 orange/mustard/banana가 박스(x≤0.85)·r-게이트(0.80)에 걸렸는데 손-유도로는 전부 도달 가능. 두 상수 모두 **밀리기 전 좌표계에서 어림잡은 선**이라 5.14°/11 cm 프레임 변화 후엔 오보정(r 0.80의 근거였던 "orange 0.87 스톨" 라벨도 옛 프레임 값). 부수 규명: **z 마진 실측 10 cm = 명령 15 cm(TCP 기준) − 공구 스택 ~5 cm**(프로브 z델타 +4.5~6 cm로 실측) — 설계대로이며 파지 단계 TCP 재정의 때 물리 기준화 | (`e69555c`) 박스를 fit 변환으로 재베이스 x[0.30,0.94] y[−0.37,0.63] z 불변, r-게이트는 0.92 조잡 안전망으로 강등 — **도달 판정의 심판은 ready-seed IK**(목표별 해석, 불가 시 무동작+명확 거부). 유의: hover(꼭대기+15 cm, 수직계열 R)는 터치보다 reach 요구가 큼 — IK가 soft-R을 ~40°까지 굽혀 시도하고도 거부하면 그게 진짜 hover 한계 |
| **E23✅** | **원거리 soft-R 정체 재발 + 'd' 허위 오차(21시대)**: 게이트 완화 후 banana(r 0.83)가 잔차 235 mm, mustard(r 0.86)가 62 mm로 거부 — **가까운 쪽이 더 크게 "모자란" 것은 reach가 아니라 E18 gradient 정체**(잔차 = 부족량이 아니라 솔버가 멈춘 위치). 계산상 banana는 공구 ~40° 기울이면 hover 가능. 부수: 'd'(LogDistanceError)가 Target 0,0,0 대비 0.79 m/214° 출력 — ready-seed 경로가 `goal_tcpPose` 레지스터를 안 채워 초기값과 비교된 허위 수치(제어 오류 아님; 실제 오차는 `[REFINE] done` 줄) | (`1694414`) **w 사다리**: soft-R(0.3) 실패 시 같은 q_ready 시드로 순수 위치(w=0) 재시도, 해의 공구 기울기가 ready 자세 대비 **60° 이내일 때만 수용**(초과=extreme tilt 거부). 실패 시 잔차가 진짜 부족량에 가깝게 보고됨. 방어선(폴딩·물리한계·Δq 게이트·스테이징)은 뒤에서 유지 — E13 방랑 위험 격리. + TryReadyApproach가 goal_tcpPose를 채워 'd' 정상화. **실기 검증(22시대): 전 5클래스 접근 성공** — banana 41°/orange 44°/mustard 48° tilt(= 수직-hover 한계 밖임을 자세가 표시; 손목이 tool길이·sinθ만큼 안으로 들어와 도달), apple은 soft-R 경로로 평행 유지. refine 최종 4.9~10.6 mm. 기울기 개선은 파지 단계 top-down 고정 R에서 정식 해결 |
| **E24✅** | **파일에서 나온 손목 0점 = 중력보상 폭주(2026-07-27 22:47)**: AXIS4/5만 `HOMING_METHOD=1`(eAxisHomeStartPos) → 첫 eServoIdle에서 `home = raw_start − POS_BEFORE_EXIT` 래치(`CRobot/AxisNRMKCore.cpp:75`) → **시작 각도 = cfg의 PBE를 각도로 디코드한 값**이며 팔의 실제 위치와 무관. 성립 조건은 "**시작 자세 == 직전 종료 자세**"인데 아무도 검사하지 않음. 22:16~22:29 성공 런이 접힌 자세로 종료하며 DeInit이 PBE 갱신(`CRobot/Robot.cpp:87-94`) → 재부팅 사이 팔을 수직으로 세움 → 22:47 실행 시 모델은 J4를 **−93.29°**로 믿음(실제 −10.3°, **오차 81.7°**) → `h` 순간 중력토크가 0이어야 할 손목에 **2.22 N·m** 허위 토크 → 손목이 **능동적으로 −93.6° 넘어감**(처짐이 아니라 구동됨) → 원위 질량이 어긋나 어깨·팔꿈치도 주저앉음(sticky-float는 0.15·Kp·0.03 rad 상한이라 못 잡음). 무너진 자세에서 HOME 기록 → `SetReadyAnchor`의 유일한 게이트가 관절한계(J4 3.0207 < 3.0543 통과) → **m_EReady 재베이스 = 잣대가 뒤집힘** → 같은 물체가 119/136/139° tilt로 전량 거부. 한 달간 무사했던 이유: 6/18~7/26 PBE가 계속 "손목 곧은" 값(J4 −1.33°, J5 −0.006°)이었고 오퍼레이터가 늘 팔을 세워두고 시작 — **"접힌 채 종료 → 세워서 시작" 조합이 처음** | ① **AXIS4/5 → `HOMING_METHOD=3`**(eAxisHomeManual, AXIS0~3와 동일한 고정 `HOME_POSITION_OFFSET` 기준 — `App/Indy7/Indy7Ctrl.cpp:426`): 파일 유래 0점 자체를 제거, **종료 자세 규칙 폐기** ② 검증은 **엔코더 카운트 고고학** — pose 출력은 `FK(m_Q)`라 자기 0점을 검증할 수 없음(순환논법, 잔차 3e-14는 어떤 HPO에서도 나옴): HPO_4는 5/28 park raw와 **정확히 1회전 modulo 437카운트(0.024°)**, HPO_5는 2614카운트(0.142°), 5/17~7/26 커밋된 park 7개가 J4 3.53°/J5 2.62° 밴드에 군집 → **잔차 0점 오차 ≤2.8° = 허위 토크 ≤0.16 N·m**(폭주값의 1/14) ③ 교차확인: cfg 카운트로 예측한 J5 **−32.0858°** ↔ pose 역산 **−32.0859°** = 디코드·조인트매핑·FK 체인 정상 + 롤축 무이동 ④ 실기: 플랜지면 수평계 **23°**(예측 21°, 실패가설 10~12°) → `g`/`h` 정상 유지 → **전 5클래스 재현**. **미구현 가드**: 래치 시 두 기준 잔차 교차검증(당시 91.96° → 즉시 정지 가능했음), `h` 무장 전 전축 `IsHomeSet()` + 모델이 믿는 q/FK 출력, `SetReadyAnchor` 자세 급변 거부 + temp→rename+`.prev`(좋은 앵커가 복구불능으로 소실됨), DeInit의 PBE 샘플링을 브레이크 낙하 **이전**으로. 부수 발견: `POSITION_LIMIT_U/L=±35.0`인데 q가 라디안 → **소프트웨어 관절 리밋 부재**(35 rad=2005°), `ONE_TURN_REF=360.0`은 사문(`Axis.cpp:506-512`가 revolute면 미할당), `ABSOLUTE_ENCODER`는 파싱만 되고 호출처 없음 |
| **E25⏳** | **`w=0` 폴백의 브랜치 사고(2026-07-28 01:46, 미해결)**: E23 "사다리"가 실제로는 2단(0.3 → 0)뿐이라 soft-R 실패 시 **자세를 전혀 구속하지 않는 순수 위치 해**로 점프 → 수용 여부가 하강 경로의 우연에 좌우됨. 새 앵커(툴이 수직에서 9.8°)로 mustard(베이스원점 3D 0.912 m) **36° 통과** / **banana(0.877 m — 더 가까움) 87° 거부** / orange(0.946 m) 102° 거부 — **더 쉬운 목표가 더 크게 튄 것 자체가 물리가 아니라 솔버 아티팩트라는 증거**. mustard는 어제 48°→오늘 36°로 오히려 개선(앵커가 나빠서가 아님) | (예정) **진짜 w 사다리** 0.3→0.1→0.03→0.01→0 중 첫 수렴 채택 + w=0 해를 시드로 한 **soft-R 재정제**(위치 유지, 자세만 당김). 원칙: ready 자세의 툴 각도는 작업공간 요구각의 **중앙값**이어야 함(가까운 물체 ~0°, 원거리 ~45~50° → 수직 앵커는 한쪽 극단) — 앵커 재기록은 부차적 |
| **E26⏳** | **refine 진동·미수렴(2026-07-28 01:56, 미해결)**: tennis_ball refine이 39.5→36.3→23.1→18.6→**26.2**→13.1 후 최대반복 도달, **잔차 14.5 mm**로 종료(목표 (0.705,0.159,0.314) vs 실제 (0.700,0.148,0.323) = 11 mm 이탈). 같은 세션 apple 4.4 mm / mustard 4.3 mm는 정상 수렴 → 목표 의존. 진행 중 `R held within 11~17°`로 자세가 계속 흔들림(refine은 oriWeight=0인데 해가 자세를 흔듦) | (예정) 감쇠 0.65·최대 6회 재검토 + 자세 자유도와의 상호작용 분리. 파지 <5 mm 요구 전에 반드시 해결 |

**★★ 하드웨어 제약 — J2 고정 = 이 로봇은 실질 5-DOF (오퍼레이터 확인, 2026-07-28 03:50)**:
베이스를 J0로 셀 때 **J2(팔꿈치)에 lock이 걸려 있고, 서보온 상태에서도 움직이지 않는다.
의도적으로 잠가둔 것**(원래부터). 즉 제어 가능 자유도는 6이 아니라 **5**다. 지금까지의 모든
IK·게이트·커버리지 판단이 6-DOF 가정 위에서 이뤄졌으므로 **전면 재해석이 필요하다.**

- **당장의 버그**: RBDL IK는 6축이 전부 자유롭다고 보고 **J2를 움직이라는 해를 계속 생성**한다
  (로그 실물: `[REFINE] dq_max 0.11 rad (J2)`, `0.04 rad (J2)`). 그 해는 달성 불가이고 refine이
  나머지 관절로 보상하려다 진동 → **E26의 직접 원인 후보**. **IK 자유도에서 J2 제외가 최우선
  작업**(제외 전에는 E25/E26 진단을 확정하지 말 것).
- **자유도 산술(로드맵에 직결)**: 위치만 구속(3) → nullspace가 3차원이 아니라 **2차원**.
  위치(3) + 완전 자세(3) = 6구속은 5-DOF로 **원리적으로 불가능**. 따라서 §9의
  **"파지 단계 top-down 고정 R"은 현 상태로 달성 불가**다. 실현 가능한 최대치는
  **툴축 수직(2구속) + 위치(3) = 5구속/5-DOF = 정확결정**(고립해, 여유 0)이며, 여기에
  그리퍼 yaw까지 지정하면 6구속이 되어 다시 불가능해진다.
- **E25 재해석**: 원거리에서 soft-R이 수렴 못 하는 것이 솔버 아티팩트만이 아니라 **자유도 부족**
  일 수 있다. w 사다리는 여전히 옳은 수정이지만 기대치는 낮춰 잡아야 한다.
- **커버리지 `n/8`은 낙관치**: 6-DOF 가정으로 계산된 값이므로 실제 도달률은 그보다 낮다.
- **★ 최대 레버리지 질문(다음 세션 1순위)**: **이 lock을 해제할 수 있는가.** 해제 가능하면
  E25·E26·top-down R 문제군이 대부분 자연 소멸한다. 잠근 이유(안전 조치인지, 축 손상 격리인지)와
  해제 가능 여부를 먼저 확인한 뒤에 5-DOF 전제로 재설계할지 결정할 것.

**안전 제약 (실기 확인, 2026-07-26)**: **팔 전방-하강 경로에 카메라 마운트가 있어 충돌 가능**
(오퍼레이터 실측 확인). → 'a' 위치추종의 하드코딩 목표 검증 선행, Phase 5 워크스페이스 박스에
카메라 배제 존 반영 필수.

## 8. 대화형 선택 계약 (파이프라인 ↔ RAON-RT) — **구현 완료(2026-07-26, `f7c27fc`)**

구현된 확정 흐름 (`CROS2PickBridge` + Indy7Ctrl):

1. 오퍼레이터 `'p'` → 브릿지가 `/detections` 최근 ~1 s 집계(클래스당 ≥3프레임, person 제외
   — person 감지 시 경고줄만) → 번호 메뉴 + 마지막 항목 **AUTO**(desired_class 해제=자동 최적)
2. 숫자 입력 → `AsyncParametersClient`로 `/pick_logic_node`의 `desired_class` 설정
   (rcl_interfaces `set_parameters` 서비스, 2 s×3회 재시도). `0`=취소
3. pick_logic v2가 그 클래스만 통과·안정화 → `/pick_target_base` 유효 스트림 →
   브릿지 "TARGET LOCKED: <class> @ base(x,y,z)" 표시 (신선도 <1 s 감시, lost 시 표시)
4. `'v'` → **N=15프레임(≈1 s, 타임아웃 3 s) 수집 — `target_valid && depth_valid` && 수집
   시작 시점 클래스 일치 샘플만** → 축별 std<8 mm && goal=(x̄, ȳ, z̄+0.15) 워크스페이스 박스
   통과 시 SPSC 슬롯으로 RT에 전달 → RT가 grav-comp→위치전용 IK→quintic(3 s)→IK6dof+CTC
   ('n'키 검증 시퀀스). **v1은 현재 TCP 자세 유지(위치만)** — top-down 고정 R은 URDF TCP
   관례 검증과 함께 Phase 5에서
5. 안전: 게이트 전 조건 불충족 시 사유 출력 후 거부, 이동 중(`eAPPROACH_MOVING`) 및
   ISO/RECT 활성 중 재트리거 금지, goal 미소비 상태 덮어쓰기 금지, person_guard는
   파이프라인이 원천 무효화, 도달 후 홀드('g'로 grav-comp 복귀)

> 키 변경 이력: 계약 초안의 접근 트리거 `'g'`는 기존 **grav-comp 키와 충돌**해 `'v'`(vision,
> 구 VisualServo 키 재사용)로 확정. `'p'`/숫자는 기존 키맵과 무충돌.

## 9. 남은 것

- ~~위치 정확도 개선~~ **완료(2026-07-27)**: refine으로 **10.9 mm** — §5 표, `0668ed2`
- ~~Phase 5-1~5-4~~ **완료(2026-07-27)**: 캘리브 → rMc → TF/박스 정본화 → apple 실좌표 검증
  → **접근 데모 전 클래스 성공**(접근→'b' homing→다음 사이클) — §5 표,
  `8093e8a`/`a9644dc`/`61a4a86`. E13 사고·픽스 포함. 물리 검증 완료: 테이블-베이스 높이차
  ~10 cm(depth +0.096 일치), apple 수평거리 85 cm(계산 0.847 일치)
- ~~grav-comp 보완~~ **완료(2026-07-27)**: F/T 285 g URDF(−X 축, E16) + sticky-float
  속도게이트판 오퍼레이터 OK — §5 표. 잔류: 전방 뻗은 자세 미세 sag(모델 바닥, sticky가
  완충 — 필요 시 HOLD_KP_FRAC/DB 상향 나사만)
- ~~접근 IK 모드 검토~~ **결론(2026-07-27, D13)**: task-space servo 1차 시도는 손목 감김
  사고로 리버트(E17, 전제조건 3종 명문화 후에만 재시도) → **ready-seed IK 채택**(`19b15ba`,
  자동 스테이징이 경유점 스테이징 아이디어를 흡수). REFUSED→home→재시도 수동 레시피의
  자동화도 이 안에 포함됨
- ~~ready-seed 실기 검증~~ **v3 확인(2026-07-27 20시)**: [operator anchor] 부팅·괴랄 궤적
  소멸·전 접근 직행 FK 9.6~11.9 mm — 스테이징 레그·취소 키는 아직 실기 미조우(전부 직행)
- ~~E21 복구~~ **완료(2026-07-27 21시대)**: 베이스 고정 → 프로브 강체 fit → launch TF 보정
  → 전 5클래스 접근 재검증(§7 E21/E22/E23)
- **다음(즉시): E25 w 사다리** — soft-R 실패 시 0.3→0.1→0.03→0.01→0 중 첫 수렴 채택 +
  w=0 해를 시드로 한 soft-R 재정제. 그다음 **E26 refine 진동**. 둘 다 빌드+실기 필요
- **E24 후속 가드(미구현, §7 E24)**: 호밍 래치 두 기준 교차검증 / `h` 무장 전 전축
  `IsHomeSet()` + 모델이 믿는 q·FK 출력 / `SetReadyAnchor` 자세 급변 거부 +
  temp→rename+`.prev` / DeInit의 PBE 샘플링을 브레이크 낙하 이전으로 /
  `POSITION_LIMIT_U/L`이 라디안이라 관절 리밋 사실상 부재(±35 rad = 2005°)
- **파지 단계 준비**: TCP 재정의(현 원점은 태그면 안쪽 29 mm — 그리퍼 TCP로 이동, E16),
  <5 mm 정밀도(게인/적분 검토, refine 바닥 12 mm), top-down 고정 R, 물체 6D pose
  estimation(나중 — 들어오면 soft-R 타깃만 물체 기준으로 교체)
- **D10 3+1 격리 배치 완료(2026-07-27 22~23시)** — cmdline + cfg `CPU=` 키 + /proc 토폴로지
  확인 + EtherCAT OP/Lost 재베이스. **순수 지터 A/B만 미완**: 앱의 RT97/95가 같은 CPU3를
  점유해 gate0(prio 90)이 앱 뒤에 줄서는 값을 재는 오염 발생 → **앱 정지 상태**에서 gate0을
  CPU3/CPU0으로 각각 돌려야 유효. 비격리 베이스라인: CPU3 avg 1.0/max 33.9 µs,
  CPU0 avg 1.2/max 104 µs
- **백로그**: 연속 추적(closed-loop), 해석적 IK(브랜치 전수 열거 — E16 프레임 뒤틀림
  리스크로 후순위), task-space servo 재시도(E17 전제조건), MuJoCo sim 합류, indy_iface
  GUI, DC sync0 실험(cfg 주석 해제), RPU+SOEM 트랙

## 10. 재개 가이드 (새 세션용)

1. 이 문서 + 메모리 `raon-vs-merge-plan.md`부터. RAON-RT 코드 질문은 재탐색 말고
   `~/RAON-RT_guide_for_CLAUDE.md`의 해당 §.
2. 작업은 `~/RAON-RT-Revision` **kv260-merge 브랜치에서만**. lab 저장소 push 금지.
3. EMasterApp 수정 시 실행 전 `sudo make install` (루트에서) — /opt가 런타임 정본.
4. 로봇 전원만 켜면 `ethercat slaves`로 7슬레이브 즉시 확인 가능(일반유저 CLI 가능).
5. sudo가 필요한 단계는 명령을 준비해 사용자에게 요청(비대화형 sudo 불가).
6. **운영 수칙(E24 이후 갱신)**
   - 세션 첫 행동 = **자연 자세에서 HOME 기록**(앵커). 전 생애 1회지만 앵커가 오염되면 재기록.
   - **종료 자세는 이제 무관**(AXIS4/5도 HOMING_METHOD=3 → POS_BEFORE_EXIT를 아무도 안 읽음).
     "팔을 세워두고 종료" 규칙은 폐기.
   - 종료는 **Ctrl-C**. `'q'`는 `DeInit`을 안 타서 종료 진단 로그(`AXIS[n] CurrentRawPos/
     HomePosition/Diff`)와 PBE 갱신이 없음.
   - ⚠️ `s`(TCP pose 출력)는 `App/CalibUtils/kv260/robot_poses.csv`와 `pose_rPe_*.yaml`을
     **덮어쓴다**(캘리브 세트 파괴). 진단용으로 눌렀으면 곧바로
     `git checkout -- App/CalibUtils/kv260/`로 복원할 것.
   - **★ 6/8 앵커 시드 보존(2026-07-28 03:41:51 기록)**: `q = [0.26, −0.90, −0.87, −0.14, 0.15, −0.56]`
     rad — 커버리지 **6/8**(미달 2점은 z=0.50 최상단 코너 (0.30,0.63,0.50)·(0.76,0.51,0.50)로
     실제 물체와 무관). 이후 다른 자세 실험으로 `~/.indy7_ready_seed`가 덮어써졌으므로
     **`~/.indy7_ready_seed.6of8-20260728T0341`에 별도 보존**했다. 복원 =
     `cp ~/.indy7_ready_seed.6of8-20260728T0341 ~/.indy7_ready_seed` (앱 실행 전에).
     정밀도는 로그 표기대로 소수 2자리(±0.005 rad = ±0.29°) — 시드는 브랜치 선택이 역할이고
     자세 기준선으로도 0.3°는 무시 가능하므로 충분하다. 참고로 현재 파일에 남은 값은
     03:46:34에 기록된 다른 자세 `[0.248, −0.844, −0.719, −0.024, −0.750, −0.560]`이다.
   - **J2 판독값이 기록마다 다르다(미해명)**: 잠겨 있다는 J2가 세 번의 HOME 기록에서
     **−0.92 → −0.87 → −0.72 rad(총 11.5°)** 로 변했다. 완전 강체 고정이면 나올 수 없는 값이므로,
     lock의 성격(컴플라이언스·백래시 / 판독 신뢰도 / 카운터 재참조)을 다음 세션에서
     반드시 확인할 것. **IK에서 J2를 제외할 때 "어느 값으로 고정할지"가 이 답에 달려 있다.**
   - 앱에는 **로그 싱크가 없다**(`~/.ros/log/Indy7Ctrl.out_*.log`는 전부 0바이트, run.sh는
     stdout으로만). 사고 조사 시 터미널 스크롤백이 유일한 사본이므로 세션 종료 전에 덤프.
