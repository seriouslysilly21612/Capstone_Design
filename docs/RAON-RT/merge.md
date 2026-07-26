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

## 0. 30초 요약 (2026-07-26 현재)

- **목표**: perception 파이프라인은 그대로 살린 채, `/pick_target_base`(base_link 좌표)를
  RAON-RT 제어 앱이 구독 → 오퍼레이터가 터미널 메뉴로 물체 선택 → `z+margin`, top-down
  자세로 **look-then-move 접근** (CST 토크 + RBDL, 1 kHz).
- **완료**: Phase 0(기반: RT-POSIX·EMasterApp aarch64) → Phase 1(버스 검증: 7슬레이브 PDO
  비트단위 일치) → Phase 2a(ViSP 제거 앱 빌드 + §14.2 픽스) → **pick_logic v2**(선택·안정성·
  person 가드, 합성테스트 6/6) → **Gate 2b**(ROS2 브릿지 + 대화형 메뉴 'p'/숫자/'v',
  합성테스트 5/5 ×2연속).
- **다음**: Phase 3(서보-오프 통합 런, 파이프라인 동시) → Phase 4(서보온 grav-comp) →
  Phase 5(손-눈 캘리브 → TF 정본화 → 접근 데모).
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
  'v' 키: N프레임 통계 게이트(std·워크스페이스 박스) → goal=(x, y, z+margin)
          → SetTargetPosePositionOnly(RBDL IK) → quintic 궤적 → CTC → CST 토크
          → EtherCAT → Indy7      (v1은 현재 자세 유지; top-down R은 Phase 5)
```

### 1.3 좌표 변환의 소재 (자주 헷갈리는 지점)

**변환은 전부 ros2_ws 파이프라인에서 한다** (`pick_target_base_node` + 정적 TF).
RAON-RT는 이미 base_link로 변환된 좌표를 소비만 한다 — RAON-RT의 rMc/VS 변환 기계는 안 쓴다.
실제 남은 일은 코드 이동이 아니라:

1. **placeholder TF 정본화** — 현재 `base_link→camera_link`는 가짜값. Phase 5 손-눈 캘리브
   (`App/CalibUtils`의 AX=XB solver)가 산출한 rMc를 이 정적 TF에 넣는다.
2. rMc의 "r"은 RAON-RT FK의 로봇 베이스 → 넣는 순간 파이프라인 base_link = RBDL 모델 베이스로
   구성상 자동 정합. (rMc의 "c"는 color 광학 프레임 — `camera_link`로 환산 시
   realsense 내부 TF를 역곱해야 하는 축 관례 함정 있음.)
3. `pick_target_base_node`의 reachability 게이트를 실제 워크스페이스로 재조정.

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
| D10 | 3+1 코어 격리 = **Phase 2~3 경계에 적용** | RT-POSIX가 전 태스크를 CPU0에 기본 pin → isolcpus만으론 무효, 앱 pin 코드와 한 세트. Phase 3 런에서 격리 전/후 A/B 실측 |
| D11 | 캘리브 이미지 = **파이프라인 토픽에서 캡처** | librealsense 직접 열기 금지(충돌). intrinsics도 camera_info에서 → `save_camera_params` 불필요 |
| D12 | 캘리브 체인 = **ViSP 완전 배제, 태그 pose는 OpenCV로** (2026-07-26) | `visp-compute-apriltag-poses`는 x86-64 바이너리+소스 미포함+원저자 홈 rpath → 어디서도 실행 불가. `cv2.aruco`(APRILTAG_36h11)+`solvePnP`로 동일 YAML 산출. intrinsic은 공장값(camera_info) 1순위 — 파이프라인 3D와 같은 카메라 모델이어야 rMc 정합(불일치가 rMc에 흡수되는 계통오차 방지). 원저자 camera.xml은 640×480이라 어차피 재사용 불가(우린 848×480). 잔차 불량 시에만 `cv2.calibrateCamera` 재캘리브로 escalation |

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
`f7c27fc` **Gate 2b** (ROS2 pick bridge + 대화형 흐름 + 합성테스트).

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
| 3 서보-오프 런 | 수 분 무단절 사이클 + 조인트 판독 + 파이프라인 동시 (in-app 'p'/'v' 흐름 실증 포함) | ⬜ 다음 작업 |
| 4 서보온 | grav-comp → 'a' 위치추종 | ⬜ |
| 5 캘리브+데모 | rPc→TF 정본화 → top-down R 확정 → 'v' 접근 데모 | ⬜ |

## 6. 코드 변경 요약

### RAON-RT (kv260-merge)

| 파일 | 변경 |
|---|---|
| `App/Indy7/Indy7Ctrl.{h,cpp}` | VisualServo 완전 제거(멤버·TASK5·구 'v'키·RT루프 주입블록·proc), `SetAsDCRef(slave0)` 가드콜 추가, calib 출력 경로 `App/CalibUtils/kv260/` · **Gate 2b**: 브릿지 수명주기(Init 비치명 실패 허용/DeInit), 'p'/숫자/'v' 키 핸들러(원자플래그만), RT 루프 goal 소비 SM('n'키 검증 시퀀스 재사용: grav-comp→위치전용 IK→quintic 3 s→IK6dof+CTC, ISO/RECT 상호배제·이동중 재트리거 금지) |
| `App/Indy7/CalibCapture.{h,cpp}` | **ViSP-free 재작성** — Eigen `AngleAxisd`로 theta-u 변환, `vpPoseVector::saveYAML` 포맷 호환(→ `eye_to_hand_calib.py` 무수정 소비) |
| `App/Indy7/FullDynControllerRT.cpp` | x86 SSE 헤더 `__SSE__` 가드 (aarch64 빌드 차단 해소) |
| `App/Indy7/Makefile` | ViSP/librealsense/OpenCV/PCL 제거, `RBDL_DIR` override 추가 · **Gate 2b**: humble+my_interfaces 배선(E5의 include **화이트리스트** 방식), rpath 내장(ROS env source 불필요), `make gate2b_test` 타깃 |
| `App/Indy7/ROS2PickBridge.{h,cpp}` | **신규(Gate 2b)** — `/pick_target_base`·`/detections` 구독 + `AsyncParametersClient`로 `desired_class` LIVE 설정. 스레딩 계약: ROS2 I/O·메뉴·통계게이트는 브릿지 자체 non-RT 스레드(spin+worker), RT는 wait-free 원자 API+SPSC goal 슬롯만. `SignalHandlerOptions::None`(앱 SIGINT 보존). 게이트: N=15, 축별 std<8 mm, 워크스페이스 박스(Phase 5 전 placeholder — 하네스는 `SetWorkspaceBox`로 확장), z마진 0.15 m |
| `tools/gate2b_bridge_test.cpp` | 브릿지 단독 하네스(EtherCAT/로봇 불필요) — stdin 명령 p/숫자/v/q, RT 소비자 대역 poller |
| `tools/test_gate2b_bridge.py` | 합성 검증 드라이버 — 실브릿지+실 pick_logic 노드 vs 합성 `/detections`·`/pick_target_base` 피더. C1 메뉴 / C2 param ack / C3 `ros2 param get` 실증 / C4 lock / C5 게이트 통과 goal(z=0.27). E1~E3 방어 패턴 이식 |
| `App/Indy7/INDY7.cfg` | **전 축 `AUTO_SERVO_ON=0`**, `ENABLE_CONTROLLER_AT_STARTUP=0`, 5태스크(VS 태스크 제거), 경로 로컬화, DC 실험용 주석 템플릿 |
| `EMasterApp/Device/EcatSlaveBase.{h,cpp}` | `RegisterPDOEntry` **UINT32→INT64** (내부 `<0` 체크도 unsigned라 이중 사망 상태였음) |
| `EMasterApp/Device/Slave{CIA402Base,NRMKEndTool}.cpp` | **죽어있던 PDO 실패 가드 27개 소생** — signed 임시변수 캡처(대입식이 unsigned면 반환형만 고쳐도 무효) |
| `CRobot/AxisCIA402.h` | `GetEcatSlave()` 접근자 (DC ref 지정용) |
| `tools/gate0_rtposix_test.c` | 1 ms 주기 지터 스모크(RT/NRT 폴백, warm-up 스킵) |
| `tools/patches/rtposix-aarch64-stackmin.patch` | RT-POSIX 스택 버그 패치 보존본 |

### ros2_ws (main)

| 파일 | 변경 |
|---|---|
| `src/pick_logic_pkg/.../pick_logic.py` | **v2**: L2 스코어 선택(conf/중앙/크기 가중) + `desired_class` LIVE 파라미터('':자동) + L3 안정성 3프레임·히스테리시스·5프레임 생존 + L4 근접 person 래치. 신규 reject 사유 3종. 게이트·metrics·yield CSV 계약 불변 |
| `config/pick_logic.yaml` | v2 파라미터 블록(주석 포함) |
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

**개발환경 함정** (재발 방지 규칙):

| # | 함정 | 규칙 |
|---|---|---|
| E1 | `ros2 run` 래퍼에 SIGTERM → 노드 고아화. **유휴 rclpy spin은 SIGTERM을 삼킴**(EINTR 재대기, 콜백 없으면 python 핸들러 실행 기회 없음) | 하네스는 `start_new_session` + 그룹 SIGTERM 후 **무조건 그룹 SIGKILL** 마무리. (launch 환경은 트래픽이 executor를 깨워 무관) |
| E2 | SIGKILL된 노드는 미등록해제 → **ros2 daemon 그래프 캐시 오염** — half-dead 노드가 graph-count 대기를 속이고 `param set`이 죽은 노드로 감 | 준비판정은 **기능적 핸드셰이크**(빈 프레임→응답 수신)로. preflight에서 `ros2 daemon stop` + param set 재시도 |
| E3 | `pgrep/pkill -f`는 패턴이 **자기 셸 명령줄에 있으면 자살** | 킬 전 `/proc/PID/comm` 검사(python3/노드명만), 확인 명령은 별도 호출로 분리 |
| E4 | pick_logic 실행파일명은 `pick_logic` (노드명 `pick_logic_node`와 다름) | — |
| E5 | **Humble include의 CycloneDDS 지뢰**: `include/dds/features.h`·`include/idl/{string.h,endian.h}`가 glibc 표준 헤더를 섀도잉 — 저자 Indy7_ROS2 Makefile의 `find -maxdepth 1` -I 샷건이 이 디렉토리들을 포함해 libstdc++ 컴파일이 `__throw_length_error` 류로 붕괴(저자의 `-include cstdio/cstring/cstdlib`는 이것의 밴드에이드) | -I는 실제 include하는 패키지만 **화이트리스트** (`ROS_PKGS` 변수) — 섀도 디렉토리 원천 배제, 누락 시 명시적 에러로 드러남 |

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

- **Phase 3**: 서보-오프 통합 런 — OP 도달·사이클 유지·손으로 팔 움직여 조인트 판독 확인
  + 파이프라인 동시 가동으로 E2E 신호 경로까지 (모션 0; in-app 'p'/메뉴/'v' 게이트까지 실증
  가능 — goal은 서보-오프라 소비돼도 모션 없음). 격리 전/후 A/B 실측도 여기서
- **Phase 4**: 서보온 grav-comp (`t`→`r`→`g`) → `'a'` 위치추종
- **Phase 5**: 손-눈 캘리브(파이프라인 토픽 캡처 + 데스크톱 OpenCV solve, D12) → TF 정본화
  → 워크스페이스 박스 실측치로 교체 → top-down R 확정 → `'v'` 접근 데모
- **백로그**: 연속 추적(closed-loop), MuJoCo sim 합류, indy_iface GUI, isolcpus 3+1
  (cmdline `isolcpus=3 nohz_full=3 rcu_nocbs=3 irqaffinity=0-2` + 앱 태스크 CPU3 pin),
  DC sync0 실험(cfg 주석 해제), RPU+SOEM 트랙

## 10. 재개 가이드 (새 세션용)

1. 이 문서 + 메모리 `raon-vs-merge-plan.md`부터. RAON-RT 코드 질문은 재탐색 말고
   `~/RAON-RT_guide_for_CLAUDE.md`의 해당 §.
2. 작업은 `~/RAON-RT-Revision` **kv260-merge 브랜치에서만**. lab 저장소 push 금지.
3. EMasterApp 수정 시 실행 전 `sudo make install` (루트에서) — /opt가 런타임 정본.
4. 로봇 전원만 켜면 `ethercat slaves`로 7슬레이브 즉시 확인 가능(일반유저 CLI 가능).
5. sudo가 필요한 단계는 명령을 준비해 사용자에게 요청(비대화형 sudo 불가).
