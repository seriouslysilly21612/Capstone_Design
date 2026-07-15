# inst_claude.md — 새 세션 온보딩 / 인수인계 문서

> **이 문서의 목적**: 새 Claude 세션이 이 파일 하나만 읽고 **마지막 지점에서 바로 이어서 작업**할 수 있게 한다. 여기서 각 주제의 정본 문서로 라우팅하므로, 추가 탐색 없이 시작 가능.
> 최종 갱신: 2026-07-09

---

## 0. 30초 요약 (지금 상태)

- **프로젝트**: Kria KV260 기반 **pick&place 시스템** (RealSense → 비전 검출 → 3D 위치 → 로봇 궤적). 타깃 로봇 = Neuromeka Indy7.
- **동시 진행 중인 트랙 2개**:
  1. **비전(YOLO 교체)** — SSD stand-in → YOLOv3-tiny **6-class**(peach 드롭). **재학습·양자화·컴파일·보드 배치·Gate 5(실물 top-down) 통과 = 6종 확정**. D14 apple 실물 재학습으로 **실물 apple 0.5→0.88** 해결 → **모델 교체(Gate 2~5) 완성**. 남은 건 Gate 6/7. ← 이 문서의 §2 (최신은 §2.0)
  2. **로봇 제어 (RT 커널 패치 → EtherCAT → RPU)** — 3단계. **지금 활성 = RT-PREEMPT 커널 패치.** 그 다음 APU IgH EtherCAT, 이후 RPU FreeRTOS+SOEM. ← 이 문서의 §3
- **현재 물리 상태**: 사용자가 **RT 커널 패치 진행 중**. 비전 config는 YOLO로 전환된 상태(검증 미완). DPU·카메라 정리됨.
- **먼저 물어볼 것**: "어느 트랙을 이어서 할까요?" (비전 재검증/재렌더 vs 로봇 제어=RT커널/EtherCAT/RPU).

---

## 1. 문서 지도 (어디에 무엇이 있나)

새 세션은 **작업할 트랙의 정본 문서부터** 읽어라. 이 지도로 충분하니 무작정 grep 하지 말 것.

| 주제 | 정본 문서 | 성격 |
|---|---|---|
| 시스템 전체 구조·규칙 | `CLAUDE.md` | 노드/토픽/제약/소통규칙 (항상 우선) |
| 파이프라인 상세(노드별 파라미터·기법) | `workflow.md` | 변경 전 필독 |
| 시간순 히스토리(결정·측정·root-cause) | `progress.md` | 과거 경위 추적용 |
| **비전 YOLO — 명령어·게이트 레퍼런스** | `yolov3_tiny_execution_plan.md` | Phase 0~7, Gate 0~7, **맨 아래 "작업 일시중단 지점"이 재개점** |
| **비전 YOLO — 논리 서사(왜 이 선택인가)** | `yolo_v3_process.md` | 맥락 복구용 |
| 3D 역투영 원리 | `reprojection.md` | 단일점 reverse projection |
| **RPU/EtherCAT — 새 세션 진입점** | `rpu_guide_for_claude.md` | **RPU 작업이면 여기부터** (함정 지도 포함) |
| RPU 실행 계획(Gate 1~5) | `rpu_freertos_soem_execution_plan.md` | 명령어·게이트 |
| RPU 논리 전개 | `rpu_plan.md` | 왜 이 선택인가 |
| 지속 메모리 | `~/.claude/projects/-home-ubuntu/memory/MEMORY.md` | `kria-rt-preempt-project`, `yolov3-vision-swap-resume` |

---

## 2. 비전 트랙 (YOLOv3-tiny → 6-class) — 이 세션의 작업

### 2.0 ★ 최신 상태 (2026-07-09) — 아래 2.1~2.4보다 이게 우선
- **7-class → 6-class**(peach 드롭, D13) 재학습·양자화·컴파일·보드 배치 완료. **Gate 5(실물 top-down) 통과**: mustard 0.02→0.814, tennis 0.677, orange 0.850, banana 0.777 — **5종 확정**. apple만 0.489~0.549(0.50 경계, color 갭).
- **apple 타깃 재학습(D14) → Gate 5 통과 🎉 (2026-07-10)**: 배포 카메라로 apple만 실촬영(`real_apple_yolo` train-only) + `hsv_v`↑ 재학습 → 실물 top-down **apple 0.5→0.88**(나머지 5종 0.81~0.88 전부 개선/유지). **모델 교체(Gate 2~5) 완성.** `apple:0.40` threshold는 0.50 default로 원복함. (재학습이 데스크톱 GPU 드라이버 hang으로 2회 중단됐던 건 §2.8 — `nvidia-driver-580` proprietary 롤백으로 해결.) **남은 것: Gate 6(풀 파이프라인)·Gate 7.**
- **정본·재개 순서**: `yolov3_tiny_execution_plan.md` 맨 아래 **"D13 결과 / D14"** 섹션. 재개 = 새 mAP(apple↑?) → `12` → `13` → scp → 보드 Gate5 → apple 강하면 `apple:0.40`→**0.50 원복** → Gate6/7.
- 신규 스크립트: 보드 `~/vitis_ai_work/scripts/autolabel_single_object.py`(단일물체 auto-label), `capture_color_frames.py --manual`(Enter=1장).
- ⚠️ 아래 **2.1~2.4는 7-class 시절 기록(참고용)** — class id·Gate5 실패표는 구버전. 현재 class = `{0 apple,1 orange,2 banana,3 tennis_ball,4 mustard_bottle,5 person}`.

### 2.1 무엇을 왜 했나 (핵심만; 논리 전체는 `yolo_v3_process.md`)
- 모델 = **YOLOv3-tiny 7-class** (`0 apple,1 peach,2 orange,3 banana,4 tennis_ball,5 mustard_bottle,6 person`). 이유: DPU 지원 op만으로 구성 → 단일 DPU subgraph 가능.
- 학습 도구 = **yolov5 v7.0 repo** (구조는 YOLOv3-tiny, PyTorch라 vai_q_pytorch 직결). **activation은 SiLU→Hardswish 교체**(D11, SiLU가 DPU 미지원).
- 데이터 = **synthetic(YCB mesh)+COCO+OpenImages+BOP ycbv**. **실환경 촬영 이미지는 학습에 미사용**(D2, 검증만 허용).
- 전처리 = **letterbox 416**(D7, train/calib/worker 통일).
- 배포 기하 = 회색 optical table, **top-down 0.8m 수직**(D10).

### 2.2 완료된 것 (재작업 불필요)
- **Gate 2** 학습 PASS: hswish run `pickplace_v3tiny_hswish`, val mAP@0.5 **all 0.766** (apple .625/peach .955/orange .734/banana .738/tennis .978/mustard .823/person .509).
- **Gate 3** 양자화 PASS: cosine head0 0.9923 / head1 0.9849, VAIQ_WARN 0건.
- **Gate 4** 컴파일 PASS: DPU subgraph 1개. `xdputil`로 fingerprint·fixpos 검증됨.
- **보드 배치 완료**: `~/vitis_ai_work/models/yolov3_tiny_7class.xmodel` + `decode_meta.json` (md5 `e2ca87c2466f715e9ecc00c43b599cc4`).
- **라이브 파이프라인 기동 성공**: dpu_ms ~17-23ms. apple/orange/banana 실물 검출 OK, 오검출 없음.

### 2.3 Gate 5 결과 — 진짜 top-down 재검증 완료 (2026-07-08): 도메인 갭 확정
실제 D10 top-down(회색 타공 optical table 수직 하방)에서 재검증한 최종 결과. **형상 탓이 아니라 진짜 sim-to-real 도메인 갭**:
| 물체 | top-down 실측 | 판정 |
|---|---|---|
| banana / orange | 0.61~0.67 / 0.88 | ✅ 정상 |
| **tennis_ball** | **0.491** (0.5 바로 아래) | ⚠️ threshold 0.4로 즉시 해결 (top-down이 도움됨) |
| **apple(실제 빨강)** | 0.26 | ❌ 약함 + peach에 밀림 |
| **peach** | apple로 0.66 / peach로 ~0 | ❌ 사과로 오분류 |
| **mustard** | **0.02** | ❌ 완전 실패 (누운 top-down + Morehouse 스퀴즈병 ≠ YCB006 경질병) |
- peach·mustard는 **YCB mesh ≠ 실물 외형** → 같은 mesh 재렌더로는 못 닫음. 그래서 **D2 재검토 결정(D12)**.

### 2.4 재개 계획 (2026-07-09~) — D12: 소량 real 학습데이터 추가 (사용자 결정 2026-07-08)
**방향 전환**: synthetic 재렌더가 아니라 **real 촬영 이미지를 학습(weight update)에도 사용**. 기존 synthetic+public 유지(banana/orange 등 OK), real은 실패 class(peach/mustard/apple) 보강.
1. **real 학습셋 수집**: 이 테이블에서 6종 위치/각도/자세 다양(특히 **mustard 눕힘+세움**, 프레임 가장자리) 수십~수백 장. 사용자 물체 배치 + `capture_color_frames.py`(카메라만 기동, RT 커널 2코어 부담 회피).
2. **라벨링**(D2가 피하던 수작업): 도구/워크플로 제안 필요 — 잘 되는 class는 현 모델 auto-label 후 교정, 실패 class는 수동. (labelImg/CVAT/Roboflow 후보.)
3. **병합 + fine-tune**: 데스크톱에서 기존 데이터셋에 real 추가 → hswish cfg 재학습(`11_train.sh`) → 재양자화(`12`) → 재컴파일(`13`) → 보드 재배치 → Gate 5 재검증.
4. **quick win(즉시)**: tennis_ball threshold 0.5→0.4 (image-test 스크립트 + worker CLASS_THRESHOLDS). 0.491이라 바로 잡힘.
- 상세: `yolov3_tiny_execution_plan.md` 맨 아래 "▶ 다음 작업" 섹션. 보존 자산: `test_images/topdown/`(실물 top-down 5장).

### 2.5 보드에서 재검증할 때 쓰는 명령 (사용자 명령 없이 Claude가 직접 실행)
```bash
# (1) 파이프라인 기동 (background)
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py > /tmp/pl.log 2>&1 &
# "RealSense Node Is Up" 뜰 때까지 대기 후:
# (2) 프레임 캡처 (카메라만 필요)
python3 ~/vitis_ai_work/scripts/capture_color_frames.py --count 3 --out ~/vitis_ai_work/test_images/runN
# (3) 파이프라인 정지 → DPU 해제 (isolated 테스트 전 필수: DPU는 1 프로세스만 점유)
pkill -f "pick_place_vitis_ai.launch"; pkill -f realsense2_camera_node; pkill -f vitis_ai_worker_yolo; sleep 3
# (4) 격리 테스트 (물체별 confidence/bbox + overlay)
python3 ~/vitis_ai_work/scripts/yolov3_tiny_image_test.py \
  --model ~/vitis_ai_work/models/yolov3_tiny_7class.xmodel \
  --meta  ~/vitis_ai_work/models/decode_meta.json \
  --image ~/vitis_ai_work/test_images/runN/obj_00.jpg --output /tmp/ov.jpg
# 저-threshold 분포 진단: /tmp/.../scratchpad/diag_lowthr.py (재작성 필요시 계획서 참고)
```
**주의**: DPU는 한 번에 한 프로세스만. 파이프라인(worker)이 떠 있으면 `yolov3_tiny_image_test.py`가 충돌한다. 캡처는 카메라 토픽만 쓰므로 파이프라인과 공존 가능하지만, isolated 추론 전엔 반드시 파이프라인 정지.

### 2.6 config 현재 상태 (중요)
`~/ros2_ws/src/system_bringup_pkg/config/` (install→build→src 전부 symlink, **수정 즉시 반영, rebuild 불필요**):
- `vitis_ai_detector.yaml`: model_path→**yolov3_tiny_7class**, worker→**vitis_ai_worker_yolo.py**, **send_resized_input: false**, metrics_duration_sec **0.0**(int→double 버그 수정됨).
- `pick_logic.yaml`: allowed_classes→**6 pickable**(person은 검출은 되나 pick 대상 제외=safety).
- **SSD stand-in 복구**: `vitis_ai_detector.yaml` 상단 주석의 3줄(SSD model_path/worker/resized=true)로 되돌리면 됨.

### 2.7 학습·양자화·컴파일은 어디서 (데스크톱)
- **보드 아님**. 데스크톱 `jaehyeon@jaehyeon-Raimlab`(RTX 4060), 경로 `~/capstone_training/`.
- 스크립트 원본은 **보드 `~/ros2_ws/yolo_v3_tiny_training/`**(구 `training/`에서 개명)에 있고 데스크톱이 rsync로 당겨 씀:
  `rsync -av ubuntu@192.168.120.132:~/ros2_ws/yolo_v3_tiny_training/ ~/capstone_training/training/`
- 학습 = native venv(`~/capstone_training/venv_train`, torch cu121). 양자화/컴파일 = CPU docker `xilinx/vitis-ai-cpu:2.5.0.1260`.
- 게이트 순서·정확한 명령: `yolov3_tiny_execution_plan.md`. 스크립트 실행 순서: `training/README.md` (01~13, 12a는 학습 전 Gate0 inspect).

### 2.8 지뢰밭 (이미 밟아서 해결한 것 — 반복 금지)
- yolov5 기본 activation은 **SiLU(DPU 미지원)** → cfg에 `activation: nn.Hardswish()`. **학습 전 `12a_inspect_docker.sh`(Gate 0)로 DPU 매핑 선검증 필수.**
- VAI 2.5 `hardswish.py` 배포 버그 2건(FixNeuronWithBackward 미정의 / fake_quantize_per_tensor 인자 누락) → 12·12a wrapper가 docker 안에서 `sed`로 자동 패치.
- `cosine 수치보다 VAIQ_WARN/unknown-op 경고를 먼저 볼 것`.
- `metrics_duration_sec`는 double(`0.0`)이어야 함(정수면 detector 死).
- `sudo bash 12_*.sh` 금지(내부에서만 sudo). 스크립트는 sudo 없이 실행.
- **데스크톱 GPU 드라이버(2026-07-09)**: 자동 업데이트로 `nvidia-driver-595-open`(open 커널모듈+GSP)이 깔리면 CUDA 학습 중 **로그 없는 hard-freeze**(epoch 0에서 죽음, 온도 정상, OOM/Xid/MCE 전무). → **proprietary `nvidia-driver-580`로 롤백**(open은 GSP 강제라 회피 불가) + `apt-mark hold`로 자동 업데이트 차단. 확인=`cat /proc/driver/nvidia/version`에 'Open Kernel Module' 문구 없으면 정상. 재발 시 하드웨어(memtest/PSU/PCIe).

---

## 3. 로봇 제어 트랙 — RT 커널 패치 → EtherCAT → RPU (별도 관리)

이 세션에서 다루지 않았지만 **동시 진행 중**. 로봇(Indy7) 제어 스택을 3단계로 쌓는다. 정본: 메모리 `kria-rt-preempt-project`, 계획 `rpu_freertos_soem_execution_plan.md`, 논리 `rpu_plan.md`. **이 트랙 작업이면 반드시 `rpu_guide_for_claude.md`부터 읽어라** (검증된 DT binding, UART0/CMA 함정, macb unbind 클럭 게이팅 등 함정 지도 포함).

### Phase A — RT-PREEMPT 커널 패치 (APU) ★ 지금 활성
- **목표**: KV260 APU(A53) 커널을 RT-PREEMPT로 교체 (실시간 제어 지연 보장).
- **현 커널**: `linux-xilinx-zynqmp` 5.15.0-1070.74 (업스트림 베이스 5.15.199), **PREEMPT_VOLUNTARY / HZ=250** → RT로 바꿔야 함.
- **방법**: x86 PC에서 **aarch64 크로스컴파일**. 소스 = Ubuntu Xilinx 커널(linux-xilinx-zynqmp) + kernel.org RT 패치(5.15.199에 가장 가까운 `patch-5.15.x-rt`).
- **Kria 특수사항**: 부팅은 **U-Boot + flash-kernel(`/boot/firmware/image.fit`), GRUB 없음**. **CMA 유지**(FPGA/PL 사용 중 — 랩 가이드의 CMA 비활성화는 적용 안 함). cmdline에 `cma=1000M`, `clk_ignore_unused`.
- **참고 가이드**: 랩(RAIMLAB, 명지대) MAN-20241113-LX02H0001 (x86 NUC용 RT-PREEMPT + IgH EtherCAT). 큰 흐름만 따르고 Kria 차이(GRUB→flash-kernel, CMA 유지) 반영.

### Phase B — IgH EtherCAT Master (APU)
- APU에서 **IgH EtherCAT Master 1.6.2** 구동. NIC 드라이버는 igc가 아니라 **Cadence GEM(macb)** → generic 드라이버(`--enable-generic`) 필요.
- **GEM3=ff0e0000 (eth0, 유일한 활성 GEM)**. Indy7는 **STEP 우회·드라이브 직결**(CiA402+CSP) 확정. 사용자가 **PC에서 EtherCAT으로 Indy7 제어 성공한 코드 보유** → 이식 작업. 테스트 드라이브 **LS Mecapion L7N**. 제어주기 1kHz 추정.

### Phase C — RPU FreeRTOS + SOEM (이후)
- **FreeRTOS + SOEM**(IgH는 APU 전용). Vitis **2022.1**(x86)로 R5F-0 split 모드 빌드, remoteproc 로드(BOOT.BIN 아님, systemd 자동화). GEM3를 RPU로 이관, SOEM osal/oshw를 emacps 기반 직접 포팅, APU↔RPU는 OpenAMP RPMsg.
- DT에 R5 노드 없음 → configfs DT overlay 필요(smartcam이 이미 이 방식). 그리퍼 없음 → end-to-end 목표는 말단부가 물체 위 정지까지.

### 트랙 간·단계 간 충돌 주의
- **GEM3(eth0)가 유일한 활성 이더넷 포트**. EtherCAT 전용/RPU 이관 시 보드 원격접속(**Tailscale over eth0**)이 끊긴다 → 대체 네트워크(WiFi/USB Ethernet/UART) 필수.
- **RPU(Phase C) 실제 착수는 APU IgH(Phase B) 완료 후** (GEM3 공유 충돌). 단 Vitis 설치+RPU Phase1은 병행 가능.
- **비전 트랙과는 리소스 충돌 없음** (DPU/카메라 vs 커널/GEM은 별개). 단 같은 보드라 재부팅·커널 교체 시 비전 파이프라인도 함께 내려감.

---

## 4. 보드·환경 필수 사실

- **보드(이 머신)**: Kria KV260, `ubuntu@192.168.120.132`, Ubuntu 22.04.5 + ROS2 Humble. 작업 루트 `~/ros2_ws/`.
- **DPU**: DPUCZDX8G_ISA1_B3136, fingerprint `0x101000016010406`, kv260-smartcam overlay(systemd 자동로드), VART/VAI 2.5.0. arch: `~/vitis_ai_work/arch/arch_b3136.json`.
- **카메라**: RealSense D435i, color+depth 848×480×30, `align_depth` OFF. color 토픽 `/camera/camera/color/image_raw`.
- **파이프라인 실행**: `ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py` (카메라+detector+pick_logic+3D+TF). 3D는 단일점 reverse projection(`camera_depth_optical_frame`), base TF는 placeholder.
- **detector↔worker 경계 = model-agnostic JSON contract**. 모델 교체는 worker 전처리/decode만 건드림. `vitis_ai_detector_pkg`는 editable-install(src가 live), config YAML은 symlink(rebuild 불필요), `target_3d_pkg`만 편집 후 `colcon build --packages-select target_3d_pkg --symlink-install`.
- **데스크톱**: `jaehyeon@jaehyeon-Raimlab`, RTX 4060 8GB, Ubuntu 22.04.5. 학습/양자화/컴파일 담당. 보드로 xmodel 전송은 `scp ... ubuntu@192.168.120.132:~/vitis_ai_work/models/`.

---

## 5. 새 세션 시작 절차 (권장)

1. 이 문서(`inst_claude.md`)를 읽는다.
2. 사용자에게 **어느 트랙**인지 확인 (비전 / RPU).
3. 해당 트랙 정본 문서를 연다:
   - 비전 → `yolov3_tiny_execution_plan.md` 맨 아래 "작업 일시중단 지점" + 필요시 `yolo_v3_process.md`.
   - RPU → `rpu_guide_for_claude.md`.
4. 비전 재개면: §2.5 명령으로 **진짜 top-down 형상 재검증부터**. 상태 변경(파이프라인 기동/정지, DPU 점유)은 사용자 명령 없이 Claude가 직접 하되, 물리 배치(물체 놓기·카메라 형상)는 사용자에게 요청.
5. 소통: **한국어 답변, 기술용어는 영어 유지.** 사용자는 학부생 수준·절차 지향 → 명령어 중심 단계별 안내. 파괴적/외부영향 작업만 사전 확인, 가역적 작업은 진행.

---

## 6. 미해결 TODO 스냅샷

**비전**: [D13 6-class Gate5 통과 ✔] → [D14 apple 실물 재학습 ✔] → [보드 Gate5: 실물 apple 0.5→0.88, 6종 확정 ✔] → [apple threshold 0.50 원복 ✔] → [Gate 6 실측 ✔ 15Hz·E2E137ms·CPU79%/4코어] → [**target_3d CPU 최적화 ★다음**(depth 30→15fps 등)] → [Gate 7 live].
**로봇 제어**: **[Phase A: RT-PREEMPT 커널 패치 ★진행 중]** → [Phase B: APU IgH EtherCAT master, Indy7 제어 코드 이식] → [Phase C: RPU FreeRTOS+SOEM]. 상세 게이트=`rpu_freertos_soem_execution_plan.md`, 진입=`rpu_guide_for_claude.md`.
**공통 시스템**: base_link↔camera_link TF calibration(현재 placeholder), APU↔RPU bridge 미구현.
