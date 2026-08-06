# Pick & Place Perception Workflow (KV260)

Kria KV260(APU: Ubuntu 22.04 + ROS2 Humble) 위에서 동작하는 **카메라 → 비전 추론 → 3D pick 좌표** 파이프라인의 전체 문서입니다.

- **하드웨어**: Kria KV260 + Intel RealSense D435i
- **가속기**: `kv260-smartcam` 오버레이의 DPU (`DPUCZDX8G_ISA1_B3136`)
- **현재 모델**: `ssd_adas_pruned_0_95` (car / bicycle / person 검출 — 최종 pick 물체용이 아닌 **검증용 stand-in**)
- **통합 실행**: `ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py`

문서 구성:
1. [노드별 파이프라인](#1-노드별-파이프라인) — 각 노드의 파일 경로 / 입력·출력 / 내부 동작 / 핵심 파라미터와 그 값의 이유
2. [성능 향상 기법 (전처리·후처리)](#2-성능-향상-기법-전처리후처리)
3. [파이프라인 아키텍처](#3-파이프라인-아키텍처)

---

## 0. 전체 개요

### 데이터 흐름

```
[RealSense D435i]
   /camera/camera/color/image_raw          (sensor_msgs/Image, BGR8 848x480)
   /camera/camera/depth/image_rect_raw      (sensor_msgs/Image, 16UC1 848x480, raw=unaligned)
   /camera/camera/{depth,color}/camera_info (intrinsics)
   /camera/camera/extrinsics/depth_to_color (realsense2_camera_msgs/Extrinsics)
        │ color image
        ▼
[vitis_ai_detector_node]  ── pipe ──▶ [vitis_ai_worker.py (DPU 추론 프로세스)]
   → /detections                          (my_interfaces/DetectionArray)
        │
        ▼
[pick_logic_node]
   → /pick_target                         (my_interfaces/PickTarget)   2D 필터 통과 1개
        │                + raw depth + intrinsics + extrinsics
        ▼
[pick_target_3d_node]   (single-point reverse projection)
   → /pick_target_3d                      (my_interfaces/PickTarget3D, frame=camera_depth_optical_frame)
        │   + TF (base_link → camera_link → camera_depth_optical_frame)
        ▼
[pick_target_base_node]
   → /pick_target_base                    (my_interfaces/PickTarget3D, frame=base_link)
```

### 노드 ↔ 파일 ↔ 설정 매핑

| 노드 | 패키지 / 파일 | 설정 파일 |
|---|---|---|
| `camera` (realsense2_camera) | third-party | `src/system_bringup_pkg/config/realsense_pick_place.yaml` |
| `vitis_ai_detector_node` | `src/vitis_ai_detector_pkg/vitis_ai_detector_pkg/vitis_ai_detector_node.py` | `src/system_bringup_pkg/config/vitis_ai_detector.yaml` |
| (worker 프로세스) | `src/vitis_ai_detector_pkg/vitis_ai_detector_pkg/vitis_ai_worker.py` | (위 yaml의 `worker_*`) |
| `pick_logic_node` | `src/pick_logic_pkg/pick_logic_pkg/pick_logic.py` | `src/system_bringup_pkg/config/pick_logic.yaml` |
| `pick_target_3d_node` | `src/target_3d_pkg/target_3d_pkg/pick_target_3d_node.py` | `src/system_bringup_pkg/config/target_3d.yaml` |
| `base_to_camera_tf` | `tf2_ros/static_transform_publisher` | launch 파일 내 인자 |
| `pick_target_base_node` | `src/target_3d_pkg/target_3d_pkg/pick_target_base_node.py` | `src/system_bringup_pkg/config/target_base.yaml` |
| 통합 launch | `src/system_bringup_pkg/launch/pick_place_vitis_ai.launch.py` | — |

---

## 1. 노드별 파이프라인

### 1.1 RealSense 카메라 (`realsense2_camera`)

- **역할**: 센서에서 color/depth 스트림과 카메라 파라미터(intrinsics/extrinsics)를 ROS 토픽으로 발행.
- **출력 토픽**
  - `/camera/camera/color/image_raw` — BGR8, 848×480
  - `/camera/camera/depth/image_rect_raw` — 16UC1, 848×480, **raw(정렬 안 됨)**, scale 0.001 m/unit
  - `/camera/camera/{depth,color}/camera_info` — `k`(fx,fy,cx,cy), 왜곡계수 `d`(전부 0 = rectified)
  - `/camera/camera/extrinsics/depth_to_color` — depth↔color 회전·평행이동 (baseline ≈ 15 mm)
  - `/tf_static` — `camera_link → camera_depth_optical_frame`, `→ camera_color_optical_frame`

**핵심 파라미터** (`realsense_pick_place.yaml`)

| 파라미터 | 값 | 이유 |
|---|---|---|
| `rgb_camera.color_profile` | `848x480x30` | 1280×720이면 realsense 노드가 100% busy-spin에 빠져 약 70초 후 프레임 공급이 멈추는(전체 파이프라인 silent stall) 현상이 있었음. detector가 어차피 480×360으로 resize하므로 해상도를 낮춰도 **검출 품질 영향 없음**. 둘 다 16:9라 resize aspect 동일. USB/CPU 부하 ~2.3배 감소. |
| `depth_module.depth_profile` | `848x480x30` | depth도 동일 해상도. |
| `align_depth.enable` | **`false`** | full-frame depth→color 정렬은 realsense 노드의 단일 스레드가 A53 코어 하나를 100% 점유하게 만들어 검출 throughput을 절반으로 떨굼. 우리는 bbox 중앙 1점의 depth만 필요하므로 **정렬을 끄고 `pick_target_3d_node`에서 점 1개만 reverse projection** 함 (§3.5). |
| `enable_sync` | `false` | color/depth를 독립 처리 (정렬 안 하므로 sync 불필요). |
| `enable_gyro/accel/infra` | `false` | pick-place에 불필요한 스트림 차단으로 부하 절감. |

> 참고: realsense는 **aligned 토픽을 누가 구독할 때만** 정렬을 계산한다. 과거 "카메라 단독 25Hz" 측정이 오도했던 이유 — 단독에는 구독자가 없어 정렬이 안 돌았기 때문.

---

### 1.2 비전 추론: `vitis_ai_detector_node` (+ `vitis_ai_worker.py`)

비전 추론은 **두 프로세스**로 나뉜다: ROS 래퍼 노드와 DPU 추론 worker. 분리 이유는 §3.1 참고.

#### (A) `vitis_ai_detector_node.py` — ROS 래퍼

- **입력**: `/camera/camera/color/image_raw` (`sensor_msgs/Image`)
- **출력**: `/detections` (`my_interfaces/DetectionArray`), (옵션) `/vitis_ai_detector/overlay/compressed`
- **내부 동작**
  1. `image_callback`: 들어온 최신 프레임만 보관(경량, 즉시 반환) — §3.3 파이프라이닝
  2. `worker_loop`(전용 스레드) → `process_frame`:
     - `ros_image_to_bgr(msg)`로 ROS Image → numpy BGR 변환
     - `detect_with_worker(image)`: 480×360으로 resize → worker stdin에 `[JSON 헤더 + raw 바이트]` 전송 → worker stdout에서 detection JSON + timing 수신
     - JSON → `Detection[]` 변환, `DetectionArray`로 `/detections` 발행
     - (옵션) overlay 생성/압축 발행, (옵션) per-frame metric을 메모리에 적재(CSV)

**핵심 파라미터** (`vitis_ai_detector.yaml`)

| 파라미터 | 값 | 이유 |
|---|---|---|
| `detector_mode` | `worker` | in-process VART는 `execute_async()`에서 segfault → **별도 worker 프로세스**로 격리 (§3.1). `oneshot`은 fallback. |
| `send_resized_input` | `true` | worker로 보내기 전 480×360으로 resize → pipe payload 2,764,800 → 518,400 byte(**약 81% 감소**). 검출은 어차피 480×360 입력이라 무손실. bbox는 source 해상도 좌표로 반환. |
| `process_period_sec` | `0.0` | 스로틀 없음 = 가능한 한 빨리 다음 프레임 처리(최대 throughput). |
| `publish_empty_detections` | `true` | 검출 0개여도 매 프레임 빈 배열 발행 → 파이프라인 heartbeat 유지 + `topic hz`가 실제 처리율을 반영(측정 편의). 운영에서 트래픽을 줄이려면 `false`. |
| `publish_overlay` / `publish_compressed_overlay` | `false` / `false` | overlay는 디버그 시각화용인데 프레임당 ~70 ms(콜백의 ~40%)나 먹어 throughput을 깎음 → 운영은 off. bbox를 보려면 compressed를 잠깐 켬. |
| `worker_log_path` | `""` | worker가 진단 로그를 프레임당 ~18회 file open/write/flush 하던 것을 제거(SD/eMMC I/O). 운영 off. |
| `worker_startup_timeout_sec` / `timeout_sec` | `30.0` / `10.0` | worker는 XIR/VART 초기화에 수 초 걸려 **기동 타임아웃을 길게**(30s), 프레임 추론 타임아웃은 짧게(10s). |
| `metrics_csv_path` | `""` | 프로파일링 전용(운영 off). 경로를 주면 매 프레임 timing을 모아 종료 시 CSV 저장. |
| `model_path` | `.../ssd_adas_pruned_0_95.xmodel` | B3136용으로 컴파일된 SSD. (최종 pick 모델로 교체 예정 — 노드/worker 계약은 model-agnostic.) |

#### (B) `vitis_ai_worker.py` — DPU 추론 프로세스 (rclpy 없음)

- **입력(stdin)**: JSON 헤더(width/height/channels/data_len/source_width/source_height) + raw BGR 바이트
- **출력(stdout)**: `{"detections":[...], "timing":{pre_ms,dpu_ms,post_ms,worker_ms}, "error":null}`
- **수명주기**: 기동 시 1회만 — `xir.Graph.deserialize` → DPU subgraph → `vart.Runner.create_runner` → 출력 텐서 버퍼/priors/LUT 준비 → `{"status":"ready", "input_shape":[1,360,480,3]}` 전송. 이후 프레임마다 재사용.
- **프레임 처리** `detect()`:
  1. `preprocess_image()` — **LUT 방식**으로 BGR → int8 입력 텐서 (§2.1)
  2. `runner.execute_async([input]) → runner.wait()` — DPU 추론(~13 ms)
  3. conf/loc 역양자화 → `postprocess()` — **사전필터 + softmax + box decode + NMS** (§2.2)
  4. detection 리스트(원본 source 좌표) 반환

**핵심 상수** (`vitis_ai_worker.py` 상단)

| 상수 | 값 | 의미 |
|---|---|---|
| `MODEL_W, MODEL_H` | `480, 360` | SSD 입력 크기. |
| `MEAN_BGR` | `[104, 117, 123]` | 모델 학습 시 사용한 평균(Caffe BGR). 추론도 같게 빼야 정확도 유지. |
| `CLASS_THRESHOLDS` | `{car:0.6, bicycle:0.4, person:0.3}` | 클래스별 confidence 임계값. car는 보수적으로 높게. |
| `NMS_THRESHOLD` | `0.4` | 중복 박스 억제 IoU. |
| `priors` | 16436개 | SSD anchor box (6개 feature layer에서 생성). |

> 핵심 버그 수정: VART runner를 만든 뒤 XIR graph/subgraph 파이썬 객체가 GC되면 이후 `execute_async()`가 segfault → **`self.graph`/`self.dpu_subgraphs`를 runner 수명 동안 보관**해서 해결.

---

### 1.3 `pick_logic_node` — 2D 필터링

- **파일**: `src/pick_logic_pkg/pick_logic_pkg/pick_logic.py`
- **입력**: `/detections` (`DetectionArray`)
- **출력**: `/pick_target` (`PickTarget` = `target_valid` + Detection 필드)
- **내부 동작**: `detections_callback`에서 배열을 순회하며 `is_detection_acceptable()`로 필터 → **처음으로 통과한 1개**를 `target_valid=true`로 발행. 빈 배열이거나 전부 reject면 `target_valid=false` 발행(이유 로그).
- **필터 순서**: confidence → allowed class → bbox 크기>0 → **edge margin** → bbox 면적(min/max).

**핵심 파라미터** (`pick_logic.yaml`)

| 파라미터 | 값 | 이유 |
|---|---|---|
| `min_confidence` | `0.3` | 너무 약한 검출 제거(낮춰 두어 ADAS stand-in 테스트 편의). |
| `allowed_classes` | `[car, bicycle, person]` | 현재 모델 클래스. (최종 모델 교체 시 pick 물체 클래스로 변경.) |
| `image_width / image_height` | `848 / 480` | **카메라 color 해상도와 반드시 일치** — detector가 bbox를 source(848×480) 좌표로 주기 때문에 edge/area 필터가 이 값을 기준으로 함. |
| `edge_margin_px` | `30` | 프레임 가장자리에 걸친 박스는 잘려서 중심·depth가 부정확 → 거부. |
| `min_bbox_area_px` | `400.0` | 너무 작은 박스(노이즈) 거부. |
| `max_bbox_area_ratio` | `0.5` | 화면의 절반 넘는 비정상적으로 큰 박스 거부. |

---

### 1.4 `pick_target_3d_node` — 단일 점 reverse projection 3D

- **파일**: `src/target_3d_pkg/target_3d_pkg/pick_target_3d_node.py`
- **입력**:
  - `/pick_target` (`PickTarget`) — bbox 중앙 = color 픽셀
  - `/camera/camera/depth/image_rect_raw` (16UC1 raw depth)
  - `/camera/camera/depth/camera_info`, `/camera/camera/color/camera_info` (intrinsics)
  - `/camera/camera/extrinsics/depth_to_color` (extrinsics, transient_local QoS)
- **출력**: `/pick_target_3d` (`PickTarget3D`, `header.frame_id = camera_depth_optical_frame`)
- **내부 동작** (`pick_target_callback`):
  1. bbox 중앙 color 픽셀 `(u_c, v_c)`에 대해 `color_pixel_to_depth_pixel()`로 **대응 depth 픽셀** 탐색 (rs2_project_color_pixel_to_depth_pixel = epipolar 선분 탐색, §3.5)
  2. 찾은 depth 픽셀 주변 patch median으로 z(m) 계산
  3. depth intrinsics로 `deproject` → depth optical frame에서 `(x, y, z)` → 발행
- **degrade**: depth/intrinsics/extrinsics 미준비, 매칭 실패, 범위 밖 depth → `depth_valid=false`로 발행(경고 throttle).

**핵심 파라미터** (`target_3d.yaml`)

| 파라미터 | 값 | 이유 |
|---|---|---|
| `depth_topic` | `.../depth/image_rect_raw` | **raw(정렬 안 된) depth** — align을 껐으므로. |
| `extrinsics_topic` | `.../extrinsics/depth_to_color` | color↔depth 대응을 풀기 위한 외부 파라미터. |
| `depth_scale_16uc1` | `0.001` | 16UC1 1단위 = 1 mm (D435i). |
| `patch_radius` | `4` | 매칭 depth 픽셀 주변 9×9 median → 노이즈/구멍에 강건. |
| `min_depth_m / max_depth_m` | `0.05 / 3.5` | 유효 depth 범위 = reverse projection 탐색 선분의 깊이 범위이기도 함. |

> 출력 frame이 `camera_color_optical_frame` → `camera_depth_optical_frame`으로 바뀜(둘 다 TF로 base_link에 연결). 메시지 계약은 동일해 다운스트림 무수정.
> rs2 extrinsics 회전은 **column-major** 라 `reshape(3,3, order='F')`로 읽어야 함(검증 완료).

---

### 1.5 `base_to_camera_tf` — 정적 TF

- **노드**: `tf2_ros/static_transform_publisher` (launch 인자)
- **역할**: `base_link → camera_link` 정적 변환 발행.
- **현재 값(placeholder)**: `x=0.45, y=0.10, z=0.70, roll=pitch=yaw=0`
  - ⚠️ **측정값 아님(placeholder)** → `/pick_target_base`는 TF 파이프라인이 동작함을 증명하지만 **최종 로봇 좌표는 아님**. camera-to-base 캘리브레이션이 남은 TODO.

---

### 1.6 `pick_target_base_node` — base_link 좌표 변환

- **파일**: `src/target_3d_pkg/target_3d_pkg/pick_target_base_node.py`
- **입력**: `/pick_target_3d` + TF
- **출력**: `/pick_target_base` (`PickTarget3D`, `frame_id = base_link`)
- **내부 동작**: 입력 유효성 검사(`target_valid`, `depth_valid`, frame 존재, z 범위) → `tf_buffer.lookup_transform(base_link, 입력frame)` → `do_transform_point`로 좌표 변환 → 발행.

**핵심 파라미터** (`target_base.yaml`)

| 파라미터 | 값 | 이유 |
|---|---|---|
| `target_frame` | `base_link` | 로봇 기준 좌표. |
| `require_depth_valid` | `true` | depth 없는 타깃은 변환 안 함. |
| `min_camera_z_m / max_camera_z_m` | `0.20 / 1.50` | 작업영역(reachability) 필터 — 너무 가깝거나(0.2m 미만) 먼(1.5m 초과) 타깃 거부. |
| `transform_timeout_sec` | `0.2` | TF 조회 대기 한도. |

---

### 1.7 메시지 타입 (`my_interfaces`)

```
Detection.msg       : int32 class_id, string class_name, float32 confidence,
                      float32 center_x, center_y, width, height
DetectionArray.msg  : std_msgs/Header header, Detection[] detections
PickTarget.msg      : bool target_valid + (Detection 필드들)
PickTarget3D.msg    : std_msgs/Header header, bool target_valid, bool depth_valid,
                      (Detection 필드들), float32 x, y, z
```

---

## 2. 성능 향상 기법 (전처리·후처리)

측정으로 병목을 분해(detector 콜백을 `img/pre/dpu/post/ipc/overlay`로 계측)한 뒤, 큰 leaf부터 제거했다. **DPU 추론 자체는 ~13 ms(전체의 일부)** 였고, 진짜 병목은 **Python 전처리/후처리**였다.

### 2.1 LUT 전처리 (`preprocess_image`)

**문제**: SSD 입력 텐서를 만들 때 픽셀마다
`int8 = clip(round((픽셀 − mean) × scale), −128, 127)` 를 480×360×3 = 518,400 픽셀에 대해 float32 배열로 여러 번 패스 + 매번 임시배열 할당 → A53(메모리 대역폭 약함)에서 **~42 ms**.

**통찰**: 입력은 `uint8`라 값이 **0~255, 256가지뿐**이고, 결과는 (값, 채널)에만 의존 → 같은 계산을 수십만 번 반복 중.

**기법 (Look-Up Table)**:
- 기동 시 1회, 채널별 256칸 int8 표를 미리 계산:
  `lut[c][v] = clip(round((v − mean[c]) × scale), −128, 127)` (`scale = 2**input_fix`)
- 매 프레임은 계산 없이 표 조회 3번:
  ```python
  arr[:, :, 0] = lut[0][resized[:, :, 0]]   # B
  arr[:, :, 1] = lut[1][resized[:, :, 1]]   # G
  arr[:, :, 2] = lut[2][resized[:, :, 2]]   # R
  ```

**결과**: `pre_ms 42 → ~12 ms`. 표는 기존 수식과 동일한 round/clip으로 만들어 **출력이 bit 단위로 동일**(검출 정확도 불변, 무작위 이미지로 검증).

### 2.2 SSD 후처리 사전필터 (`postprocess`)

**문제**: 16,436개 prior 전부에 softmax + box decode + NMS를 돌림. 대부분은 background인데도 전수 처리 → `post_ms ~20 ms`.

**기법 (안전한 necessary-condition 사전필터)**: foreground 클래스 임계값을 통과할 가능성이 있는 prior만 남기고 나머지는 버린 뒤, **살아남은 소수에만** softmax/decode/NMS:
- softmax 적용 케이스(conf=logits): `prob_c ≥ t_c` 이려면 필연적으로
  `z_c − z_background ≥ logit(t_c)` (sigmoid 상계에서 유도). 이 조건으로 1차 필터(softmax 불필요, 뺄셈·비교만). FP 경계값을 안 놓치게 ε 마진.
- 이미 확률인 케이스: `foreground prob ≥ min(threshold)` 로 필터.
- `loc` 역양자화·`decode_boxes`도 **후보에 대해서만** 수행.

**결과**: `post_ms 20 → ~7 ms`. 필터가 "필요조건"이라 통과 집합은 전수 처리의 상위집합 → 이후 정확한 클래스 임계값으로 잘라내므로 **최종 detection이 전수 처리와 동일**(무작위 데이터 30회로 검증).

### 2.3 기타 부하 절감

| 기법 | 효과 |
|---|---|
| 입력 resize 후 전송(`send_resized_input`) | pipe payload **−81%** (2.76 MB → 0.52 MB/frame) |
| overlay off | 콜백 **−~70 ms/frame** (디버그 전용) |
| worker file-log off | 프레임당 ~18회 file I/O 제거 |
| 카메라 해상도 1280×720 → 848×480 | realsense 부하 ~2.3배↓ + busy-spin stall 해소 |
| full-frame alignment 제거 (§3.5) | realsense 코어 ~65% 회수, 검출 **~13 → ~17 Hz** |

**누적 결과**: 검출 `processing_ms 88.7 → 45.8 ms`, throughput `8.6 → 13 Hz`(compute), alignment 제거까지 더해 **~17 Hz**, end-to-end latency `349 → ~261 ms`. 모든 최적화에서 **검출 정확도 불변**.

---

## 3. 파이프라인 아키텍처

### 3.1 프로세스 분리 — detector 노드 ↔ worker 프로세스

- **이유**: VART `execute_async()`가 rclpy 노드 in-process에서 segfault/bus error. 근본 원인은 XIR graph/subgraph 객체가 GC되어 runner가 무효 상태를 참조한 것. 이를 **별도 프로세스 + 객체 수명 보장**으로 안정화.
- **구조**: ROS 래퍼(`vitis_ai_detector_node`)는 이미지 구독/전처리/발행만, **DPU는 `vitis_ai_worker.py`라는 별도 프로세스**가 담당. 둘은 stdin/stdout **pipe IPC**로 통신.
- **계약(JSON)**: `Detection` 호환 항목 + timing. **model-agnostic** — 모델을 바꿔도 worker 내부(전처리 상수·decode)만 바뀌고, 노드/파이프라인/3D/다운스트림은 그대로.

### 3.2 worker 수명주기

```
기동: xmodel deserialize → DPU subgraph → VART runner → (priors, LUT, 출력버퍼) 준비
      → "ready"(+ input_shape) 핸드셰이크
프레임 루프: stdin에서 이미지 수신 → detect() → stdout으로 detection+timing
이상 시: 노드가 worker death 감지하면 재시작(restart_worker), 기동 타임아웃 30s
```
- xmodel/runner를 **1회만** 로드하고 출력 텐서 버퍼도 재사용 → per-frame 오버헤드 최소화.

### 3.3 콜백 파이프라이닝 (스레드 분리)

- **문제**: 기존엔 `image_callback`이 worker 응답까지 블록 → executor가 묶이고 콜백 간 idle 발생.
- **구조**:
  - `image_callback`: **최신 프레임만 저장**(락+이벤트), 즉시 반환 → executor가 카메라를 안 굶김.
  - `worker_loop`(전용 스레드) → `process_frame`: 최신 프레임을 집어 **연속 처리**(latest-frame-only, 묵은 프레임 버림).
- **GIL 안전성**: 무거운 DPU는 **별도 프로세스**(GIL 무관), 노드 스레드는 주로 pipe I/O 대기(GIL 해제) + numpy/cv2(GIL 해제) → 스레드가 효과적.
- **결과**: throughput은 그대로(시스템이 **camera-limited**임이 드러남), **latency는 ~31 ms 감소**. 깨끗한 구조 분리도 확보.

### 3.4 QoS / latest-frame-only

| 인터페이스 | QoS | 이유 |
|---|---|---|
| 카메라 color/depth 구독 | `KEEP_LAST, depth=1, BEST_EFFORT` | 항상 **최신 1장**만 — 묵은 프레임 누적 방지, latency 최소. |
| extrinsics 구독 | `RELIABLE, TRANSIENT_LOCAL` | 한 번만 latched 발행되므로 늦게 떠도 받기 위해 transient_local. |
| `/detections` 등 발행 | `RELIABLE, depth=1` | 다운스트림 누락 방지. |

### 3.5 Reverse projection (full-frame alignment 제거)

- **배경**: `target_3d`가 aligned depth를 구독하면 realsense가 매 프레임 depth 전체(≈407k 픽셀)를 color로 재투영 → 단일 스레드가 코어 100% 점유 → 카메라 12 Hz로 throttle. 그런데 우리가 쓰는 건 **bbox 중앙 1점**뿐.
- **기법 (`rs2_project_color_pixel_to_depth_pixel`)**: align을 끄고, color 픽셀 1개를 다음으로 매칭:
  1. color 픽셀을 `dmin`/`dmax`로 deproject → color→depth extrinsic 변환 → depth 영상의 **짧은 선분** 양 끝 계산
  2. 그 선분을 따라가며, 각 depth 픽셀을 다시 color로 reproject해 원래 픽셀과 가장 가까운 것을 선택
  3. 찾은 depth 픽셀의 z로 depth frame에서 `(x,y,z)` 산출
- **왜 단순 근사가 안 되나**: color↔depth는 ~15 mm baseline이라 시차(disparity)가 큼(가까운 물체에서 60~90 px). "같은 픽셀" 가정 시 엉뚱한 depth를 읽음. 선분 탐색이 이를 정확히 보정(시뮬 5점 0 px 오차, 실측 z 자로 검증).
- **결과**: realsense 코어 ~65% 회수, 검출 **~13 → ~17 Hz**, latency 감소, z 정확도 유지. 회수한 코어는 향후 RT/EtherCAT 로봇 제어 여유로 사용.

### 3.6 성능 요약 (검출 단계)

| 단계 | baseline | +LUT | +post필터 | +pipe | +reverse-proj(align off) |
|---|---|---|---|---|---|
| `pre_ms` | 42.5 | 12.7 | 12.5 | 11.8 | — |
| `post_ms` | 20.1 | 19.4 | 7.4 | 7.8 | — |
| `dpu_ms` | 12.7 | 13.0 | 12.8 | 12.5 | (하드웨어 하한) |
| `processing_ms` | 88.7 | 58.2 | 45.8 | 46.1 | ~46 |
| **publish Hz** | **8.6** | 12.2 | 13.0 | 12.7 | **~17** |
| `frame_age_ms`(latency) | 349 | 307 | 292 | 261 | ~261 |

> throughput의 최종 천장은 **카메라 공급률(realsense 노드 단일 스레드 처리)** 이다. 더 올리려면 카메라 측(노드 부하/해상도) 또는 C++ 비전 경로가 필요. 단, pick-and-place(정적 물체)엔 현 ~17 Hz로 충분.

> **⚠️ 2026-08-05 보강**: 위 문장의 "최종 천장"은 *모든 소프트웨어 상한을 걷어낸 뒤*의
> 천장이라는 뜻으로만 맞다. **지금 실제로 묶고 있는 건 카메라가 아니다** — 카메라는
> 29.4 Hz를 공급하는데 파이프라인은 15.3 Hz만 내고, 구속조건은 `process_period_sec: 0.045`
> 게이트(의도된 15 Hz 상한)다. DPU도 73% 놀고 있다. 성능을 올리는 순서와 각 단계의
> 이득·비용·위험은 **`docs/vision/throughput.md`**에 정리했다. 카메라나 DPU부터
> 손대자는 제안은 거의 항상 틀렸다.
> **08-05 후속**: 이동 물체 전환으로 게이트 0.045→0.030 재결정 + worker IPC를
> pipe→/dev/shm mmap으로 교체(`shm_frame.py`) → **21.7 Hz**, CPU 변화 0. 같은 문서 §4에 실측.

---

## 부록: 알려진 placeholder / TODO

- `base_link → camera_link` 정적 TF는 **placeholder** → camera-to-base 캘리브레이션 필요(그 전엔 `/pick_target_base`는 실제 로봇 좌표 아님).
- `ssd_adas`는 **검증용 stand-in** → B3136용 최종 pick-object(YCB/커스텀) detector로 교체 예정.
- 로봇 제어 레이어(APU→RPU(FreeRTOS)→Indy7, Ethernet/EtherCAT)는 미구현 — CPU 코어 격리 설계 필요.
