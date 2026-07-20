# 코드 정독 가이드 — detector node + YOLO worker

`vitis_ai_detector_node.py`(788줄)와 `vitis_ai_worker_yolo.py`(434줄)를 **코드에 나온 순서 그대로** 함수 하나씩 짚어가며 "무엇을 하고, 그 내용이 무슨 뜻인지" 설명한다. 작성 2026-07-20.

---

## 0. 전체 아키텍처 / 흐름

### (A) 두 프로세스 + 한 파이프

```
        /camera/camera/color/image_raw  (sensor_msgs/Image, BGR8 848x480)
                     │
                     ▼
┌──────────────────────────────────────────────────────────────┐
│  프로세스 A : vitis_ai_detector_node.py   (rclpy O = ROS 배관) │
│                                                                │
│   image_callback ──▶ [latest_msg]  ──▶ worker_loop            │
│   (구독 콜백)         (최신 1장만)      (전용 스레드)          │
│                                            │                   │
│                                            ▼                   │
│                                     process_frame              │
│                                       │        ▲               │
│                              stdin ▼  │        │ ▲ stdout      │
└────────────────────────────────────┼──┼────────┼──────────────┘
                                      │  │        │   pipe(IPC)
                       JSON헤더+raw바이트 │        │ JSON 응답
┌─────────────────────────────────────┼──┼────────┼────────────┐
│  프로세스 B : vitis_ai_worker_yolo.py ▼  │        │ (rclpy X)  │
│                                                                │
│   main 루프 ──▶ worker.detect ──▶ [DPU 하드웨어] ──▶ 응답      │
│   (stdin 읽기)   letterbox→추론→decode→NMS                     │
└──────────────────────────────────────────────────────────────┘
                     │
                     ▼
        /detections  (my_interfaces/DetectionArray)
```

- **프로세스 A(node)** 는 ROS를 안다(`rclpy`). 카메라를 구독하고, worker에게 일을 시키고, 결과를 `/detections`로 발행하고, 프레임마다 시간을 잰다.
- **프로세스 B(worker)** 는 ROS를 모른다. `xir`/`vart`(Vitis-AI 런타임)만 쓴다. 이미지 바이트를 받아 박스를 뱉는 순수 계산 프로세스.
- **왜 둘로 나눴나**: VART `execute_async()`가 rclpy 노드 안에서 돌면 XIR 객체 GC 문제로 segfault가 난다 → 프로세스로 격리해서 회피(`worker load_model:225`, `node start_worker:435`).
- **계약(contract)**: A→B는 `{height,width,...}` **JSON 한 줄 + raw 이미지 바이트**. B→A는 `{detections,timing,error}` **JSON 한 줄**. 이 계약 덕에 모델을 바꿔도(SSD↔YOLO) worker 내부만 바뀌고 node는 그대로다.

### (B) 프레임 한 장의 여정 (함수 흐름)

```
[node] image_callback → worker_loop → process_frame
          │                               ├─ ros_image_to_bgr        (ROS Image → numpy BGR)
          │                               └─ detect_with_worker ─────┐
          │                                                          │ pipe write
[worker]  main 루프 → read_exact → worker.detect                     ▼
                                     ├─ letterbox_lut   (전처리: 416 letterbox + int8)
                                     ├─ execute_async/wait  (★ DPU 추론)
                                     ├─ decode_head ×heads  (grid → 박스+conf)
                                     └─ make_detections (NMS + letterbox 역변환)
                                            │ write_response (pipe)
[node]  read_worker_json → json_items_to_detections → publish → /detections
```

아래부터는 이 함수들을 **코드에 나온 순서대로** 하나씩 본다.

---

## 1. `vitis_ai_detector_node.py` — 코드 순서대로

### 모듈 상단 (import + 상수)

- **import [3–24]**: `subprocess`(worker 프로세스 띄우기), `select`(worker 응답 타임아웃), `signal`(SIGTERM 처리), `threading`(파이프라이닝), `cv2`/`numpy`(이미지), `rclpy`(ROS). `my_interfaces`에서 커스텀 메시지 `Detection`, `DetectionArray`.
- **`BOX_COLORS` [27]**: overlay 박스 색 dict `{car, bicycle, person}`. ⚠️ **옛 SSD stand-in 색의 잔재** — 현재 6-class와 교집합이 person 하나뿐이고, overlay가 평소 off라 실행엔 영향 없다.

### `_default_model_path()` [34]
> 역할: launch 없이 `ros2 run`만 해도 모델을 찾도록 기본 xmodel 경로를 준다.

`get_package_share_directory("vitis_ai_detector_pkg")`로 패키지 설치 위치를 얻어 `models/yolov3_tiny_7class.xmodel`을 반환. 패키지 안에 모델이 동봉돼 있어 `/home/<user>/...` 같은 절대경로 가정이 없다. 실패하면 `""`.

### `_default_worker_script_path()` [48]
> 역할: 쓸 worker 스크립트의 기본 경로.

`os.path.dirname(os.path.abspath(__file__))`, 즉 **이 노드 파일과 같은 디렉터리**의 `vitis_ai_worker_yolo.py`. 그래서 node와 worker가 항상 같은 소스 트리에서 짝이 된다.

### `ros_image_to_bgr(msg)` [55]
> 역할: ROS `Image` 메시지를 OpenCV가 쓰는 numpy **BGR** 배열로 변환.

- encoding을 보고 채널 수를 정한다(bgr8/rgb8=3, bgra8=4, mono8=1) [56–65].
- `np.frombuffer(msg.data, uint8)` — **복사 없이** 바이트를 numpy로 본다 [71].
- `rows = data.reshape((height, step))` 후 `[:, :expected_row_bytes]` — **`msg.step`(한 줄의 바이트 수)로 reshape한 뒤 실제 폭만 잘라낸다.** 이게 핵심: ROS 이미지는 줄 끝에 패딩이 있을 수 있어서 width가 아니라 step으로 줄을 끊어야 한다 [78–79].
- encoding에 맞춰 `cv2.cvtColor`로 BGR 통일(rgb8이면 RGB→BGR) [87–94].

### `class VitisAiDetectorNode(Node)`

#### `__init__` [100]
> 역할: 노드 구성 전체 — 파라미터, QoS, 발행/구독, worker 프로세스, 처리 스레드를 세팅. **분기 없이 선형**이라 훑고 지나가면 된다.

순서대로:
1. `declare_parameter` 25개 [103–128] — 튜닝 가능한 값들의 선언+기본값(`process_period_sec`, `send_resized_input`, `publish_overlay`, `metrics_csv_path` 등).
2. 그 값들을 `self.*`로 읽어 저장 [130–164].
3. `metrics_rows = [] if metrics_csv_path else None` [161] — CSV 경로가 있을 때만 프레임 계측을 메모리에 쌓는다.
4. 상태 변수 초기화 [168–173] — `worker_process`, `last_worker_timing` 등.
5. **파이프라이닝 상태** [179–183] — `frame_lock`(뮤텍스), `latest_msg`(최신 프레임), `frame_event`(새 프레임 신호), `shutdown_event`, `worker_thread`.
6. **QoS 정의**: `output_qos`=RELIABLE depth1(발행용, 놓치면 안 됨) [185], `image_qos`=BEST_EFFORT depth1(구독용, 최신 1장) [191].
7. `create_publisher(/detections)` [198], (옵션) overlay 발행자 [203–216].
8. `create_subscription(input_topic, self.image_callback, image_qos)` [218] — 여기서 카메라와 연결.
9. `detector_mode`가 `"worker"`인지 확인 [225] → `start_worker()` [230].
10. `worker_loop`를 도는 **데몬 스레드 시작** [232–235].

#### `image_callback(msg)` [249]
> 역할: 카메라 프레임이 올 때마다 실행되는 구독 콜백. **최대한 가볍게.**

`frame_lock`을 잡고 `latest_msg = msg`, `frame_event.set()` 하고 **즉시 반환**한다. 무거운 DPU 작업을 여기서 하지 않기 때문에 ROS executor가 카메라를 굶기지 않는다. 이전 프레임이 아직 안 처리됐어도 그냥 덮어써서 **항상 최신 1장만** 남긴다(latest-frame-only).

#### `worker_loop()` [256]
> 역할: 별도 스레드에서 프레임을 **연속 처리**하는 루프. DPU worker가 놀지 않게.

- `shutdown_event`가 설 때까지 반복 [260].
- `frame_event.wait(timeout=0.5)` — 새 프레임이 올 때까지 대기(0.5초마다 종료 여부 재확인) [261].
- lock 잡고 `latest_msg`를 꺼내고 비운다(집은 건 다시 안 봄) [263–266].
- `should_process_frame()` 게이트를 통과하면 `process_frame(msg)` [269–271].

#### `process_frame(msg)` [273]
> 역할: **한 프레임의 실제 처리.** 다만 160줄 중 **핵심 로직은 281–316의 16줄뿐이고, 318–428은 전부 계측/로그**다. 처음엔 계측부를 접고 봐라.

핵심 로직 [280–316]:
- `image = ros_image_to_bgr(msg)` — ROS Image → BGR [281].
- `detections = self.detect_with_worker(image)` — worker에게 추론 시킴 [284].
- `out_msg.header = msg.header` — **입력 이미지의 헤더(타임스탬프)를 그대로 복사** [288]. 이 덕분에 나중에 검출과 원본 이미지를 stamp로 짝지을 수 있다.
- `self.publisher.publish(out_msg)` — 검출이 있거나 `publish_empty_detections`면 `/detections` 발행 [291–292].
- (overlay 켜졌으면) 박스 그려서 발행 [295–315] — 평소 off.

계측부 [318–428]:
- 시간 3층 분해: `detect_ms = img_ms + worker_call_ms`, `worker_call_ms = worker_ms + ipc_overhead_ms`, `worker_ms = pre+dpu+post` [318–338 주석].
- `metrics_csv_path`가 있으면 프레임마다 dict를 `metrics_rows`에 append [340–380], 지정 시간 지나면 CSV 저장.
- 아니면 `should_log()`일 때 한 줄 로그로 출력 [382–428].
- 예외는 `TimeoutExpired`/`Exception`을 잡아 에러 로그 [430–433].

#### `start_worker()` [435]
> 역할: worker 서브프로세스를 띄우고 "준비됨" 핸드셰이크를 받는다.

- 이미 살아있으면 그냥 반환 [436].
- `command = [python, worker_script, --model, model_path, --softmax, ...]` 구성 [439–448].
- `subprocess.Popen(..., stdin=PIPE, stdout=PIPE, stderr=PIPE)` — 파이프로 연결된 자식 프로세스 생성 [451].
- `read_worker_json("worker ready", startup_timeout)` — worker가 모델 로드(수 초)를 마치고 보내는 `{"status":"ready","input_shape":[...]}`를 기다림 [458–467].
- `input_shape`에서 worker 입력 H/W를 뽑아 저장 [469–473].

#### `stop_worker()` [479]
> 역할: worker 프로세스를 단계적으로 종료.

`process`를 지역변수로 빼고 self를 먼저 비운다(재진입 안전). 살아있으면 **stdin close → wait 2s → terminate → wait 2s → SIGKILL** 순으로 점점 강하게 [487–499].

#### `restart_worker()` [501]
> 역할: `stop_worker()` + `start_worker()`. worker가 죽거나 파이프가 깨졌을 때 되살리는 데 쓴다.

#### `read_worker_json(label, timeout_sec)` [505]
> 역할: worker stdout에서 **JSON 한 줄**을 타임아웃과 함께 읽는다.

- `select.select([stdout], [], [], timeout)` — 지정 시간 안에 읽을 게 없으면 `TimeoutExpired` [513–515].
- `readline()`이 빈 줄이면 worker가 죽은 것 → `stderr`를 읽어 원인과 함께 예외 [517–526].
- 정상이면 `json.loads` [528].

#### `detect_with_worker(image)` [530]
> 역할: 한 이미지를 worker에 보내고 검출 리스트를 받는 **IPC 왕복**.

- `np.ascontiguousarray(uint8)` + 3채널 확인 [531–533].
- worker가 죽었으면 `restart_worker()` [535–536].
- `send_resized_input`이면 worker 입력 크기로 resize(현재 YOLO는 **false**라 원본 그대로 보냄) [540–551].
- `request` dict 구성: `{height,width,channels,data_len,source_width,source_height}` [555–562].
- **stdin에 JSON 한 줄 + 이미지 바이트를 써서 flush** [565–567] → `read_worker_json("worker inference")`로 응답 [568].
- 파이프 깨지면 restart 후 예외 [569–571]. 응답에 error면 예외 [573–574].
- `self.last_worker_timing = response.timing` 저장, `json_items_to_detections`로 변환해 반환 [576–577].

#### `json_items_to_detections(items)` [579]
> 역할: worker가 준 dict 리스트를 ROS `Detection[]` 메시지로 변환.

각 dict의 `class_id/class_name/confidence/center_x/center_y/width/height`를 `Detection()`에 채워 리스트로 [581–590].

#### `prepare_overlay(image)` [594]
> 역할: (overlay용) 이미지를 표시 해상도로 줄이고 스케일 비율을 반환. overlay 꺼져 있으면 안 쓰임.

`overlay_width/height`로 `cv2.resize`하고 `(overlay, sx, sy)` 반환. 크기가 같거나 0이면 원본 복사 + 스케일 1.0.

#### `draw_detections(image, detections, scale_x, scale_y)` [616]
> 역할: 검출 박스와 라벨을 이미지에 그린다(overlay용).

각 det의 중심±절반으로 코너 계산 후 스케일·클램프 [618–626], `BOX_COLORS`에서 색(없으면 초록) [628], `cv2.rectangle` + `cv2.putText`로 `"이름 0.xx"` 라벨.

#### `make_overlay_msg(image, header)` [647]
> 역할: numpy 이미지를 ROS `Image`(bgr8) 메시지로 포장. raw overlay 발행용.

#### `make_compressed_overlay_msg(image, header)` [660]
> 역할: numpy 이미지를 `cv2.imencode(".jpg")`로 JPEG 압축해 `CompressedImage`로 포장. compressed overlay 발행용.

#### `should_process_frame()` [676]
> 역할: 검출 속도를 제한하는 **최소 간격 게이트**(divider 아님).

`process_period_sec<=0`이면 항상 통과 [677]. 아니면 마지막 처리 후 경과가 `process_period_sec` 미만이면 `False`(skip) [686–688]. 통과 시 타임스탬프 갱신. → 30fps여도 45ms 게이트면 2프레임에 1번(15Hz).

#### `should_log()` [693]
> 역할: 로그를 `log_period_sec`(기본 1초)마다 한 번만 내보내는 게이트. `should_process_frame`과 같은 패턴.

#### `frame_age_ms(msg)` [707]
> 역할: 이 프레임이 **캡처된 뒤 지금까지 얼마나 지났나**(ms). `현재시각 - header.stamp`. stamp가 0이거나 음수 age면 `None`. latency 측정의 근거.

#### `_round(value, ndigits=3)` [718]
> 역할: (staticmethod) CSV용 반올림 유틸 — 값이 `None`이면 빈 칸.

#### `stop_pipeline()` [725]
> 역할: 종료 시 **worker 스레드부터** 멈춘다. `shutdown_event.set()` + `frame_event.set()`(대기 중이면 깨움) + `worker_thread.join()`. metrics_rows에 더 이상 append가 안 되게 하는 게 목적.

#### `save_metrics_csv()` [734]
> 역할: 모아둔 `metrics_rows`를 지정 CSV로 저장(`csv.DictWriter`). 경로/행이 없으면 아무것도 안 함.

### `main(args)` [761]
> 역할: 진입점. `rclpy.init` → 노드 생성 → `rclpy.spin`.

특이점: **SIGTERM을 `KeyboardInterrupt`로 바꾸는 핸들러**를 건다 [767–770]. `pkill`로 종료해도 Ctrl-C처럼 `finally`가 돌아서 metrics CSV가 저장되고 worker가 정리된다 [777–784].

---

## 2. `vitis_ai_worker_yolo.py` — 코드 순서대로

### 모듈 docstring + import + 상수

- **docstring [2–19]**: 이 파일이 뭔지(SSD worker와 같은 JSON 계약의 YOLO판), SSD와 다른 점(letterbox 416, yolov5 grid decode, objectness pre-filter)을 요약. **제일 먼저 읽을 것.**
- **import [21–31]**: `xir`, `vart`(Vitis-AI). `rclpy` 없음 — ROS를 모르는 프로세스.
- **상수 [34–42]**: `DEFAULT_THRESHOLD=0.50`, `CLASS_THRESHOLDS_BY_NAME={person:0.30}`(안전 클래스는 recall 우선), `NMS_THRESHOLD=0.45`, `KEEP_TOP_K=100`, `PAD_VALUE=114`(letterbox 회색).

### `get_dpu_subgraphs(graph)` [45]
> 역할: xmodel 그래프에서 **DPU에서 도는 subgraph**만 골라낸다.

루트 subgraph를 위상정렬로 자식들로 펼친 뒤, `device` 속성이 `"DPU"`인 것만 필터 [46–51]. (컴파일이 잘 됐으면 DPU subgraph는 1개.)

### `tensor_shape(tensor)` [54]
> 역할: 텐서의 shape을 튜플로. `dims` 또는 `get_shape()` 중 있는 걸 씀(버전 호환).

### `tensor_fix_point(tensor)` [62]
> 역할: 텐서의 **fix_point**(양자화 소수점 위치)를 반환, 없으면 0. → 실제값 = int8값 / `2^fix_point`.

### `sigmoid(x)` [68]
> 역할: 시그모이드. `exp` 오버플로 방지로 입력을 `[-30, 30]`으로 clip.

### `iou(box, boxes)` [72]
> 역할: 한 박스와 여러 박스의 **IoU(겹침 비율)를 벡터로** 한 번에 계산. NMS의 부품.

교집합 넓이 / (합집합 넓이). 분모에 `1e-9`를 더해 0 나눗셈 방지 [82].

### `nms(boxes, scores, threshold)` [85]
> 역할: **Non-Max Suppression** — 겹치는 박스 중 점수 높은 것만 남긴다.

점수 내림차순 정렬 [88] → 맨 앞을 keep에 넣고 [92] → 그것과 IoU가 `threshold` 초과인 나머지를 버림 [96] → 반복. keep된 인덱스 리스트 반환.

### `build_input_lut(input_fix)` [100]
> 역할: **전처리 LUT** — uint8 픽셀값(0~255) → int8 양자화 입력의 256칸 표.

`round(v/255 * 2^fix)`를 `[-128,127]`로 clip [102–104]. 매 프레임 계산 대신 표 조회로 대체하려는 것. (`v/255`는 정규화, `*2^fix`는 양자화.)

### `letterbox_lut(image, input_size, lut, out_buf, state)` [107]
> 역할: **letterbox + BGR→RGB + LUT 양자화를 한 번에** 해서 `out_buf`(입력 텐서)에 기록. 좌표 역변환용 `(ratio, pad_x, pad_y)` 반환.

- `r = min(416/h, 416/w)` — 비율 유지 축소율 [117]. `nh, nw` = 축소 후 크기.
- 필요하면 `cv2.resize` [119–120].
- `top/left` = 가운데 정렬 pad 위치 [126–127].
- pad(회색)는 `(nh,nw)`가 바뀔 때만 다시 칠함 — 고정 해상도면 최초 1회뿐(콘텐츠 영역은 매 프레임 덮어써지니 안전) [129–132].
- `cv2.cvtColor(BGR2RGB)` 후 `cv2.LUT`로 int8 매핑해 out_buf 중앙에 넣음 [134–138]. (`cv2.LUT(x,lut)==lut[x]`.)

### `decode_head(raw, scale, anchors_np, stride, num_classes, thr_q)` [142]
> 역할: DPU 출력 한 scale(격자)을 **박스+conf로 디코드**. yolov5식.

- 출력을 `(H,W,na,no)`로 reshape [159] (na=anchor 수, no=5+클래스).
- **objectness pre-filter** [161–165]: int8 raw 그대로 `obj_q >= thr_q`인 칸만 `np.nonzero`로 고른다. 대부분 배경이라 여기서 걸러 sigmoid/decode 비용을 아낀다. (`conf=sig(obj)*sig(cls) ≤ sig(obj)`라 안전한 필요조건.) 하나도 없으면 빈 배열 조기반환.
- 고른 것만 `/scale`로 역양자화 후 `sigmoid` [167–168].
- yolov5 decode [171–174]: `cx=(sig_x*2-0.5+격자x)*stride`, `bw=(sig_w*2)^2*anchor`.
- `boxes`(xyxy) + `conf=sig(obj)*sig(cls)` 반환 [176–178].

### `read_exact(stream, size)` [181]
> 역할: 스트림에서 **정확히 `size` 바이트**를 읽는다(부족하면 반복, 끊기면 `None`). 파이프로 이미지 raw를 받을 때 씀.

### `class YoloWorker`

#### `__init__(model_path, meta_path, log_file)` [192]
> 역할: **`decode_meta.json`을 읽어** 모델 스펙을 세팅하고 `load_model` 호출.

- meta에서 `names`, `num_classes`, `input_size`, `strides`, `anchors_pixel` 로드 [198–204]. **← 클래스 수·이름의 정본은 파일명이 아니라 여기.**
- 클래스별 threshold 배열 구성(person만 0.30, 나머지 0.50) [206–209].
- `min_obj_logit` = 최소 threshold의 logit − ε — pre-filter 임계값의 원본 [210–212].
- `self.load_model()` [214].

#### `log(message)` [216]
> 역할: `log_file`이 설정됐을 때만 파일에 append(평소 off, 파일 I/O 비용 제거).

#### `load_model()` [223]
> 역할: **1회 셋업** — "프레임마다 하면 느린 것"을 전부 여기 모아 미리 한다. 그래서 `detect`가 가볍다.

- `xir.Graph.deserialize` → `get_dpu_subgraphs` → `vart.Runner.create_runner` [227–232]. **graph/subgraph를 `self`에 붙들어 둔다** — 해제되면 execute_async가 segfault(§0의 격리 이유) [225 주석].
- 입력 텐서 shape/fix_point 확인, meta의 input_size와 일치 검사 [236–243].
- `build_input_lut` + 입력 버퍼 `input_buf` 준비 [245–248].
- **프레임 불변 상수 미리 계산** [252–265]: 각 출력 head의 `(index, scale=2^fix, anchors 배열, stride, thr_q=min_obj_logit*scale)`를 `head_params`에 저장.
- 출력 버퍼 `output_data`(int8) 준비 [270–273].

#### `detect(image, output_width, output_height)` [276]
> 역할: **한 프레임 처리의 심장.** §0-B의 흐름 그대로 + 단계별 timing.

- `letterbox_lut(...)` → `(ratio, pad_x, pad_y)`, 소요 `pre_ms` [284–287].
- `runner.execute_async([input_buf], output_data)` + `runner.wait(job_id)` — **DPU 추론**, `dpu_ms` [289–291].
- 각 head를 `decode_head`로 디코드해 boxes/conf를 concat [293–301].
- `make_detections(...)`로 최종 검출, `post_ms` [303–306].
- `last_timing`에 pre/dpu/post/worker_ms 기록 후 반환 [308–314].

#### `make_detections(boxes, conf, ratio, pad_x, pad_y, image_w, image_h, output_width, output_height)` [316]
> 역할: 디코드된 후보를 **클래스별 NMS + letterbox 역변환**으로 최종 정리해 dict 리스트로.

- `sx, sy` = 출력 해상도 스케일(현재 `send_resized_input:false`라 image==source → 1.0) [320–321].
- 클래스마다: threshold 넘는 것 mask → `nms` → keep된 박스마다 **letterbox 역변환** `x=(cb-pad)/ratio*s` [332–335], 화면 밖 클램프, 너무 작으면 skip [336–343].
- `{class_id, class_name, confidence, center_x, center_y, width, height}` dict 생성(좌표는 **원본 픽셀** 기준) [344–352].
- confidence 내림차순 정렬 후 상위 `KEEP_TOP_K`개 반환 [354–355].

### `write_response(response)` [358]
> 역할: 응답 dict를 **JSON 한 줄 + 개행**으로 stdout에 쓰고 flush. node의 `read_worker_json`이 이걸 받는다.

### `main()` [363]
> 역할: worker 진입점 — 인자 파싱 → 모델 로드 → **stdin/stdout 무한 루프**.

- 인자: `--model`(필수), `--meta`(기본은 model 옆 `decode_meta.json`), `--softmax`(SSD 호환용, **YOLO는 무시**), `--log-file` [364–371].
- `YoloWorker` 생성 후 `{"status":"ready","input_shape":...}` 전송 [386–390]. (node의 `start_worker`가 이걸 기다림.)
- 루프 [393–430]: stdin에서 JSON 한 줄 readline → `read_exact`로 이미지 바이트 → numpy 복원 [400–410] → `worker.detect(output=source 크기)` [414] → `write_response({detections,timing,error})` [419–423]. 예외는 error 응답으로 감싼다 [424–430].

---

## 부록: 1분 용어 사전

- **fix_point / int8 양자화**: DPU는 float 대신 int8로 계산. 실제값 = int8 / `2^fix`. 입력은 곱해서 넣고(`build_input_lut`), 출력은 나눠서 되돌린다(`decode_head`).
- **LUT**: 픽셀값이 256가지뿐이라 변환을 표로 미리 만들어 조회만. A53 CPU가 약해 크게 빠름.
- **letterbox**: 848×480을 416×416에 비율 유지로 넣고 남는 자리는 회색(114) pad. 그래서 마지막에 `(x-pad)/ratio`로 원본 좌표 복원.
- **grid decode**: YOLO 출력은 격자 칸마다 objectness+위치+크기+클래스. yolov5 공식으로 실제 박스로 푼다.
- **NMS**: 한 물체에 겹친 박스를 점수순으로 하나만 남김.
- **QoS depth=1 BEST_EFFORT**: 카메라 구독은 "최신 1장, 놓쳐도 됨" → latency 최소.
- **프로세스 격리**: VART segfault 회피용으로 worker를 별도 프로세스로.

> 관련 정본: 파라미터 값의 근거 [`workflow.md`](workflow.md) · 최적화 측정 델타 [`vision_final.md`](vision_final.md) · 전체 상태 [`../STATUS.md`](../STATUS.md).
