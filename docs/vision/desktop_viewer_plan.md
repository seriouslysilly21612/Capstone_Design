# 데스크톱 bbox overlay 뷰어 — ✅ 실행 완료

**작성·실행 2026-07-16.** 보드 실측 조사(8 에이전트 / 125 사실 / 34 리스크 + 적대적 3중 검증) 기반으로 계획하고, **같은 날 실물로 완주했다.**
목적: 파이프라인 검출 결과를 데스크톱 화면에서 bbox overlay 영상으로 본다. **보드 부하는 순감**이어야 한다.

> **2026-07-20 통합**: `viewing.launch.py`(검출 전용)를 `pick_place_vitis_ai.launch.py`에 흡수하고 삭제했다. 이제 launch는 **하나**다 — 전체 pick 파이프라인이 항상 돌고 데스크톱 뷰어는 거기 붙는다. `realsense_viewing.yaml`(color 15 fps)도 제거했다(프로덕션 pick path 신선도 때문에 **color 30 fps 유지**). 통합 근거: 압축 토픽은 플러그인(`ros-humble-compressed-image-transport`)만 깔려 있으면 전체 파이프라인에서도 광고되고 **구독 시에만 인코딩**되므로(§1.2 lazy) 뷰잉 전용 launch가 애초에 불필요했다 — 2026-07-20 보드 실측으로 재확인(전체 6노드 기동 + `/compressed` 광고 + Subscription 0에서 인코딩 0 + `/detections` 15.3 Hz). **아래 본문의 `viewing.launch.py`·`realsense_viewing.yaml` 명령/수치는 2026-07-16 당시 기록이라 그대로 둔다** — 실행은 `pick_place_vitis_ai.launch.py`로 하면 된다.

## 실행 결과 (2026-07-16, 전 게이트 통과)

| Gate | 결과 |
|---|---|
| 1 — compressed 토픽 | ✅ `/camera/camera/color/image_raw/compressed` 생성 |
| 2 — `.msg` 무결성 | ✅ **md5 4/4 일치** (보드 ↔ 데스크톱) |
| 3 — **원격 discovery (최초 2-host 시험)** | ✅ 데스크톱에서 3개 토픽 모두 관측 |
| 4 — 렌더 정확성 | ✅ 5클래스 동시 검출, 박스 정렬, 라벨 정확 |
| 5 — 보드 부하 | ✅ **0.79 core** (전체 파이프라인 1.8 core 대비 절반 이하) |

- **처리율**: 압축 스트림·`/detections` 각 **15 Hz**. 영상 버벅임 없음.
- **CPU 내역**: camera 36.8% (JPEG 인코딩으로 29→36.8, **+7.8%p**) + detector **21.1%** (31→21.1, color 15fps 전환으로 버려질 프레임의 콜백이 사라져 **−10%p**) + worker 21.1%. **인코딩 비용을 15fps 전환이 상쇄하고도 남았다.**
- **Gate 4 실물**: apple 0.85 / orange 0.88 / banana 0.85 / tennis_ball 0.81 / mustard_bottle 0.83 동시 검출. 이 한 장면이 네 가지를 동시에 증명했다 — ① `decode_meta.json` 클래스 순서 정확(코드로는 검증 불가했던 항목: apple·banana·orange가 한 화면에 있는데 라벨이 안 뒤바뀜) ② 색 채널 처리 정확(틀렸다면 오렌지가 파랗게 보였을 것) ③ bbox가 원본 좌표계 맞음(역변환 안 넣은 판단이 옳음) ④ stamp 조인 정확.
- **조인 100% 달성** (커밋 `8eb6338`): 초기 join이 95~100%로 흔들렸는데 버그가 아니라 **경합**이었다 — detection은 프레임 캡처 ~37 ms 뒤에 발행돼 네트워크를 건너고, 압축 이미지는 인코딩 후 독립 경로로 건너므로 **어느 쪽이든 먼저 도착할 수 있다.** 뷰어가 detection 도착 순간에만 이미지를 찾고 없으면 버렸던 게 원인(30장 버퍼는 이미지가 *먼저* 올 때만 유효). **조인을 양방향으로** 바꿔 `on_image`에서도 대기 중인 detection과 짝을 맞추게 했다.
  - 실측: `img=75 det=75 drawn=75 (hit=70 late=5) drop=0 stale=0` — **전 구간 100%**. `late` 1~9개/5초가 예전에 버려지던 바로 그 프레임들이다.
  - **단조 가드**(늦게 온 이미지가 이미 그린 최신 프레임보다 오래됐으면 skip)는 실측에서 **한 번도 발동하지 않았다**(`stale=0`) — 한 프레임 주기(66 ms)보다 덜 늦은 이미지는 어차피 다음 detection보다 먼저 도착하기 때문. 병적 지연용 보험이다.
  - 부수 효과: 카운터가 `hit`/`late`/`drop`/`stale`로 분리돼 **"조인 문제"와 "네트워크 손실"이 구분된다.** `drop=0`이면 링크가 깨끗한 것.

> 저장소가 private이라 데스크톱 `git clone`은 인증을 묻는다. 연구실 PC에 PAT를 남기지 않도록 **`scp`로 두 패키지만** 받았다(§3 참고). `.msg` 무결성은 md5 대조로 보장.

---

> ⚠️ 이 문서의 수명. 이 계획의 **전임 버전은 "보드 = CycloneDDS"라는, 작성 시점엔 맞았지만 실행 시점엔 거짓이 된 전제** 때문에 핵심 설정을 정확히 반대로 지시했다. 아래 §0의 "전제 스냅샷"을 착수 전에 반드시 재확인할 것. 전제가 바뀌었으면 계획이 아니라 전제를 먼저 고쳐라.

---

## 0. 전제 스냅샷 (착수 전 재확인)

| 항목 | 값 | 재확인 명령 |
|---|---|---|
| 보드 RMW | `rmw_fastrtps_cpp` (launch가 pin) | `grep RMW_IMPL src/system_bringup_pkg/launch/pick_place_vitis_ai.launch.py` |
| 데스크톱 RMW | **설정 안 함** (Humble 기본이 곧 fastrtps) | `printenv RMW_IMPLEMENTATION` → 빈 값이어야 정상 |
| ROS_DOMAIN_ID | **양쪽 다 unset = 0** | `printenv ROS_DOMAIN_ID` → 양쪽 빈 값 |
| 데스크톱 distro | Humble (보드와 동일) | `printenv ROS_DISTRO` |
| 보드 IP | 192.168.120.132/24 eth0 (DHCP) | `ip -4 addr show eth0` |
| color | 848×480×30, `/camera/camera/color/image_raw` | `realsense_pick_place.yaml:30` |
| detections | `/detections`, 15 Hz | `vitis_ai_detector.yaml:24,35` |

---

## 1. 아키텍처와 그 근거

```
[Kria 192.168.120.132]
  realsense ─┬─→ /camera/camera/color/image_raw           (raw, 로컬 SHM으로 detector에게)
             └─→ /camera/camera/color/image_raw/compressed (JPEG — 구독자 있을 때만 인코딩)
                        │                                   
  vitis_ai_detector ────┼──→ /detections                   (RELIABLE, 15 Hz, header = color header 복사본)
                        │
                   (UDPv4 / eth0 / domain 0)
                        ▼
[Desktop Humble]  detection_viewer_node
                    CompressedImage + DetectionArray → stamp 조인 → cv2.imdecode → draw → imshow
```

**보드는 압축만, 그리기는 전부 데스크톱.** `publish_overlay`는 계속 OFF.

### 왜 보드에서 그리면 안 되는가 (실측)

| 항목 | 값 |
|---|---|
| `detect_ms` 평균 / p95 | 37.63 / 40.16 ms |
| 현재 설정 overlay 비용 | **44.14 ms/frame** |
| 합 | **81.8 ms** |
| 15 Hz 예산 | **66.6 ms** |

→ `publish_compressed_overlay: true`로 켜면 **검출률이 조용히 15 Hz 아래로 떨어진다.** 계측이 안 붙어 있어서(`metrics_csv_path: ""`) 눈치채기도 어렵다.

> 문서에 적힌 "overlay ~70 ms"와 재측정치 15 ms / 44 ms가 서로 다투는 중이다(무엇을 벤치했는지에 따라 갈림). **이 계획은 그 논쟁의 결과와 무관하게 성립한다** — 데스크톱에서 그리면 보드는 이 비용을 아예 치르지 않기 때문. 이게 이 아키텍처를 고른 진짜 이유다.

### 왜 압축 비용이 평소에 0인가 (기계어 수준 확인)

`image_transport::Publisher::publish`가 플러그인 벡터를 돌며 `getNumSubscribers()`(vtable slot 3)를 호출하고 **0이면 `publish()`(slot 5)를 건너뛴다.** 설치된 `libimage_transport.so` 디스어셈블로 확인. 즉 **데스크톱 뷰어를 끄면 JPEG 비용이 즉시 0으로 돌아온다.**

주의: `getNumSubscribers()`는 **raw + compressed 전 transport를 합산**한다. compressed만 구독해도 realsense의 publish 경로 전체가 깨어난다(의도된 동작).

---

## 2. 조사로 확정된 것 (추측 아님)

### ✅ bbox 좌표는 원본 848×480 픽셀 — 뷰어는 변환하지 말 것

worker가 letterbox를 **이미 되돌린다**: `x1 = (cb[k,0] - pad_x) / ratio * sx` (`vitis_ai_worker_yolo.py:332-335`). 그리고 `send_resized_input: false`라 `sx = sy = 1.0`. node는 좌표를 **손대지 않고** JSON 실수를 그대로 msg에 복사(`vitis_ai_detector_node.py:579-592`).

→ **뷰어는 받은 숫자를 원본 프레임에 그대로 그린다.** 416×416 역변환 금지.

```python
xmin = int(round(det.center_x - det.width  / 2.0))
ymin = int(round(det.center_y - det.height / 2.0))
xmax = int(round(det.center_x + det.width  / 2.0))
ymax = int(round(det.center_y + det.height / 2.0))
```

### ✅ stamp 조인은 유효 — header가 바이트 복사본

`vitis_ai_detector_node.py:287-292`: `out_msg.header = msg.header` — 대입이지 재스탬프가 아니다. 파일 전체에서 `now()`는 계측용 3곳뿐이고 header에 들어가지 않는다. **`(sec, nanosec)` 정확 일치 조인이 성립한다.**

### ✅ QoS

| 토픽 | 발행자 | 뷰어 구독 |
|---|---|---|
| `/detections` | RELIABLE, KEEP_LAST 1, VOLATILE | **RELIABLE** depth=1 |
| `.../image_raw/compressed` | SYSTEM_DEFAULT → **RELIABLE** (BEST_EFFORT 아님!) | **BEST_EFFORT** depth=1 |

`/compressed`는 `/image_raw`의 QoS를 그대로 상속한다. RELIABLE 발행자 ↔ BEST_EFFORT 구독자는 **호환**(발행자가 더 강한 보장을 제공). 네트워크 재전송 지연을 피하려면 뷰어는 BEST_EFFORT로 받는 게 낫다.

### ✅ DDS는 원격을 막지 않는다 — XML 수정 불필요

`useBuiltinTransports=false`는 기본 **transport**를 끄지, 기본 **locator**를 끄지 않는다. UDPv4가 `userTransports`에 있으므로 FastDDS가 거기서 metatraffic locator를 파생시킨다. 프로파일을 물린 probe로 실측: `239.255.0.1` eth0 조인 확인, `0.0.0.0` 바인딩 확인, 대조군(프로파일 없을 때 549 KB vs 있을 때 16.8 MB 세그먼트)으로 **프로파일이 진짜 로드됐음**까지 확인.

**`interfaceWhiteList`를 추가하지 말 것** — eth0가 DHCP인데 FastDDS 2.6은 CIDR을 지원하지 않아 IP를 하드코딩하게 되고, lease가 바뀌면 조용히 깨진다.

### ✅ 방화벽은 지금 안 막는다 (단, 지뢰)

`/etc/ufw/ufw.conf` → `ENABLED=no`. `systemctl is-active ufw`가 `active`로 나와도 **룰을 설치하지 않는다.** 단 `/etc/default/ufw` → `DEFAULT_INPUT_POLICY="DROP"`이라 **누가 `sudo ufw enable`을 하는 순간 DDS가 전부 죽는다.** (iptables 레벨 확인은 root 필요 — 미확인.)

---

## 3. ★ 최대 위험: .msg drift = 조용한 오염 데이터

**Humble에는 type hash가 없다.** `rmw_topic_endpoint_info_t`에 `type_hash` 필드가 없고(`/opt/ros/humble/include/rmw/`에 `type_hash` grep 0건), 매칭은 **토픽 이름 + 타입 *이름* + QoS**로만 이뤄진다. **`.msg` 내용은 비교되지 않는다.**

→ 데스크톱의 `Detection.msg`가 필드 순서 하나만 달라도 **엔드포인트는 정상 매칭되고, 뷰어는 잘못된 레이아웃으로 CDR을 역직렬화한다.** 에러도, no-match도 아니고 **그럴듯해 보이는 쓰레기 값**이 나온다.

**대응: 파일을 복사하지 말고 같은 git 저장소를 clone한다.** 그러면 drift가 구조적으로 불가능해지고, 어긋나면 git이 알려준다.

```bash
# 데스크톱
git clone https://github.com/seriouslysilly21612/Capstone_Design.git ~/kria_viewer_ws
cd ~/kria_viewer_ws && colcon build --packages-select my_interfaces detection_viewer_pkg --symlink-install
```

검증 (양쪽에서 같은 값이 나와야 함):
```bash
md5sum src/my_interfaces/msg/*.msg
```

> `my_interfaces`는 외부 의존이 `ament_cmake` / `rosidl_default_generators` / `std_msgs` / `rosidl_default_runtime` 넷뿐이고 보드 전용 의존(VART/DPU/realsense)이 **0개**라 데스크톱에서 그대로 빌드된다. **`.msg` 4개를 전부** 가져가야 한다(`CMakeLists.txt`가 4개를 모두 `rosidl_generate_interfaces`에 넘김).

---

## 4. Phase 1 — 보드: 압축 transport

```bash
sudo apt install ros-humble-compressed-image-transport   # 후보 2.5.5, 현재 미설치
```

JPEG 품질을 낮추려면 (파라미터 이름 실측 확정):
```bash
ros2 param set /camera/camera camera.color.image_raw.compressed.jpeg_quality 70
```
**이름이 틀리면 조용히 무시된다** — set 후 `ros2 param get`으로 되읽어 확인할 것.

`ros-humble-image-transport` 3.1.12는 **이미 설치돼 있다** — 플러그인만 없다.

**설치 후 카메라 노드를 반드시 재시작.** 플러그인은 pluginlib `ClassLoader` 생성 시 **한 번만** enumerate되고 재스캔하지 않는다. `Publisher::publish`는 미리 만들어진 고정 벡터를 순회할 뿐 조회하지 않는다.

**Gate 1** — 두 토픽이 새로 생겨야 한다:
```bash
ros2 topic list | grep compressed
#   /camera/camera/color/image_raw/compressed        ← 우리가 쓸 것
#   /camera/camera/depth/image_rect_raw/compressed   ← 부산물 (구독 안 하면 비용 0)
```

플러그인이 붙을 조건 2개는 **이미 충족**돼 있다: `_use_intra_process=false`(rs_launch.py가 standalone Node로 띄움), `USE_LIFECYCLE_NODE` 미정의(apt 빌드 기본 OFF).

---

## 5. Phase 2 — 데스크톱

```bash
# RMW: 설정하지 않는다. ROS_DOMAIN_ID: 설정하지 않는다. 둘 다 기본값이 이미 맞다.
# FASTRTPS_DEFAULT_PROFILES_FILE: 설정하지 않는다 (보드 전용 프로파일이다).
printenv RMW_IMPLEMENTATION ROS_DOMAIN_ID FASTRTPS_DEFAULT_PROFILES_FILE   # 셋 다 비어야 정상

git clone <repo> ~/kria_viewer_ws && cd ~/kria_viewer_ws
colcon build --packages-select my_interfaces detection_viewer_pkg --symlink-install
source install/setup.bash
```

**데스크톱에 `compressed_image_transport`는 필요 없다.** `sensor_msgs/CompressedImage`를 평범한 rclpy 구독으로 받아 `cv2.imdecode`하면 된다.

**Gate 2**:
```bash
ros2 interface show my_interfaces/msg/Detection    # 7필드가 보드와 동일한 순서인지
```

---

## 6. Phase 3 — 연결 (★ 진짜 관문)

**모든 검증을 이름의 *등장*으로 확인할 것. 부재로 추론하지 말 것.** ros2cli의 기본 discovery window가 짧아 **불완전한 그래프를 정상인 양 반환**한다(실증: `--no-daemon`이 `/tf_static`을 누락, `--spin-time 8`을 주면 찾음).

```bash
# 데스크톱
ros2 topic list --no-daemon --spin-time 8 | grep -E "detections|color/image_raw"
ros2 topic hz /camera/camera/color/image_raw/compressed
ros2 topic echo /detections my_interfaces/msg/DetectionArray --once   # ★ 타입 명시가 가장 날카로운 시험
ros2 topic info /detections -v | grep -i "node name"                  # _NODE_NAME_UNKNOWN_ 이면 안 됨
```

**Gate 3 실패 시 분해 순서** (전임 계획의 "ufw부터"는 틀렸다 — ufw는 지금 안 막는다):

1. **멀티캐스트 자체** — DDS를 배제하고 10초에 판정:
   ```bash
   # 보드          | 데스크톱
   ros2 multicast receive   |   ros2 multicast send
   ```
   실패면 스위치 IGMP snooping 문제. 보드 XML로 고칠 수 없다.
2. **domain / RMW** — 양쪽 `printenv ROS_DOMAIN_ID RMW_IMPLEMENTATION`이 모두 비었는지.
3. **스테일 env** — `test -f "${FASTRTPS_DEFAULT_PROFILES_FILE:-/dev/null}" && echo OK || echo BROKEN`
4. **ufw** — `sudo iptables -L -n`으로 진짜 상태 확인(`systemctl is-active`는 거짓말한다).

---

## 7. Phase 4 — 뷰어 설계

`src/detection_viewer_pkg/detection_viewer_pkg/detection_viewer_node.py` 참조. 핵심 판단:

- **detection 도착 시에 렌더** — detector가 최신 프레임만 처리하고 나머지를 버리므로 `/detections`(15 Hz)가 color(30 Hz)보다 sparse하다. detection 기준으로 그려야 박스가 항상 그 프레임에 정확히 정렬되고 stale 박스가 안 생긴다.
- **stamp 조인** — color 프레임을 `(sec, nanosec)` 키로 최근 ~30장 버퍼링, DetectionArray 도착 시 동일 stamp를 꺼낸다. 없으면 그 detection은 버린다(로그만).
- **원본 해상도로 그린다** — 창을 리사이즈해서 그리면 sx/sy를 각각 계산해야 하고, 균일 스케일 하나로 처리하면 **y 좌표가 틀어진다**(848/480 = 1.767 aspect). 원본에 그린 뒤 `imshow`가 알아서 표시하게 두는 게 안전하다.
- **색상 맵은 `decode_meta.json`에서 만든다** — 노드의 `BOX_COLORS`(`vitis_ai_detector_node.py:27-31`)는 **구 SSD용 {car, bicycle, person}**이라 현재 6개 클래스와 교집합이 **0**이다. 복사하면 전부 기본색으로 떨어진다. `class_name` 문자열이 매 Detection에 실려 오니 그걸 쓴다.
- **빈 배열 처리** — `publish_empty_detections: true`라 0개짜리도 stamp와 함께 온다 → 박스 clear에 그대로 쓴다.

---

## 8. Phase 5 — 보드 부하 최소화

**뷰잉 전용 launch**를 따로 만든다: camera + detector만. `pick_logic` / `pick_target_3d` / `pick_target_base` 제외 → 약 **0.9 core 절감**.

> 🚨 **새 launch에 `SetEnvironmentVariable` 2줄을 반드시 복사해 넣을 것.** 빠뜨리면 보드가 조용히 SHM을 잃고 CPU를 **+6.6%p** 더 쓴다 — 이 계획의 목적과 정반대다.

**color 30 → 15 fps를 기본으로.** `process_period_sec: 0.045`는 *최소 간격* 게이트다:

| color | 프레임 간격 | 게이트 통과 | 검출률 |
|---|---|---|---|
| 30 fps | 33.3 ms | 2프레임마다 (33.3 < 45 → skip) | 15 Hz |
| **15 fps** | **66.6 ms** | **매 프레임 (66.6 ≥ 45)** | **15 Hz** |

→ **검출률은 그대로인데 JPEG 인코딩 횟수가 절반**이 된다. 30 fps로 인코딩하면 detection 기준 렌더에서 **절반을 버리므로** 순수 낭비다. 프로덕션 `realsense_pick_place.yaml`을 고치지 말고 뷰잉용 config를 따로 둘 것.

---

## 9. Gate 4/5 — 검증

**Gate 4 (기능)**: 실물 top-down 물체에 박스가 정렬되고 라벨·confidence가 표시된다.
- 기준값(2026-07-15 실측): orange 0.88~0.90, tennis_ball 0.90.
- **apple 하나 + banana 하나를 같이** 놓고 라벨이 뒤바뀌지 않는지 확인 → `decode_meta.json`의 `names` 순서가 학습 라벨 순서와 맞는지 검증하는 유일한 실물 시험.

**Gate 5 (부하)**: 뷰어를 켠 상태의 보드 총 CPU가 **뷰어 없는 지금(~1.8 core / 44%)보다 낮거나 비슷**해야 한다. 3D 노드를 끈 절감(−0.9 core)이 JPEG 비용(15 fps 기준 추정 ~0.15–0.3 core)보다 크므로 **순감**이 나와야 정상.

> ⚠️ 기존 CPU 수치는 전부 **순정 커널** 측정치다. 지금 보드는 RT 커널(`5.15.199-rt91-rt-kv260c`)이고 RT 오버헤드가 +5~10%p로 예상된다. 절대값 비교는 무의미하니 **같은 커널에서 뷰어 ON/OFF를 비교**할 것.

---

## 10. 함정 목록 (증상이 조용한 순)

| # | 함정 | 증상 | 판별 |
|---|---|---|---|
| 1 | **`.msg` drift** | **에러 없이 그럴듯한 쓰레기 값** | 같은 repo clone. `md5sum msg/*.msg` 양쪽 대조 |
| 2 | 스테일 `FASTRTPS_DEFAULT_PROFILES_FILE` | 에러 1줄 흘러가고 정상 동작, CPU만 +6.6%p | `test -f "$FASTRTPS_DEFAULT_PROFILES_FILE"` |
| 3 | ros2 daemon이 옛 env를 pin | CLI만 오작동, 파이프라인은 멀쩡 | `tr '\0' '\n' < /proc/$(pgrep -f ros2cli.daemon)/environ \| grep FASTRTPS` → `ros2 daemon stop` |
| 4 | 짧은 discovery window | 토픽이 없는 것처럼 보임 | `--spin-time 8` 주고 **등장**을 확인 |
| 5 | `ufw enable` | 원격만 전멸, 보드는 멀쩡(SHM) | `sudo iptables -L -n` |
| 6 | 플러그인 설치 후 카메라 미재시작 | `/compressed`가 안 생김 | Gate 1 |
| 7 | 고아 프로세스 + 삭제된 SHM 세그먼트 | discovery는 되는데 데이터가 안 옴 | `grep -l 'fastrtps.*(deleted)' /proc/*/maps` |
| 8 | per-class NMS | 한 물체에 두 클래스 박스(유령) | 뷰어 버그 아님. `/detections` echo로 확인 |
| 9 | per-class threshold (person 0.30, 나머지 0.50) | 박스 깜빡임 | 뷰어 버그 아님. conf가 임계 근처인지 확인 |
| 10 | `ros2 run`으로 노드 단독 실행 | `send_resized_input` 기본값이 **True**라 정확도만 조용히 하락 | 항상 launch로 띄울 것 |

---

## 11. 미해결 — 착수 시 확인할 것

### ✅ 해소됨 (2026-07-16 실측)

- **encoding = `rgb8`** — 카메라를 직접 띄워 관측(`--field encoding --once`). **뷰어에 색 반전 함정은 없다**: `compressed_image_transport`는 컬러 입력을 인코딩 전에 `bgr8`로 변환하고 `format`에 `"rgb8; jpeg compressed bgr8"`로 기록한다. 즉 JPEG 바이트에 들어있는 건 BGR이고, `cv2.imdecode(..., IMREAD_COLOR)`는 BGR을 돌려주며, 이는 `cv2.imshow`가 원하는 그대로다. **변환 코드를 넣지 말 것** — 넣으면 오히려 반전된다.
- **`jpeg_quality` 파라미터 이름 = `camera.color.image_raw.compressed.jpeg_quality`** — 선행 점 없음. 이미 노출된 `camera.color.image_raw.enable_pub_plugins`를 실물 관측해 접두어 규칙을 확정(topic `/camera/camera/color/image_raw`에서 노드 namespace `/camera`를 떼고 `/`→`.`).
- **viewing config 동작 확인** — `realsense_viewing.yaml`로 color 실측 **14.7 Hz**, width 848.

### 남은 것

- **2-host 시험은 한 번도 없었다**: 이 문서의 DDS 근거는 전부 same-host 측정이다. Phase 3이 곧 최초의 진짜 시험이다.
- **UDP 수신 버퍼**: `net.core.rmem_max = 212992`(208 KB), MTU 1500. 37 KB JPEG는 ~25개 데이터그램으로 쪼개지고 **하나만 잃어도 프레임 전체가 날아간다.** 프레임 드롭이 보이면 `nstat -az UdpRcvbufErrors`로 확인 후 `rmem_max` 상향.
- **RT 커널 baseline 부재**: 모든 CPU 수치가 순정 커널 측정치.
