# Kria KV260 Pick & Place — Perception Pipeline

RealSense D435i → DPU 물체검출(YOLOv3-tiny 6-class) → 2D 필터 → 단일점 3D 역투영 → base frame 목표점.
ROS 2 Humble, 15 Hz, E2E ~81 ms.

```
[D435i] → camera → vitis_ai_detector_node ─(pipe)→ vitis_ai_worker_yolo (DPU)
                          │ /detections
                          ▼
                     pick_logic_node → /pick_target
                          ▼
                   pick_target_3d_node → /pick_target_3d
                          ▼
                  pick_target_base_node → /pick_target_base   ← 최종 출력
```

---

## 실행

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws && colcon build --symlink-install    # --symlink-install 필수
source install/setup.bash

ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py
```

확인 (다른 터미널):
```bash
ros2 topic hz /detections        # ~15 Hz
ros2 topic hz /pick_target_base  # 최종 단계까지 흐르는지
```

> `build/`와 `install/`은 `--symlink-install`로 서로 묶여 있습니다. **하나만 지우면 링크가 깨집니다** — 지울 때는 항상 둘 다 지우고 재빌드하세요.

---

## 전제조건 (git으로 받아지지 않는 것들)

이 저장소에는 **우리 코드만** 들어 있습니다. 아래는 보드에 미리 갖춰져 있어야 합니다.

| # | 항목 | 버전 | 확인 명령 |
|---|---|---|---|
| 1 | Kria KV260 + Ubuntu | 22.04 | `uname -a` |
| 2 | ROS 2 | Humble | `printenv ROS_DISTRO` |
| 3 | DPU (`kv260-smartcam` overlay) | B3136 | `xdputil query` |
| 4 | Vitis-AI runtime (VART/XIR) | 2.5.0 | `python3 -c "import vart, xir"` |
| 5 | RealSense | librealsense 2.57.7 | `rs-enumerate-devices` |
| 6 | D435i 펌웨어 | **5.17.0.10** | `rs-enumerate-devices \| grep Firmware` |
| 7 | realsense-ros | 4.57.7 | `ros2 pkg prefix realsense2_camera` |

### 1. DPU 활성화

`kv260-smartcam` overlay가 부팅 시 자동 로드되어야 합니다.

```bash
sudo systemctl enable --now kv260-smartcam
xdputil query        # DPUCZDX8G_ISA1_B3136 / fingerprint 0x101000016010406 확인
```

fingerprint가 다르면 이 모델(xmodel)은 **로드되지 않습니다**. 모델은 이 DPU에 맞춰 컴파일된 것이라 fingerprint가 일치해야 합니다.

### 2. Vitis-AI runtime

`import vart, xir`이 되면 끝입니다. 보드 이미지에 포함돼 있거나 Xilinx 패키지로 설치합니다.
(모델을 새로 만들 게 아니라면 **데스크톱 Vitis-AI 2.5 docker는 필요 없습니다.** 재학습/재컴파일 시에만 필요 — `yolo_v3_tiny_training/README.md` 참고.)

### 3. RealSense

```bash
sudo apt install ros-humble-librealsense2 ros-humble-realsense2-camera
```

**펌웨어 5.16.0.1은 쓰지 마세요.** 스트리밍 수십 초 후 RGB가 멈추는 버그가 있습니다(원인·진단법: `progress.md` §13). 5.17.0.10으로 업데이트:

```bash
wget https://librealsense.intel.com/Releases/RS4xx/FW/D4XX_FW_Image-5.17.0.10.bin
rs-fw-update -f D4XX_FW_Image-5.17.0.10.bin
```

### 4. realsense-ros 소스 (저장소에 없음)

`src/realsense-ros`는 third-party라 gitignore돼 있습니다. 별도로 클론하세요:

```bash
cd ~/ros2_ws/src
git clone -b 4.57.7 https://github.com/IntelRealSense/realsense-ros.git
```

### 5. DDS 설정 (성능에 직결 — 빠뜨리면 CPU +6.6%p)

노드 간 1.16 MB Image를 UDP loopback이 아니라 **shared memory**로 보냅니다. `~/.bashrc`에 추가:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/ros2_ws/fastdds_shm_profile.xml
```

확인: `env | grep FASTRTPS`.
빠뜨려도 **에러 없이 동작**하지만 CPU를 6.6%p 더 씁니다. (기본 SHM segment가 512 KB라 1.16 MB Image가 안 들어가서 UDP로 조용히 폴백 — 그래서 프로파일 XML이 필요합니다.)

### 6. RT 커널 (선택 — EtherCAT 로봇제어 단계에서만)

비전 파이프라인만 돌릴 거면 **순정 커널로 충분합니다.** EtherCAT 통합 시에는 RT 커널이 필요하고, 직접 빌드해야 합니다:

- 최종 커널: `5.15.199-rt91-rt-kv260c`
- 필수 패치 2개가 이 저장소에 있습니다:
  - `kernel_configs/vanilla-5.15.199-radix-fix/` — Ubuntu SAUCE radix-tree revert 되돌리기 (없으면 RT에서 부팅 중 위반 253건)
  - `zocl_patches/apply_zocl_uaf_fix.py` — zocl KDS use-after-free (없으면 DPU 가동 ~30초 후 커널 crash)
- 전체 절차: `rt_patch.md`, 사건 기록: `rt_kernel_postmortem.md`

---

## 문서 지도

| 문서 | 내용 |
|---|---|
| `integrated_progress.md` | **여기부터** — 통합 허브 + 정본 라우팅 표 |
| `workflow.md` | 노드별 파라미터와 **그 값의 근거** |
| `vision_final.md` | 비전 모델 전체 (SSD→YOLO 학습·DPU 배포·최적화) |
| `rt_patch.md` / `rt_kernel_postmortem.md` | RT 커널 구축 / 크래시 규명 |
| `progress.md` | 시간순 전체 히스토리 |

---

## 알아둘 것

- **모델**: `vitis_ai_work/models/yolov3_tiny_7class.xmodel` — 파일명은 `7class`지만 실제는 **6-class**(apple/orange/banana/tennis_ball/mustard_bottle/person). `decode_meta.json`과 **반드시 같은 디렉터리**에 있어야 합니다(worker가 xmodel 옆에서 자동 탐색).
- **`base_link → camera_link` TF는 placeholder**입니다. 캘리브레이션 전까지 `/pick_target_base`는 파이프라인이 동작함을 증명할 뿐, 실제 로봇 좌표가 아닙니다.
- **rclpy에서 노드를 병합하지 마세요.** 시도했다가 CPU +5.4pt 역효과를 실측했습니다(rclpy executor가 매 콜백마다 waitset을 재구성). 근거와 코드: `target_3d_pkg/pick_post_stack.py`.
