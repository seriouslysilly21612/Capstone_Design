# RPU 작업 가이드 — Claude 세션용 운영 문서

> **대상 독자: Claude (Opus 포함 모든 모델의 새 세션).**
> 목적: 이전 대화 기록 없이 이 문서만으로 RPU FreeRTOS + SOEM 작업을 막힘없이 이어가게 하는 것.
> 작성: 2026-07-08 (Fable, 보드 실사 기반) / **최종 재검증·갱신: 2026-08-03, 커널 `5.15.199-rt91-rt-kv260c` 기준.** 아래 "검증된 사실"은 전부 이 보드에서 명령으로 직접 확인된 것.
> **현재 트랙 상태 (2026-08-03)**: Gate 1(remoteproc + FreeRTOS 구동) 착수 단계. GEM3 이관(Phase 2+)은 사용자 재결정 전 금지 — APU IgH가 Indy7 실운용에 사용 중.

---

## 0. 필독 순서

1. **이 파일 전체** — 특히 §1(세션 환경)과 §4(금지 사항)는 작업 전 반드시 숙지
2. `~/ros2_ws/docs/rpu/rpu_plan.md` — 전체 논리(왜 이 순서인가, 포팅 난제 분석)
3. `~/ros2_ws/docs/rpu/rpu_freertos_soem_execution_plan.md` — Gate 기준·리스크 표
4. `~/ros2_ws/CLAUDE.md` — 프로젝트 전체 규칙 (한국어 답변, 학부 수준 설명, source priority)
5. 질문 주제별 공식 문서 링크: `~/ros2_ws/docs/reference/reference_02_openamp_freertos_ethernet.md`
6. 지속 메모리: `kria-rt-preempt-project.md` (APU IgH 트랙과의 관계 포함)
7. APU 쪽(RAON-RT/IgH) 맥락이 필요하면: `~/ros2_ws/docs/RAON-RT/merge.md` (정본 로그 — RPU+SOEM은 §9 백로그 항목)

---

## 1. 세션 환경 — 가장 먼저 인지할 것

**Claude Code는 KV260 보드 그 자체에서 실행 중이다.** (`uname -r` → `5.15.199-rt91-rt-kv260c`, PREEMPT_RT, hostname `kria`)

이것이 의미하는 것:
- 보드 상태를 추측하지 말 것. `/proc/device-tree`, `/sys`, `dmesg`로 직접 확인 가능하다.
- **원격 = AX88179B USB Ethernet NIC(`enx…`, 드라이버 `ax_usb_nic` v4.1.0, `192.168.120.50/24`) 경유 SSH. Tailscale은 제거됨.**
  eth0(GEM3)은 더 이상 원격 생명줄이 아니다 — 대신 **IgH EtherCAT 예약 포트**다(아래 참조). USB NIC를 죽이는 명령(`ip link set enx… down`, `ax_usb_nic` 언로드, USB 리셋)이 이제 세션 즉사 경로다.
- **eth0(GEM3)의 현재 상태**: 평시엔 macb 소유·IP 없음(유휴). RAON-RT 실기 세션 때 IgH(`ethercat.service` 기동)가 점유해 Indy7을 제어한다. **RPU가 GEM3를 가져가는 Phase 2+는 사용자 재결정 전 착수 금지.**
- **로봇 실기 세션 중 금지(E34)**: RAON-RT 실기 구동 중에는 보드에서 무거운 작업(빌드, 대량 IO, 에이전트 다중 실행) 금지 — 1kHz 제어 지터를 오염시킨 전례가 있다. RPU 작업은 실기 세션 밖에서.
- **sudo는 비밀번호 필요** (passwordless 아님). sudo가 필요한 단계(dmesg, iomem, overlay 적용, modprobe)는 사용자에게 실행을 요청하거나 비밀번호 입력을 받는다.
- Vitis는 이 보드에서 실행 불가(aarch64). **모든 R5 elf 빌드는 사용자의 x86 PC에서** 이루어진다. Claude는 소스/링커스크립트/DT를 작성해 주고, 빌드는 사용자에게 절차를 안내하는 분업 구조다. elf 전송은 USB NIC 경유 scp — GEM3 상태와 무관하다.
- 사용자 작업 스타일: 일일이 허락 묻지 말고 자율 진행하되, **연결 단절·비가역 작업만은 사전 고지**. 답변은 한국어(기술용어는 영어), 학부생 수준에서 원리부터 설명.

**원격 작업 온보딩 (새 세션 첫 5분)**:
```bash
uname -r                                   # 5.15.199-rt91-rt-kv260c 인지 확인 (다르면 §3 전체 재검증)
ip -br link; ip route | head -3            # 원격 경로가 enx…(USB NIC)인지, eth0 상태 확인
systemctl is-active ethercat               # active면 로봇 세션 가능성 → 사용자에게 실기 여부 확인 후 작업
ls /sys/class/remoteproc/                  # remoteproc0 있으면 overlay 적용된 상태
ls /sys/kernel/config/device-tree/overlays/  # kv260-smartcam_image_1 + (있다면) rpu
cat /sys/devices/system/cpu/isolated       # 3 — CPU3는 RAON-RT 전용, 침범 금지
```

---

## 2. 확정 결정사항 — 재논의 금지

| 결정 | 내용 | 근거 |
|---|---|---|
| 실행 코어 | R5F-**0**, **split** 모드 | lockstep 불필요, 코어1은 추후 여유 |
| RTOS | FreeRTOS 10 (Vitis BSP `freertos10_xilinx`) | Kria 공식 지원 경로 |
| EtherCAT master | **SOEM** v1.4.x (RPU) / IgH는 APU 전용 | SOEM만이 RTOS 포팅 가능 구조 |
| Vitis 버전 | **2022.1** | kria-apps-docs 튜토리얼 기준, 커널 5.15 세대 일치. **2026-08-03 현재도 미설치** → Phase 0에서 설치 |
| RPU 로드 | Linux **remoteproc** (+ 안정화 후 systemd 자동화) | Kria는 BOOT.BIN 수정이 비표준(부트펌웨어 QSPI 고정) |
| 코드 배치 | DDR 예약 영역 링크, 지연민감 코드만 TCM | SOEM+제어 코드가 TCM 128KB 초과 |
| DMA 버퍼 | 처음엔 **non-cacheable** (R5 MPU) | 캐시 일관성 버그 원천 차단, 최적화는 동작 후 |
| SOEM 수신 | 처음엔 **폴링**, 이후 인터럽트+큐 | 단순한 것부터 검증 |
| 작업 순서 | **당분간 Phase 0~1만** (2026-08-03 사용자 확정). Phase 2+(GEM3 이관)는 별도 사용자 결정 필요 | APU IgH는 "완료"가 아니라 **실운용 중**(RAON-RT가 Indy7 제어) — GEM3를 가져가면 그동안 로봇 운용 중단. 이관 시 APU 제어 중단 합의 + macb rebind 복구 절차 필수 |
| 대상 로봇 | Indy7, STEP 우회 드라이브 직결, CiA402+CSP | APU에서 IgH+RAON-RT로 실제 제어 성공·운용 중 → Phase 4는 **IgH→SOEM 번역 이식** (`~/RAON-RT/EMasterApp/Device/EcatCommon.h`가 IgH 타입 사용) |
| 테스트 슬레이브 | LS Mecapion **L7N** 서보 + 모터 + 전원 보유 | 문서 PDF는 사용자에게 요청 → `~/ros2_ws/docs/l7n/` (2026-08-03 현재 미수령) |
| 범위 | 그리퍼 제외, 말단부가 물체 위 정지까지. **주기 1kHz 확정** | RAON-RT APU 트랙이 1kHz로 실운용하며 실증 (2026-07~08) |

---

## 3. 검증된 보드 사실 + 재검증 명령

작업 시작 시(특히 재부팅/커널 업데이트 후) 아래를 다시 실행해 상태가 유지되는지 확인하라.
**2026-08-03에 RT 커널(`5.15.199-rt91-rt-kv260c`) 기준으로 전 항목 재검증 완료** — 7/8의 우려(커스텀 커널에서 remoteproc 지원 소실)는 발생하지 않았다. 커널 재빌드 불필요.

| 사실 | 검증 명령 | 2026-08-03 결과 (RT 커널) |
|---|---|---|
| remoteproc 드라이버 모듈 존재, vermagic 일치, **binding = `xlnx,zynqmp-r5-remoteproc`** (Xilinx 5.15 벤더 binding — upstream 6.x의 `zynqmp-r5fss`와 다름!) | `modinfo /lib/modules/$(uname -r)/kernel/drivers/remoteproc/zynqmp_r5_remoteproc.ko \| grep alias` | 존재·alias 확인. 커널 config: `ZYNQMP_R5_REMOTEPROC=m`, `REMOTEPROC=y`, `ZYNQMP_IPI_MBOX=y` |
| rpmsg 스택 완비 | `ls /lib/modules/$(uname -r)/kernel/drivers/rpmsg/` | rpmsg_core/char/ns/virtio 전부 존재 (`RPMSG*=m`) |
| DT에 R5 노드 없음 → overlay 필요 | `ls /sys/class/remoteproc/` | 비어 있음 (모듈도 미로드 — 정상, probe 대상 없음) |
| configfs overlay 동작 (`OF_OVERLAY=y`, `OF_CONFIGFS=y`) | `ls /sys/kernel/config/device-tree/overlays/` | `kv260-smartcam_image_1` applied — RT 커널에서 실증 |
| GEM3만 활성 (ff0e0000, macb, eth0). **평시 유휴(IP 없음), IgH 세션 시 점유** | `for g in ff0b ff0c ff0d ff0e; do cat /proc/device-tree/axi/ethernet@${g}0000/status; done`; `ip -br addr show eth0` | disabled×3/okay. eth0 UP·IP 없음. IgH kmod는 `/lib/modules/$(uname -r)/ethercat/`에 이 커널용으로 빌드되어 있고 `ethercat.service`는 필요 시 기동 |
| 원격 = USB NIC | `ip route \| head -3` | default via 192.168.120.1 dev `enx…` (ax_usb_nic v4.1.0) |
| UART0(ff000000) **disabled**, UART1(ff010000)=Linux 콘솔(ttyPS1) | `cat /proc/device-tree/axi/serial@ff000000/status` | disabled (변화 없음) |
| 기존 IPI: mailbox@ff9905c0, `xlnx,ipi-id = <4>` (충돌 금지) | `od -An -tx1 /proc/device-tree/zynqmp-ipi/mailbox@ff9905c0/xlnx,ipi-id` | 00 00 00 04 |
| reserved-memory 기존 항목은 `pmu@7ff00000`뿐 | `ls /proc/device-tree/reserved-memory/` | R5 carve-out은 overlay가 추가해야 함 |
| RAM 3.8GiB, CMA 1000M 중 **여유 ~340-400MB** (smartcam/VCU/zocl 점유분 제외) | `grep -i cma /proc/meminfo` | CmaFree ~340-400MB (7월의 13MB에서 크게 완화 — 시점따라 변동) |
| cmdline: `clk_ignore_unused`, `cma=1000M` + **RT 격리 파라미터 추가됨** | `cat /proc/cmdline` | `skew_tick=1 isolcpus=3 nohz_full=3 rcu_nocbs=3 irqaffinity=0-2` — **CPU3는 RAON-RT 1kHz 전용, RPU 작업이 CPU3에 부하 금지** |
| 부팅: U-Boot + flash-kernel `image.fit` (`.bak`+`image.fit.stock-1070` 백업 존재) | `ls /boot/firmware/` | 확인됨. flash-kernel db에 KV260 항목 존재 |
| 모듈 서명 강제 없음 (OOT 모듈 로드 자유) | `cat /sys/kernel/security/lockdown` | `[none]` |

**RPU 예약 메모리 확정 전 필수 확인**: `sudo dmesg | grep -iE 'cma|reserved'` 및 `sudo cat /proc/iomem`으로 CMA/기존 예약 영역의 실제 주소를 확인하고 겹치지 않게 잡을 것. 관례상 0x3ed00000 부근(Xilinx OpenAMP 예제 표준)을 쓰되 **이 보드의 CMA 배치와 겹치지 않는지 반드시 검증**.

---

## 4. 금지·주의 사항 (Opus가 세션을 망치는 지름길들)

1. **USB NIC(`enx…`)를 건드리는 명령 = 원격 세션 즉사** — 절대 금지. **eth0 관련 명령도 사전 고지 없이 실행 금지** (IgH EtherCAT 예약 포트 — 로봇 트랙과 충돌). unbind 복구는 `echo ff0e0000.ethernet > /sys/bus/platform/drivers/macb/bind` 또는 재부팅.
1-b. **RAON-RT 실기 세션 중에는 RPU 작업 자체를 보류** (E34 — 보드 부하가 1kHz 지터 오염). `systemctl is-active ethercat`이 active면 사용자에게 실기 여부부터 확인.
2. **QSPI 부트펌웨어(`xmutil bootfw_*`)를 건드리지 말 것.** 이 프로젝트에 불필요하며 잘못되면 복구가 어렵다.
3. **kv260-smartcam overlay를 내리지 말 것** (`xmutil unloadapp` 금지). perception 파이프라인(DPU)이 그 위에서 돈다. RPU 작업과 충돌하지 않는다.
4. `/boot/firmware/image.fit` 직접 수정 금지. DT를 boot-time에 바꿀 땐 §6-B의 flash-kernel 절차만 사용(자동 백업 유지).
5. **RPU reserved-memory를 runtime configfs overlay로만 잡은 채 실사용 금지** — §6-B의 이유 참조. 실험은 가능하나 본 운용은 boot-time 반영 후.
6. 이 커널의 remoteproc binding은 **Xilinx 5.15 벤더 형식**이다. 웹에서 찾은 upstream 6.x용 DT 예제(`xlnx,zynqmp-r5fss`, `compatible = "xlnx,zynqmp-r5f"`)를 그대로 쓰면 probe되지 않는다. 반드시 §6-A의 방법으로 이 커널 소스의 예제를 기준으로 삼을 것.
7. FreeRTOS elf에 **resource table 섹션이 없으면 remoteproc 로드가 실패하거나 경고 후 기능 제한**된다. 순정 "Hello World" 템플릿에는 없다 — §7의 지시대로 OpenAMP echo-test 템플릿을 베이스로 쓸 것.
8. 사용자에게 물어봐도 되는 것: L7N PDF 요청, Vitis 설치 진행 상황, **GEM3 이관 시점 합의(Phase 2 전제 — 로봇 운용 중단 수반)**, 실기 세션 일정. 그 외 기술 판단은 자율 진행. (master 종류=IgH, 주기=1kHz는 이미 확정 — §2 참조, 다시 묻지 말 것)

---

## 5. 문서·파일 지도

| 경로 | 내용 |
|---|---|
| `~/ros2_ws/docs/rpu/rpu_plan.md` | 마스터 계획 (논리 전개) |
| `~/ros2_ws/docs/rpu/rpu_freertos_soem_execution_plan.md` | 실행 절차·Gate·리스크 |
| `~/ros2_ws/docs/rpu/rpu_guide_for_claude.md` | 이 문서 |
| `~/ros2_ws/tools/rpu/` | RPU 작업 산출물 (dtso/dtbo, 스크립트) — Gate 1 착수 시 생성 |
| `~/ros2_ws/docs/l7n/` | L7N 드라이브 문서 (Phase 3 때 사용자로부터 수령 — 아직 없음) |
| `~/ros2_ws/src/apu_rpu_bridge_pkg/` | **삭제됨(2026-07-15 정리, `0313586`)** — Phase 5에서 재생성 |
| `~/ros2_ws/docs/RAON-RT/merge.md` | APU 트랙(IgH+RAON-RT) 정본 로그 — Phase 4 이식 원본 코드는 `~/RAON-RT/` |
| `/lib/firmware/` | RPU elf 배치 위치 |
| `/sys/kernel/config/device-tree/overlays/` | runtime overlay 적용 지점 |
| `~/ros2_ws/docs/reference/reference_02_*.md` | Kria FreeRTOS/OpenAMP 공식 링크 모음 |

**진행 기록 규칙**: Phase/Gate 달성·중요 발견·결정 변경 시 ①execution_plan의 해당 섹션 갱신 ②지속 메모리 `kria-rt-preempt-project.md` 갱신. 사용자 확인 사항은 날짜와 함께 기록.

---

## 6. Phase 1 상세 런북 — remoteproc + FreeRTOS

### 6-A. DT overlay 작성 (보드에서 가능)

**절대 원칙: 이 커널의 소스에서 binding과 예제를 추출해 그것만 따른다.**

```bash
# 방법 1 (권장): RT 커널을 빌드한 소스 트리가 사용자 x86 PC에 있다 (Ubuntu Xilinx 5.15 소스 + RT 패치).
#   그 트리의 Documentation/devicetree/bindings/remoteproc/ 와
#   drivers/remoteproc/zynqmp_r5_remoteproc.c (property 파싱 코드가 최종 진실) 를 기준으로 삼는다.
# 방법 2: https://github.com/Xilinx/linux-xlnx → 5.15 계열(xlnx_rebase_v5.15) 브랜치의 같은 경로.
#   (현재 커널은 커스텀 빌드라 apt-get source는 정확한 소스가 아님 — 베이스 확인용으로만)
```

overlay에 들어가야 하는 요소 (형식은 위 소스 기준으로 작성):
1. R5 클러스터 노드: `compatible = "xlnx,zynqmp-r5-remoteproc"`, split 모드 설정, r5f_0 child + TCM/sram 참조
2. `reserved-memory` children: R5 코드/데이터 영역 + rpmsg vring/buffer 영역 (§3의 주소 검증 후)
3. RPU용 IPI mailbox 노드: 기존 `xlnx,ipi-id = <4>`와 **충돌하지 않는 id** 사용 (Xilinx 예제 관례 참조)

컴파일: `dtc -@ -O dtb -o rpu.dtbo rpu.dtso` (`-@` 필수 — phandle 참조용)

### 6-B. 적용 — 두 경로의 원리와 사용 시점

**경로 1: runtime configfs (실험용, 재부팅=복구)**
```bash
sudo mkdir /sys/kernel/config/device-tree/overlays/rpu
sudo sh -c 'cat rpu.dtbo > /sys/kernel/config/device-tree/overlays/rpu/dtbo'
dmesg | tail -20   # overlay 적용 오류 확인
sudo modprobe zynqmp_r5_remoteproc
ls /sys/class/remoteproc/
```
**한계(중요)**: `reserved-memory`는 원래 **부팅 초기(memblock 단계)에만 예약**된다. runtime overlay로 추가한 예약 영역은 커널이 이미 그 페이지를 다른 용도로 쓰고 있을 수 있다 → 짧은 hello 실험은 대개 통과하지만 **본 운용에서 무작위 메모리 오염 리스크**. 그러므로:

**경로 2: boot-time 반영 (본 운용, flash-kernel 정식 경로)**
```bash
# 1) 현재 머신이 쓰는 dtb 파일명 확인
cat /proc/device-tree/compatible          # 머신 compatible
grep -ri "$(cat /etc/flash-kernel/machine 2>/dev/null)" /usr/share/flash-kernel/db/ 2>/dev/null | head
# 2) base dtb에 overlay를 오프라인 병합
fdtoverlay -i <base>.dtb -o merged.dtb rpu.dtbo   # fdtoverlay는 device-tree-compiler 패키지
# 3) flash-kernel의 custom dtb 오버라이드 경로에 배치 후 재생성
sudo mkdir -p /etc/flash-kernel/dtbs
sudo cp merged.dtb /etc/flash-kernel/dtbs/<원래 dtb와 동일한 파일명>.dtb
sudo flash-kernel     # image.fit 재생성 (자동으로 .bak 유지 — 그래도 사전에 수동 백업 권장)
sudo reboot
```
복구: `/etc/flash-kernel/dtbs/`의 파일 제거 → `sudo flash-kernel` → 재부팅. (`image.fit.bak`도 존재)

**권장 흐름**: 경로 1로 overlay 문법/probe를 빠르게 반복 검증 → 형식이 확정되면 경로 2로 전환 → 이후 모든 실사용은 경로 2 상태에서.

### 6-C. 펌웨어 로드·실행·로그

```bash
sudo cp rpu_app.elf /lib/firmware/
echo rpu_app.elf | sudo tee /sys/class/remoteproc/remoteproc0/firmware
echo start | sudo tee /sys/class/remoteproc/remoteproc0/state
cat /sys/class/remoteproc/remoteproc0/state   # → running
echo stop  | sudo tee /sys/class/remoteproc/remoteproc0/state
```

**RPU 로그를 보는 3가지 수단 (UART 문제 주의)**:
1. **remoteproc trace buffer (권장, 하드웨어 불필요)**: 펌웨어 resource table에 trace 엔트리가 있으면
   `sudo cat /sys/kernel/debug/remoteproc/remoteproc0/trace0` 으로 Linux에서 직접 읽는다. OpenAMP echo-test 템플릿에 포함됨.
2. **UART1 공유**: Linux 콘솔(ttyPS1)과 같은 포트라 출력이 섞인다. 읽기 전용 디버그 프린트는 가능하지만 지저분함 — 임시용.
3. **UART0**: DT상 disabled이고 **KV260 캐리어에서 물리적으로 어디로 나오는지 미확인** — 쓰려면 KV260 캐리어 회로도(UG1089 계열)에서 PS UART0 핀 라우팅부터 확인할 것. 확인 없이 stdout을 UART0으로 잡으면 "로그가 안 보이는" 헛수고를 하게 된다.

**Gate 1**: start/stop 반복 안정 + trace/UART로 FreeRTOS 태스크 로그 확인.

**막힐 때 진단 순서**: ①`dmesg`의 overlay 적용 오류(문법/phandle) ②probe 실패(binding 불일치 — §4-6 재확인, reserved-memory/IPI phandle 오류) ③elf 로드 실패(링커 주소가 reserved 영역 밖, resource table 부재 — §4-7) ④running인데 로그 없음(로그 수단 문제 — trace buffer로 전환).

---

## 7. PC측(Vitis 2022.1) 가이드 — 사용자 안내용

Claude는 보드에서 실행 불가하므로 아래를 사용자에게 절차로 안내한다.

1. **설치**: AMD Unified Installer로 Vitis 2022.1 (Linux 권장). 디바이스는 Zynq UltraScale+ MPSoC만 선택하면 용량 절감. 설치 후 `xsct` 동작 확인.
2. **XSA 확보**: Vivado 2022.1 → KV260 board preset(Kria K26 SOM + KV260 carrier)으로 빈 블록디자인 + ZynqMP PS 추가 → preset 적용 → generate → **Export Hardware (bitstream 불포함)** → `.xsa`. PL은 smartcam이 쓰므로 bitstream 불필요.
3. **플랫폼/앱 생성**: Vitis에서 XSA로 platform 생성 → domain: `psu_cortexr5_0`, OS: `freertos10_xilinx` → **application 템플릿은 "OpenAMP echo-test"를 베이스로** (resource table·rpmsg·trace 포함 — §4-7 함정 회피). hello 단계에서는 rpmsg 부분을 잠시 비활성화해도 resource table은 유지.
4. **링커 스크립트**: 코드/데이터를 §6-A에서 확정한 DDR 예약 영역으로. 벡터/ISR/주기태스크는 TCM(R5 view 0x0). resource table 섹션(`.resource_table`) 유지 확인.
5. **BSP 설정**: stdout/stdin = §6-C에서 정한 UART(또는 무UART + trace만). 이후 Phase용으로 XilPM 라이브러리 포함 권장(§8 클럭 이슈 대응).
6. elf를 보드로 전송(현재는 scp 가능; GEM 이관 후에는 USB 저장장치/SD 이용).

---

## 8. Phase 2 상세 — GEM3 이관의 함정 지도

**전제 확인 (2026-08-03 갱신)**: APU IgH는 "완료"를 넘어 **Indy7 실운용 중**이다. Phase 2는 GEM3를 가져가므로 그동안 APU 로봇 제어가 중단된다. 착수 전 필수: ①사용자와 운용 중단 기간 합의 ②macb rebind 복구 절차 준비(스크립트화) ③실기 세션과 겹치지 않는 시간대. 사용자 결정 없이 착수 금지.

### 절차 개요
1. 사용자 고지 + UART/물리 접근 준비 확인 (§1)
2. `echo ff0e0000.ethernet | sudo tee /sys/bus/platform/drivers/macb/unbind`
3. RPU에서 emacps 예제 기반 링크업 → raw frame 왕복 (PC와 직결, Wireshark 검증, EtherType 0x88A4)

### 함정 1 — 클럭 게이팅 (unbind의 숨은 부작용) ★반드시 읽을 것
macb 드라이버는 unbind(remove) 시 자기가 켰던 GEM3 클럭들을 **끈다**. `clk_ignore_unused`는 부팅 시 미사용 클럭을 안 끄는 옵션일 뿐, 드라이버가 명시적으로 끄는 것은 막지 못한다. 그 상태에서 RPU의 emacps 초기화는 클럭이 꺼진 하드웨어를 만지게 되어 실패한다(standalone 드라이버는 클럭이 이미 켜져 있다고 가정).

**확인 방법**: unbind 전후로 CRL_APB의 GEM3 클럭 레지스터를 읽어 비교.
```bash
# GEM3_REF_CTRL은 CRL_APB(0xFF5E0000) 영역. 정확한 오프셋/비트는 UG1085 레지스터 레퍼런스로 확인할 것.
sudo busybox devmem 0xFF5E005C   # busybox 미설치 시: sudo apt install busybox / 또는 devmem2
```
**해결 경로 (둘 중 하나)**:
- (a) RPU 펌웨어 초기화 코드에서 XilPM(`XPm_RequestNode` 등) 또는 CRL_APB 직접 write로 GEM3 클럭/전원을 스스로 켠다 — 자립적이라 권장.
- (b) boot-time DT에서 GEM3를 `status = "disabled"`로 바꿔 Linux가 아예 안 건드리게 한다(§6-B 경로) — psu_init(FSBL)이 켜둔 클럭이 그대로 유지됨. 단 이 경우 SSH 경로가 부팅부터 없다.
개발 초기는 (a)+unbind 조합(재부팅 복구 가능), 최종 운용은 (b) 권장.

### 함정 2 — DMA 캐시 일관성 (최대 난제)
- 증상: 간헐적 프레임 깨짐/옛 데이터, printf 넣으면 증상 소멸.
- 예방: **BD ring + 패킷 버퍼를 R5 MPU로 non-cacheable(Device/Strongly-ordered 아닌 Normal Non-cacheable) 영역에 배치**하고 시작. `Xil_SetTlbAttributes`/MPU 설정 API 사용. 캐시+수동 flush 최적화는 Gate 2 통과 후에만.

### 함정 3 — PHY (TI DP83867)
- embeddedsw emacps 예제에 DP83867 지원 코드 있음. PHY 주소는 MDIO 스캔으로 탐지. RGMII delay 설정이 링크업 실패의 단골 원인.
- 진단: MDIO로 PHY ID 레지스터가 읽히는가 → autoneg 완료 비트 → 링크 파트너 확인.

### 함정 4 — 인터럽트
- GEM3 IRQ(SPI 63, GIC 번호 95)가 RPU GIC(scugic)에 등록되어야 함. xparameters.h의 emacps 인터럽트 ID 사용.
- Linux가 살아있는 동안 같은 IRQ를 양쪽에서 잡으면 안 됨 — unbind가 선행되면 안전.

**Gate 2**: RPU↔PC 임의 L2 프레임 수천 회 무손실 왕복.
**진단 순서**: 클럭(함정1) → PHY 링크(함정3) → 송신만 되고 수신 안 됨(함정4) → 간헐 깨짐(함정2).

---

## 9. Phase 3 — SOEM 포팅 체크리스트

디렉토리 구조 (SOEM v1.4.x 기준):
```
soem/osal/freertos/osal.c            ← 신규 작성
soem/osal/freertos/osal_defs.h
soem/oshw/freertos/nicdrv.c          ← 신규 작성 (Phase 2 emacps 코드 래핑)
soem/oshw/freertos/nicdrv.h
soem/oshw/freertos/oshw.c            ← htons/ntohs 등 (기존 포트 복사 수준)
```

구현 함수 목록:
- `osal`: `osal_current_time`(TTC 기반 µs — FreeRTOS tick 1ms로는 불가), `osal_time_diff`, `osal_usleep`(sub-ms는 TTC 폴링), `osal_timer_start/is_expired`, mutex 계열(FreeRTOS 세마포어), `osal_thread_create`(초기엔 미사용 가능)
- `oshw`/nicdrv: `hw_open/close`, `hw_send`(emacps TX), `hw_receive`(초기 폴링), redundancy 관련은 단일 포트 스텁
- SOEM core는 무수정 원칙. 수정 충동이 들면 포팅 계층이 잘못된 것.

검증: `simple_test` 이식 → L7N 연결 → PREOP→SAFEOP→OP. 이 시점에 사용자에게 L7N PDF 요청(`~/ros2_ws/docs/l7n/`).

**Gate 3**: L7N OP 도달 + PDO read 정상.

---

## 10. Phase 4~5 요점

**Phase 4 (1kHz + DC + CiA402)**:
- TTC 인터럽트 1kHz 태스크에서 send/receive_processdata. DC 활성화, SYNC0 정렬.
- jitter를 R5 사이클 카운터(PMU cycle counter)로 측정·기록 — APU IgH 대비 정량 비교가 이 프로젝트의 성과 지표.
- L7N: CiA402 상태머신(Shutdown→Switch On→Enable Operation), CSP 모드. 랩 가이드(MAN-20241113-LX02H0001)의 드라이브 설정 개념 참조 가능(IgH 기준이지만 CiA402 시퀀스는 동일).
- 이후 **Indy7 제어 코드 이식**: 원본은 `~/RAON-RT/`(EMasterApp) — **IgH 기반 확정**(`EcatCommon.h`가 `ec_slave_config_t` 등 IgH 타입 사용) → SOEM API로 번역 계층 필요. CiA402 시퀀스·PDO 매핑·제어 로직(CTC, 마찰 FF 등)은 재사용, EtherCAT API 호출부만 교체. 상세 맥락은 `docs/RAON-RT/merge.md`.

**Phase 5 (rpmsg + ROS2 통합)**:
- RPU: OpenAMP echo-test 기반 rpmsg endpoint. Linux: `modprobe rpmsg_char` → `/dev/rpmsg*`.
- 메시지: 고정 크기 C struct + 버전 필드. APU→RPU(목표 관절값/궤적 세그먼트 + watchdog 카운터), RPU→APU(관절 상태/EtherCAT 상태/오류).
- **통신 방식 결정(2026-07-08)**: 초기 구현은 RPMsg 단독(현재 데이터량이 페이로드 한계 ~496B/msg 안). 병목 실측 시에만 raw shared memory 데이터면 추가(hybrid) — 그 경우 양측 non-cacheable + seqlock/double-buffer + Linux는 UIO(generic-uio 노드, cmdline에 이미 활성) 사용. cross-core mutex/atomic은 쓰지 말 것(A53↔R5는 캐시 coherency 도메인이 다름). 예약 영역은 처음부터 shm 확장분 포함해 잡아둘 것. 상세 논리: rpu_plan.md Phase 5 설계 노트.
- 안전: rpmsg watchdog timeout → RPU 자체 감속 정지. EtherCAT WKC 오류 → Quick Stop + 보고. **안전 로직은 전부 RPU 쪽에 둔다** (APU 죽어도 로봇은 멈춰야 함).
- `apu_rpu_bridge_pkg` 구현(`/pick_target_base` 구독). 부팅 자동화는 smartcam.service 패턴의 systemd unit: overlay 확인 → 펌웨어 로드 → start → bridge 기동.

**Gate 5 (최종)**: perception → RPU → Indy7 말단부가 물체 바로 위 정지, end-to-end 1회.

---

## 11. 트러블슈팅 도구 상자 (보드)

```bash
dmesg | tail -50                                  # overlay/remoteproc/드라이버 오류 1차 확인
sudo cat /sys/kernel/debug/remoteproc/remoteproc0/trace0   # RPU 로그 (trace 엔트리 필요)
cat /sys/class/remoteproc/remoteproc0/state       # offline/running
sudo cat /proc/iomem                              # 메모리 예약 실태
ls /sys/kernel/config/device-tree/overlays/       # 적용된 overlay
sudo busybox devmem <addr>                        # 레지스터 직접 확인 (클럭/PHY 진단)
ethtool -i eth0                                   # (이관 전) GEM 드라이버 상태
```
PC측: Vitis XSCT + JTAG (USB 케이블)로 R5 브레이크포인트 디버깅 가능 — trace/printf로 안 잡히는 문제(하드폴트 등)는 JTAG이 최후 수단.

## 12. 미결 사항 (해소되면 이 문서와 메모리 갱신)

- [x] ~~Indy7 제어 코드의 EtherCAT master 종류~~ → **IgH 확정** (2026-08-03, `~/RAON-RT/EMasterApp/Device/EcatCommon.h` 실사). Phase 4는 IgH→SOEM 번역 이식
- [x] ~~제어 주기 확정~~ → **1kHz 확정** (2026-08-03, RAON-RT 실운용 실증)
- [ ] L7N 문서 PDF 수령 (Phase 3 착수 시 — `docs/l7n/` 아직 없음)
- [ ] KV260 캐리어의 PS UART0 물리 라우팅 여부 (UART0 쓰려는 경우만 — 1순위 로그 수단은 trace buffer라 급하지 않음)
- [ ] RPU reserved-memory 최종 주소 (sudo로 dmesg/iomem에서 CMA 실배치 확인 후 확정 — §3 마지막 항목)
- [ ] GEM3 이관(Phase 2) 시점 — 사용자 결정 대기 (로봇 운용 중단 수반)
