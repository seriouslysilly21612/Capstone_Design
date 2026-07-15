# RPU FreeRTOS + SOEM EtherCAT Master 실행 계획

작성: 2026-07-07. 보드 실사(이 문서의 "보드 검증 상태" 참조) 기반으로 작성된 실행 계획.
목적: Pick & Place 시스템의 미구현 계층 — **RPU firmware + APU↔RPU bridge + 로봇 EtherCAT 제어** — 구축.
참조: `site_md/reference_02_openamp_freertos_ethernet.md` (Kria FreeRTOS/OpenAMP 공식 문서 링크),
랩 가이드 MAN-20241113-LX02H0001 (IgH 기반, 프로토콜 개념 참조용).

## 아키텍처 목표

```
┌────────────────── KV260 (ZynqMP) ──────────────────┐
│  APU (A53×4)                RPU (R5F-0, split)      │
│  Ubuntu 22.04 + ROS2        FreeRTOS 10 (Vitis BSP) │
│  perception 파이프라인       SOEM + 1kHz 제어 루프    │
│  apu_rpu_bridge_pkg  ⇄⇄⇄   OpenAMP rpmsg           │
│  (WiFi/USB Ethernet)        PS GEM3 ── RJ45 ────────┼── EtherCAT (Indy7 방향)
└─────────────────────────────────────────────────────┘
```

- APU: 기존 perception (RealSense→DPU→pick_target_base) 유지. `apu_rpu_bridge_pkg`(현재 placeholder)가 rpmsg로 목표점/궤적 전달.
- RPU: FreeRTOS + SOEM. GEM3를 전용으로 사용해 raw EtherCAT 프레임 송수신, 1kHz 이상 주기 제어.
- **주의**: kv260-smartcam DPU overlay와 RPU 사용은 독립적 — 충돌 없음 (PL vs RPU).

## 보드 검증 상태 (2026-07-07 실사)

| 항목 | 상태 |
|---|---|
| 커널 remoteproc 지원 | `zynqmp_r5_remoteproc.ko`, `rpmsg_char.ko`, `virtio_rpmsg_bus.ko` 모듈 존재 ✓ |
| device tree R5 노드 | **없음** — `/sys/class/remoteproc` 미존재. DT overlay 필요 (Phase 1 핵심 작업) |
| configfs DT overlay | 동작 확인 ✓ (`/sys/kernel/config/device-tree/overlays/`에 smartcam 로드됨, dfx-mgr active) |
| IPI mailbox | `/proc/device-tree/zynqmp-ipi` 존재 (mailbox@ff9905c0). RPU rpmsg용 IPI 채널은 overlay로 추가 필요 |
| Ethernet | GEM3(`ff0e0000`)만 okay, 드라이버 `macb`, eth0. GEM0-2 disabled |
| cmdline | `clk_ignore_unused` 이미 있음 ✓ (GEM을 Linux에서 떼어도 클럭 유지에 유리), `cma=1000M` |
| 부팅 | U-Boot + flash-kernel `image.fit` (GRUB 없음). boot.scr.uimg 존재 |
| **원격 접속 리스크** | 현재 Claude/SSH 접속이 **Tailscale over eth0**. GEM3 이관 전 대체 네트워크 필수 |

## 핵심 기술 판단 (사전 확정)

1. **SOEM에 공식 FreeRTOS+ZynqMP 포트 없음** → `osal`/`oshw` 계층을 FreeRTOS + `emacps`(Xilinx embeddedsw standalone GEM 드라이버) 위에 직접 포팅. 이것이 전체 개발량의 중심. SOEM은 TCP/IP 스택(lwIP) 불필요 — raw L2 프레임 send/recv만 있으면 됨.
2. **RPU 로드는 remoteproc** (Kria Ubuntu 표준 흐름). BOOT.BIN 수정은 Kria에선 비표준(부트 펌웨어가 QSPI에 고정, xmutil bootfw 관리) — 부팅 자동화는 **systemd 서비스로 remoteproc start** 방식 사용.
3. **R5는 split 모드, R5F-0 사용**. 코드/데이터는 DDR 예약 영역에 링크(SOEM+스택이 TCM 128KB 초과), 주기 태스크/ISR 등 지연 민감 코드만 TCM 배치.
4. **GEM3 이관은 2단계**: 개발 중엔 런타임 driver unbind(`echo ff0e0000.ethernet > /sys/bus/platform/drivers/macb/unbind`, 재부팅으로 복구 가능), 안정화 후 boot-time DT에서 disable.
5. **Vitis 버전: 2022.1** (classic IDE). 근거: kria-apps-docs FreeRTOS/OpenAMP 튜토리얼이 2022.1 기준, 보드 커널 5.15 세대와 일치, site_md 참조 문서들도 2022.1 흐름. CLAUDE.md 방침("최신보다 board-validated 버전") 부합. 빌드는 x86 PC에서 수행 (보드는 aarch64라 Vitis 실행 불가).

## 확정 사항 (2026-07-08 사용자 확인, 2차 업데이트)

- **작업 순서**: 이 RPU 계획은 **APU IgH 구현 완료 후** 본격 착수. 근거: Phase 2가 GEM3를 RPU로 가져가는데 APU IgH도 GEM3를 사용하므로 동시 진행 불가. 단, **Vitis 설치와 Phase 1(remoteproc hello)은 GEM3를 건드리지 않으므로 APU 작업과 병행 가능** — 틈틈이 미리 해두면 이전 시점에 Phase 2부터 바로 시작.
- **Indy7 접점**: STEP 우회, 드라이브 직결 확정. **PC에서 EtherCAT으로 Indy7 실제 제어에 성공한 코드 보유** → Phase 4는 신규 개발이 아니라 검증된 제어 로직의 RPU 이식. (착수 시 확인: 그 PC 코드가 어떤 master 기반인지 — SOEM 기반이면 application 계층 거의 그대로 이식됨)
- **테스트 드라이브**: **LS Mecapion L7N** (EtherCAT, CiA402) + 모터 + 전원 준비 완료. 관련 문서 PDF 제공 가능 → Phase 3 착수 시 ESI/PDO 매핑 자료로 수령.
- **UART 콘솔**: 시리얼 터미널 가능 ✓ (Gate 1 판정 수단 확보).
- **빌드 PC**: x86 PC 있음, **Vitis 미설치** → 계획서 기준 2022.1 설치가 첫 작업 (다운로드 ~100GB급, 반나절 이상 소요).
- **보드 접근**: 손 닿는 곳 ✓ → GEM3 이관 후에도 SD카드/전원/UART 직접 조작 가능. USB Ethernet은 필수 아님(편의용 선택).
- **그리퍼**: 미보유 — 범위에서 제외. Phase 5의 end-to-end 목표는 "말단부가 물체 바로 위에서 정지"까지.
- **제어 주기**: 1kHz로 추정, 추후 확정 예정 (변경 시 DC/지터 목표 조정).

---

## Phase 1 — remoteproc + FreeRTOS Hello World

**목표**: Ubuntu에서 R5F-0에 FreeRTOS elf를 로드/시작, UART(ttyPS) 로그 확인.

### PC 쪽 (x86, Vitis 2022.1)
1. Vitis 2022.1 설치, KV260 starter kit XSA 확보 (kria-apps-docs "Build Vitis platform" 또는 Vivado 2022.1 KV260 board preset에서 export).
2. Platform 생성: R5F-0 도메인, OS = `freertos10_xilinx`.
3. 템플릿 앱(FreeRTOS Hello World) 빌드. 링커 스크립트: DDR 예약 영역(아래 메모리 맵) 기준으로 수정.
4. 산출물 `rpu_hello.elf`를 보드 `/lib/firmware/`로 복사.

### 메모리 맵 (Xilinx OpenAMP 표준 배치 준수)
```
0x3ed00000 – 0x3ed3ffff : rproc 예약 (vring/공유버퍼, rpmsg용)
0x3ed40000 – 0x3efFffff : R5 코드/데이터 (약 2.75MB, 필요시 확장)
TCM ATCM (R5 view 0x0, APU view 0xffe00000) : 벡터테이블 + 지연민감 코드
```
CMA(0x… cma=1000M)와 겹치지 않는지 `/proc/iomem`으로 확인 후 확정.

### 보드 쪽
1. **R5 remoteproc DT overlay 작성** (`rpu_rproc.dtso`): `xlnx,zynqmp-r5-remoteproc` 노드(cluster-mode=split), reserved-memory 노드들, RPU용 IPI mailbox 노드. 레퍼런스: Xilinx wiki "OpenAMP" + kria-apps-docs OpenAMP landing (site_md 02 링크).
2. 컴파일: `dtc -@ -O dtb -o rpu_rproc.dtbo rpu_rproc.dtso`
3. 적용(런타임, smartcam과 같은 방식):
   ```bash
   sudo mkdir /sys/kernel/config/device-tree/overlays/rpu
   sudo sh -c 'cat rpu_rproc.dtbo > /sys/kernel/config/device-tree/overlays/rpu/dtbo'
   sudo modprobe zynqmp_r5_remoteproc
   ls /sys/class/remoteproc/   # remoteproc0 등장해야 함
   ```
4. 실행:
   ```bash
   echo rpu_hello.elf | sudo tee /sys/class/remoteproc/remoteproc0/firmware
   echo start | sudo tee /sys/class/remoteproc/remoteproc0/state
   ```

**Gate 1**: UART로 FreeRTOS 태스크 로그 출력, `stop`/`start` 반복 안정.
**디버깅 순서**: overlay 적용 오류(dmesg) → 모듈 probe 실패(reserved-memory/IPI 불일치) → elf 로드 실패(링커 주소 vs 예약 영역) → 실행 후 무반응(UART 설정, 벡터테이블).

## Phase 2 — GEM3 이관 + raw Ethernet 송수신

**목표**: RPU가 GEM3로 raw L2 프레임 송수신.

1. **선행: 대체 네트워크 확보** — USB Ethernet 어댑터 또는 WiFi를 먼저 올리고 Tailscale이 그 경로로 붙는지 확인. **eth0 unbind 전에 반드시 완료** (원격 세션 단절 방지).
2. Linux에서 GEM3 분리(개발 단계, 가역적):
   ```bash
   echo ff0e0000.ethernet | sudo tee /sys/bus/platform/drivers/macb/unbind
   ```
3. Vitis에서 `emacps` interrupt 예제 기반 테스트 앱: PHY = TI **DP83867** 초기화(embeddedsw emacps 예제에 포함), 1G 링크업.
4. EtherType 0x88A4 테스트 프레임 송신 → PC Wireshark로 수신 확인, 반대 방향도 확인.
5. **함정 대비**:
   - **DMA 캐시 일관성**: BD ring/버퍼 영역을 R5 MPU로 non-cacheable 설정하거나 flush/invalidate 일관 처리 (최다 삽질 포인트).
   - GEM3 인터럽트(IRQ 95)가 RPU GIC로 수신되는지 확인.
   - 클럭: `clk_ignore_unused`가 이미 있으나, unbind 후 GEM 클럭이 꺼지면 Linux CCF가 원인 — 필요시 boot-time DT에서 disable로 전환.

**Gate 2**: RPU ↔ PC 간 임의 L2 프레임 왕복 성공.

## Phase 3 — SOEM 포팅

**목표**: SOEM `simple_test`가 RPU에서 슬레이브 스캔, OP 도달.

1. SOEM v1.4.x 소스 트리를 Vitis 프로젝트에 편입.
2. `osal/freertos/` 작성: `osal_usleep`(vTaskDelay + 미세지연은 TTC 폴링), `osal_current_time`(TTC 기반 µs 시계), 뮤텍스(FreeRTOS 세마포어).
3. `oshw/freertos/nicdrv.c` 작성: Phase 2의 emacps 코드 래핑 — `hw_open/close`, send/recv, redundancy 미사용. 초기 구현은 **수신 폴링**으로 단순화, 동작 후 인터럽트+큐로 전환.
4. `simple_test` 이식 → 테스트 슬레이브 대상 PREOP→SAFEOP→OP 확인.

**Gate 3**: 슬레이브 OP 도달 + PDO 데이터 정상 read.

## Phase 4 — 1kHz 주기 제어 + DC + Indy7 프로토콜

1. TTC 인터럽트 기반 1kHz 태스크에서 `ecx_send/receive_processdata`.
2. Distributed Clocks 설정, SYNC0 정렬.
3. R5 사이클 카운터로 주기 지터 측정/로깅 → **APU IgH(RT-PREEMPT) 대비 성능 근거 데이터** (목표: 지터 수 µs 이하).
4. Indy7 접점 확인 결과에 따라: CiA402 상태머신(직결 드라이브) 또는 별도 프로토콜 구현.

**Gate 4**: 1kHz 사이클에서 지터 목표 달성 + 실제 축 1개 이상 위치 제어.

## Phase 5 — APU↔RPU bridge (`apu_rpu_bridge_pkg`)

1. RPU 펌웨어에 OpenAMP rpmsg endpoint 추가 (Vitis echo-test 예제 기반).
2. Linux: `rpmsg_char`로 `/dev/rpmsg0` 노출 확인.
3. 메시지 포맷 정의: APU→RPU (목표 관절값/궤적 세그먼트, watchdog), RPU→APU (관절 상태, EtherCAT 상태, 오류코드). 고정 크기 C struct, 버전 필드 포함.
4. `apu_rpu_bridge_pkg`(현재 placeholder) 구현: `/pick_target_base` 구독 → rpmsg 송신, RPU 상태 → ROS2 토픽 발행.
5. 부팅 자동화: DT overlay 적용 + remoteproc start + bridge 노드를 systemd/launch에 편입 (smartcam.service와 동일 패턴).

**Gate 5**: perception → pick target → RPU → 로봇 모션까지 end-to-end 1회 성공.

## 리스크 요약

| 리스크 | 대응 |
|---|---|
| GEM 이관 시 원격 세션 단절 | Phase 2 선행 조건으로 대체 네트워크 + Tailscale 경로 확인 명시 |
| DT overlay/reserved-memory 오류로 부팅 문제 | 런타임 configfs overlay로만 작업(재부팅=복구), boot-time 반영은 안정화 후 |
| emacps DMA 캐시 이슈 | non-cacheable MPU region으로 시작, 성능은 이후 최적화 |
| SOEM 포팅 난항 | Phase 2에서 raw 송수신 완전 검증 후 착수, 폴링→인터럽트 단계화 |
| Indy7 인터페이스 불확실 | Phase 1–3은 범용 슬레이브로 진행, Phase 4 전 랩 확인 |
| APU IgH 작업과의 관계 | APU에서 검증한 제어 로직(CiA402 등)을 RPU로 이식 — 이 계획과 상호보완 |
