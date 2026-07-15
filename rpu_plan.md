# RPU 구현 계획 — FreeRTOS + SOEM EtherCAT Master

작성: 2026-07-08. 지금까지의 논의(보드 실사 + 사용자 확인 사항)를 종합한 논리 전개 중심의 마스터 계획.
명령어 수준의 세부 절차는 `rpu_freertos_soem_execution_plan.md` 참조. 이 문서는 **무엇을, 어떤 순서로, 왜** 하는지를 설명한다.

---

## 1. 목표와 전제

**최종 목표**: Pick & Place 시스템에서 로봇 제어 계층을 완성한다.
perception(APU, 기존 완성) → pick 좌표 → **RPU의 1kHz 실시간 EtherCAT 제어** → Indy7 말단부가 물체 바로 위에서 정지 (그리퍼는 범위 제외).

**왜 RPU인가** — 원리부터:
- EtherCAT 모션 제어는 매 사이클(1ms)마다 프레임을 정해진 시각에 보내야 한다. 이 주기가 흔들리는 정도(jitter)가 제어 품질을 결정한다.
- APU의 Linux는 범용 OS라서 스케줄러, 인터럽트, 캐시 경쟁 때문에 수십~수백 µs의 jitter가 생길 수 있다. RT-PREEMPT 패치로 개선해도 수십 µs 수준.
- RPU(Cortex-R5F)는 OS 간섭 없이 코어 전체를 제어 루프에 전용할 수 있고, TCM(지연 고정 메모리)까지 갖춰 **수 µs 이하 jitter**가 가능하다. 이것이 이 작업의 존재 이유다.

**확정된 전제 (2026-07-08 기준)**:
| 항목 | 상태 |
|---|---|
| 작업 순서 | **APU IgH 구현 완료 후 착수** (GEM3 포트가 하나뿐이라 동시 사용 불가). 단 Phase 0~1은 GEM3 무관 → 병행 가능 |
| 테스트 드라이브 | LS Mecapion **L7N** (EtherCAT, CiA402) + 모터 + 전원 보유. 문서 PDF 제공 가능 |
| Indy7 | STEP controller 우회, 드라이브 직결. **PC에서 EtherCAT 제어 성공한 코드 보유** → Phase 4는 이식 작업 |
| 빌드 PC | x86 PC 있음, Vitis 미설치 (2022.1 설치 예정) |
| 보드 환경 | 손 닿는 곳, UART 시리얼 콘솔 가능. GEM3 이관 후 SSH 단절 감수 가능 |
| 제어 주기 | 1kHz 추정 (추후 확정) |

**보드 실사 결과 (2026-07-07)**:
- 커널에 `zynqmp_r5_remoteproc.ko`, `rpmsg_char.ko` 등 모듈 존재 — 소프트웨어 인프라 절반은 준비됨
- device tree에 R5 노드 없음 → **DT overlay 추가가 Phase 1의 핵심**
- configfs DT overlay 동작 확인 (kv260-smartcam이 이 방식으로 로드 중) → 같은 방법 사용, 재부팅 = 복구라서 안전
- GEM3(`ff0e0000`, macb, eth0)가 유일한 활성 이더넷. cmdline에 `clk_ignore_unused` 있음(이관에 유리)
- 현재 원격 접속(Tailscale)이 eth0 경유 → GEM3 이관 시 단절 (보드 직접 접근으로 대체)

---

## 2. 목표 아키텍처

```
┌────────────────── KV260 (ZynqMP) ──────────────────┐
│  APU (A53×4)                RPU (R5F-0, split)      │
│  Ubuntu 22.04 + ROS2        FreeRTOS 10             │
│  perception 파이프라인       SOEM + 1kHz 제어 루프    │
│  apu_rpu_bridge_pkg  ⇄⇄⇄   OpenAMP rpmsg           │
│                             PS GEM3 ── RJ45 ────────┼── L7N → Indy7 드라이브 체인
└─────────────────────────────────────────────────────┘
```

**세 개의 경계를 분리해서 본다** (site_md reference_01 원칙):
1. PL ↔ APU: DPU 비전 — 기존 완성, 이 계획과 무관 (RPU와 충돌 없음)
2. APU → RPU: OpenAMP rpmsg — 목표점 전달 + 상태 회신 (Phase 5)
3. RPU → 외부: EtherCAT raw frame over GEM3 (Phase 2~4)

**설계 원칙**: 실시간이 필요한 것(1kHz PDO 사이클, CiA402 상태머신, 궤적 보간)은 전부 RPU 안에서 닫는다. APU↔RPU 경계는 "다음 목표점"과 "상태 보고"만 넘는다. 그래서 APU가 아무리 바빠도 제어 주기는 무관하다.

---

## 3. 핵심 난제 이해 — "SOEM 포팅"이 정확히 무엇인가

SOEM 소스는 세 부분으로 나뉜다:

```
[application]  simple_test, 제어 로직        ← 순수 C, 수정 불필요
[SOEM core]    EtherCAT 프로토콜 로직        ← 순수 C, 수정 불필요
[osal]         OS 추상화 (시간, sleep, mutex) ← ★ 플랫폼별 구현 필요
[oshw]         NIC 접근 (raw frame 송수신)    ← ★★ 플랫폼별 구현 필요
```

**Linux에서는 왜 포팅이 필요 없나**: `osal`은 POSIX(usleep, gettimeofday)로, `oshw`는 raw socket 한 줄로 끝난다. 커널이 NIC 드라이버, DMA, 버퍼 관리, PHY 협상을 전부 대신해주기 때문이다.

**FreeRTOS/R5에는 그 커널 서비스가 전부 없다.** 포팅 = 커널이 해주던 일을 직접 하는 것:

| 작업 | Linux에서 | RPU에서 직접 해야 하는 것 | 난이도 |
|---|---|---|---|
| 프레임 송수신 | raw socket | GEM MAC 레지스터/DMA descriptor ring 직접 관리 (emacps 드라이버) | 중 |
| **DMA 캐시 일관성** | 커널이 처리 | MPU로 non-cacheable 영역 설정 또는 수동 flush/invalidate | **상 (최대 난제)** |
| PHY 링크업 | 커널이 처리 | DP83867 MDIO 설정, autonegotiation, RGMII delay | 중 |
| 인터럽트 | 커널이 처리 | RPU GIC에 GEM IRQ 등록/핸들러 작성 | 중 |
| µs 단위 시간 | gettimeofday | FreeRTOS tick은 1ms 해상도뿐 → TTC 하드웨어 타이머로 µs 시계 구현 | 중 |
| 리소스 인계 | 해당 없음 | Linux가 쓰던 GEM3를 클럭/전원 살린 채 넘겨받기 | 중 |

**왜 "힘들다"고 알려져 있는가** — 실체는 DMA 캐시 일관성이다:
- GEM MAC은 DMA master라서 CPU 캐시를 거치지 않고 DRAM을 직접 읽고 쓴다.
- CPU가 보낼 프레임을 버퍼에 썼는데 그 내용이 캐시에만 있으면 → DMA는 DRAM의 **옛날 데이터**를 전송한다.
- DMA가 수신 프레임을 DRAM에 썼는데 CPU 캐시에 옛 데이터가 남아 있으면 → CPU는 **옛날 데이터**를 읽는다.
- 증상이 최악이다: 간헐적으로만 프레임이 깨지고, printf를 넣으면 (타이밍이 바뀌어) 증상이 사라진다. 원인을 모르면 몇 주를 소모하는 유형의 버그.
- **대응**: 처음부터 DMA 버퍼/descriptor 영역을 R5 MPU로 non-cacheable 설정 → 이 문제 자체를 차단하고 시작. 성능 최적화(캐시+수동 flush)는 동작 확인 후.

**왜 그래도 관리 가능한가**:
1. 구현할 인터페이스가 작고 명확하다 — `oshw`는 함수 5개 내외(open/close/send/recv), `osal`은 10개 내외의 소함수. SOEM core는 한 줄도 안 고친다.
2. 어려운 부분(DMA/캐시/PHY)은 Xilinx가 제공하는 emacps 예제로 **SOEM 없이 먼저 검증**한다 (Phase 2). SOEM을 얹는 Phase 3 시점에는 하드웨어 불확실성이 이미 제거된 상태다.
3. bare-metal/RTOS SOEM 포팅은 커뮤니티 선례가 다수 있는 정형화된 작업이다.

요약: **포팅의 난이도 80%는 Phase 2(하드웨어 계층)에 있고, Phase 3(SOEM 자체)은 20%다.** 계획이 Phase 2와 3을 분리한 이유가 바로 이것이다.

---

## 4. 왜 이 순서인가 — 불확실성 제거의 원리

각 Phase는 다음 Phase의 전제가 되는 불확실성을 하나씩 제거한다. 순서를 바꾸면 문제가 생겼을 때 원인 후보가 여러 계층에 걸쳐 있어 디버깅이 불가능해진다.

```
Phase 0  개발 환경          "빌드하고 볼 수 있는가"
   ↓
Phase 1  remoteproc+FreeRTOS "RPU에 코드를 올리고 반복(빌드→로드→로그)할 수 있는가"
   ↓
Phase 2  GEM3 + raw frame    "RPU가 이더넷 하드웨어를 소유할 수 있는가"  ← 최대 난제 격리
   ↓
Phase 3  SOEM 포팅           "EtherCAT 프로토콜이 도는가"
   ↓
Phase 4  1kHz DC + CiA402    "실시간 모션 제어가 되는가" (L7N → Indy7 코드 이식)
   ↓
Phase 5  rpmsg + ROS2 통합   "전체 시스템이 이어지는가"
```

예: Phase 2를 건너뛰고 SOEM을 바로 올렸는데 슬레이브가 안 잡히면 — 원인이 PHY인지, DMA 캐시인지, SOEM 포팅 버그인지, 슬레이브 설정인지 구분할 방법이 없다. Phase 2를 통과한 상태라면 후보가 SOEM 계층으로 좁혀진다.

---

## 5. Phase별 상세

### Phase 0 — 개발 환경 (APU IgH 작업과 병행 가능)

**무엇을**: x86 PC에 Vitis 2022.1 설치. KV260 XSA 확보, R5F-0 도메인 + `freertos10_xilinx` BSP 플랫폼 생성. 보드 UART 시리얼 터미널 확인.

**왜**: 이후 모든 RPU 코드는 x86에서 크로스컴파일된다(보드는 aarch64라 Vitis 실행 불가). 버전을 2022.1로 고정하는 이유: kria-apps-docs의 FreeRTOS/OpenAMP 튜토리얼이 2022.1 기준이고, 보드 커널(5.15) 세대와 일치하며, CLAUDE.md 방침("최신보다 board-validated")에 부합.

**완료 기준**: Vitis에서 R5 hello world elf가 빌드됨.

### Phase 1 — remoteproc으로 FreeRTOS 실행 (APU IgH 작업과 병행 가능)

**무엇을**:
1. R5 remoteproc **DT overlay** 작성: R5 클러스터 노드(split 모드), reserved-memory(R5 코드/데이터 + rpmsg vring 영역), IPI mailbox 노드
2. configfs로 overlay 적용(smartcam과 동일 방식) → `zynqmp_r5_remoteproc` 모듈 probe → `/sys/class/remoteproc0` 등장
3. FreeRTOS hello elf를 `/lib/firmware`에 두고 start/stop

**왜**: 개발 반복 사이클(빌드→로드→실행→로그→수정)이 없으면 이후 어떤 작업도 불가능하다. 이것이 모든 Phase의 기반 도구다.
- device tree가 필요한 이유: Linux 커널은 하드웨어 존재를 device tree로만 안다. R5 노드가 없으면 remoteproc 드라이버가 붙을 대상이 없다. overlay는 "기존 트리에 노드를 런타임 추가"하는 메커니즘이고, configfs 방식은 재부팅하면 사라지므로 실험이 안전하다.
- 링커 스크립트를 DDR 예약 영역 기준으로 작성 (SOEM+제어 코드가 TCM 128KB를 초과하므로). 벡터테이블/지연민감 코드만 TCM.

**완료 기준 (Gate 1)**: UART로 FreeRTOS 태스크 로그 확인, start/stop 반복 안정.
**막히면 보는 순서**: dmesg의 overlay 오류 → reserved-memory/IPI 불일치로 probe 실패 → elf 링크 주소 vs 예약 영역 불일치 → UART 번호/설정 (Linux가 ttyPS1 사용 중 — RPU는 다른 UART 또는 공유 주의).

### Phase 2 — GEM3 이관 + raw Ethernet 송수신 ★ 최대 난제 구간

**전제**: APU IgH 작업 완료 (GEM3를 넘겨받는 시점). 시간 배분의 ~40%를 여기에.

**무엇을**:
1. Linux에서 GEM3 분리: `echo ff0e0000.ethernet > /sys/bus/platform/drivers/macb/unbind` (가역적 — 재부팅하면 원상복구. 안정화 후에만 boot-time DT disable로 전환)
2. Vitis emacps 예제 기반으로: DP83867 PHY 링크업 → DMA descriptor ring 구성 → 프레임 송수신
3. **DMA 버퍼/descriptor를 R5 MPU로 non-cacheable 설정** (캐시 문제 원천 차단)
4. EtherType 0x88A4 테스트 프레임을 PC와 왕복, Wireshark로 검증

**왜**: 3장에서 설명한 포팅 난이도의 80%(DMA 캐시, PHY, 인터럽트, 리소스 인계)를 SOEM이라는 변수 없이 격리 검증한다. 여기서 raw 프레임이 완벽히 왕복하면, 이후 EtherCAT이 안 될 이유는 프로토콜 계층뿐이다.

**완료 기준 (Gate 2)**: RPU↔PC 간 임의 L2 프레임 왕복 (수천 회 무손실).
**막히면 보는 순서**: PHY 링크 LED/MDIO 레지스터 → 송신은 되는데 수신 실패(인터럽트 라우팅) → 간헐적 데이터 깨짐(캐시 — MPU 설정 재확인) → 클럭(unbind 후 CCF가 껐는지, `clk_ignore_unused` 확인).

### Phase 3 — SOEM 포팅

**무엇을**:
1. SOEM v1.4.x를 Vitis 프로젝트에 편입
2. `osal` 구현: `osal_usleep`(tick 단위는 vTaskDelay, sub-ms는 TTC 폴링), `osal_current_time`(TTC 기반 µs 시계), mutex(FreeRTOS 세마포어)
3. `oshw` 구현: Phase 2의 emacps 코드를 SOEM 인터페이스(hw_open/send/recv)로 래핑. 초기엔 수신 폴링(단순) → 동작 후 인터럽트+큐
4. `simple_test`로 L7N 스캔

**왜**: Phase 2가 끝난 지금, 이 작업은 "정해진 인터페이스 15개 남짓을 이미 검증된 코드로 채우는" 작업으로 축소되어 있다. µs 시계를 TTC로 만드는 이유: FreeRTOS tick(1ms)으로는 SOEM의 프레임 타임아웃(수백~수천 µs 단위)을 정확히 잴 수 없다.

**완료 기준 (Gate 3)**: L7N이 PREOP→SAFEOP→OP 도달, PDO 데이터 정상 read. (이 시점에 L7N 문서 PDF를 `~/ros2_ws/docs/l7n/`에 수령)

### Phase 4 — 1kHz 주기 제어 + DC + CiA402 모션

**무엇을**:
1. TTC 인터럽트 기반 1kHz 태스크: `ecx_send_processdata` → 연산 → `ecx_receive_processdata`
2. Distributed Clocks 설정, SYNC0에 사이클 정렬
3. R5 사이클 카운터로 jitter 측정·로깅 → **APU(RT-PREEMPT+IgH) 대비 정량 비교 데이터** 확보
4. L7N으로 CiA402 상태머신 + CSP 모드 실축 제어
5. **보유한 Indy7 PC 제어 코드 이식** (착수 시 확인: 어떤 master 기반인지 — SOEM이면 application 계층 거의 그대로)

**왜**: EtherCAT 모션의 품질은 "주기의 정확성"이 전부다. DC는 master와 slave들의 시계를 맞춰 slave가 정확한 시각에 동작하게 하는 메커니즘이고, jitter 측정은 이 프로젝트가 APU 방식보다 나음을 증명하는 근거 데이터가 된다. L7N으로 먼저 하는 이유: 이미 검증된 단순 대상에서 master 쪽 문제를 다 잡은 뒤, 6축 Indy7 체인이라는 복잡도를 추가한다.

**완료 기준 (Gate 4)**: 1kHz에서 jitter 수 µs 이하 + L7N 실축 CSP 위치 제어 + (이후) Indy7 다축 제어.

**1ms 사이클 타이밍 예산 (2026-07-08 분석)** — 1kHz는 이 아키텍처에 여유가 큰 목표다:
| 항목 | 예상 소요 |
|---|---|
| EtherCAT 프레임 왕복 (100BASE-TX, 6~7 slave 체인, LRW ~200-300B) | ~50–100µs |
| 제어 연산 (6축 보간 + CiA402) | ~10–50µs |
| SOEM 처리 + non-cached 버퍼 접근 오버헤드 | ~수십 µs |
| **합계** | **<200µs (예산 1000µs의 20% 미만, 여유 5배+)** |

jitter 원천: R5 타이머 ISR(TCM 배치 시 ~1µs급) + DDR 경합(APU/DPU와 공유 — 임계 경로를 TCM에 두면 µs 미만 변동). 주기가 2kHz로 확정돼도 무난, 4kHz(250µs)면 수신 인터럽트화·캐시 최적화 필요하나 가능 범위.
실시간성의 실제 리스크는 성능이 아니라: ①Phase 2 bring-up 지연(일정 리스크) ②DC 동기 규율(SOEM 마스터 사이클을 DC 기준시계에 PI 제어로 정렬 — 미흡 시 드라이브 sync error(AL status 코드)로 드러남, 해법 정형화되어 있음).

### Phase 5 — APU↔RPU bridge + 시스템 통합

**무엇을**:
1. RPU 펌웨어에 OpenAMP rpmsg endpoint 추가 (Vitis echo-test 예제 기반)
2. Linux `rpmsg_char`로 `/dev/rpmsg0` 확인
3. 메시지 포맷 정의: APU→RPU(목표 관절값/궤적 세그먼트 + watchdog), RPU→APU(관절 상태, EtherCAT 상태, 오류코드). 고정 크기 C struct + 버전 필드
4. `apu_rpu_bridge_pkg`(현재 placeholder) 구현: `/pick_target_base` 구독 → IK/궤적 → rpmsg 송신, RPU 상태 → ROS2 토픽
5. 안전 로직: rpmsg watchdog timeout 시 RPU 자체 감속 정지, EtherCAT 오류 시 Quick Stop 후 APU 보고
6. 부팅 자동화: overlay 적용 + remoteproc start + bridge를 systemd에 편입 (smartcam.service 패턴)

**왜**: rpmsg를 쓰는 이유는 Xilinx가 지원하는 표준 APU↔RPU 채널이고, 목표점 전달(수십 Hz, 비실시간)에는 충분한 대역폭이기 때문. 실시간 루프는 이미 RPU 안에서 닫혀 있으므로 이 경계의 지연은 제어 품질과 무관하다. watchdog을 RPU에 두는 이유: APU가 죽어도 로봇은 안전하게 멈춰야 한다.

**통신 방식에 대한 설계 노트 (2026-07-08 논의)**: RPMsg 자체가 내부적으로 shared memory(vring) + IPI 인터럽트 + 프로토콜 계층이다. 프로토콜 계층 없이 raw shared memory로 직접 주고받는 것도 가능하며(예약 DDR 영역 + 양측 non-cacheable 매핑 + seqlock/double buffer), 성능이 필요하면 표준 패턴은 **hybrid**다:
- **제어면(RPMsg)**: 명령, 설정, 오류 이벤트 — 메시지 경계·채널·유실 감지가 필요한 low-rate 데이터
- **데이터면(raw shm)**: 1kHz 관절 상태/목표점 — "최신값만 의미 있는" 데이터라 큐 불필요, RPU는 어차피 1kHz 루프에서 읽으므로 인터럽트도 불필요(polling)
단, 본 시스템의 데이터량(관절 6축 상태/목표 = 수백 바이트, 목표점 수십 Hz)은 RPMsg 용량(기본 버퍼 512B, 페이로드 ~496B/메시지) 안에 들어오므로 **Phase 5 초기 구현은 RPMsg 단독으로 시작**하고, 실측에서 병목/지연이 확인될 때만 shm 데이터면을 추가한다(예약 영역은 처음부터 shm 공간을 포함해 잡아두면 추후 무변경 확장 가능). Linux 쪽 shm 접근은 UIO 권장 — cmdline에 `uio_pdrv_genirq.of_id=generic-uio`가 이미 있어 overlay에 generic-uio 노드만 추가하면 mmap 접근이 된다.

**완료 기준 (Gate 5)**: perception → pick 좌표 → RPU → **Indy7 말단부가 물체 바로 위 정지**까지 end-to-end 1회 성공.

---

## 6. 리스크 요약

| 리스크 | 대응 |
|---|---|
| DMA 캐시 일관성 (최대 난제) | non-cacheable MPU 영역으로 시작해 원천 차단, 최적화는 후순위. Phase 2에 시간 40% 배정 |
| DT overlay 실수로 부팅 이상 | configfs 런타임 방식만 사용(재부팅=복구), boot-time 반영은 안정화 후 |
| GEM3 이관 시 SSH/Tailscale 단절 | 보드 직접 접근 + UART 콘솔로 작업 (사용자 확인: 감수 가능). 편의용 USB Ethernet은 선택 |
| µs 타이밍 오류로 SOEM 타임아웃 오동작 | osal 시계를 TTC 하드웨어 타이머로 구현, tick 의존 금지 |
| Indy7 이식 코드가 타 master 기반일 가능성 | Phase 4 착수 시 확인, IgH/TwinCAT 기반이면 SOEM API로 번역 계층 추가 |
| 제어 주기 미확정 (1kHz 추정) | 구조는 주기 무관, 확정 시 DC/jitter 목표만 조정 |

## 7. 남은 확인 항목

- [ ] Indy7 PC 제어 코드의 master 종류 (Phase 4 전)
- [ ] 제어 주기 확정 (Phase 4 전)
- [ ] L7N 문서 PDF 수령 → `~/ros2_ws/docs/l7n/` (Phase 3 착수 시)

## 8. 참고 자료

- 명령어 수준 절차·메모리 맵: `rpu_freertos_soem_execution_plan.md`
- Kria FreeRTOS/OpenAMP 공식 문서 링크 모음: `site_md/reference_02_openamp_freertos_ethernet.md`
- 보드/아키텍처: `site_md/reference_01_kria_core_architecture.md`
- SOEM 소스: https://github.com/OpenEtherCATsociety/SOEM (v1.4.x)
- 랩 가이드 MAN-20241113-LX02H0001 (IgH 기반 — 프로토콜/드라이브 설정 개념 참조)
