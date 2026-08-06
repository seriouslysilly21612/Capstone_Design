# recovery/ — SD 사망 대비 오프보드 복구 자산 (2026-08-06)

보드 SD가 죽었을 때 "재플래시 → 이 저장소 clone"만으로 복구 재료가 전부 손에
들어오게 하는 디렉토리. **복원 절차의 정본은 백업 tar 안의 `state/RESTORE.md`**
(생성기: `tools/board_backup.sh`).

| 파일 | 정체 |
|---|---|
| `linux-image-...-kv260c-11_arm64.deb` (69 MB) | **프로덕션 RT 커널** (radix+zocl 픽스, DEBUG off) |
| `linux-headers-...-kv260c-11_arm64.deb` (8 MB) | 위 커널의 헤더 (IgH/ASIX 모듈 빌드용) |
| `board_backup_20260806_152004.tar.gz` (3.8 MB) | git에 없는 보드 로컬 상태 스냅샷 (INDY7.cfg·ready seed·/opt 3종·커널모듈·시스템 설정·Claude 메모리) |

## ⚠️ 리비전 함정 — 반드시 -11

`-10`과 `-11`은 **커널 버전 문자열(`uname -r`)이 동일**해서 헷갈리기 쉽다.
`-11`이 2026-07-20 ASIX 재빌드(`usbnet`/`mii` =m)이며, `-10`으로 복구하면
AX88179B USB NIC(= EtherCAT 경로)이 죽는다. 이 tar(08-06)의 내부 RESTORE.md에는
`-10`으로 잘못 적혀 있다 — **이 README가 정정본**이고, 다음 백업부터 tar도 맞는다.

## 갱신 정책

- **커널 deb**: 커널을 다시 빌드할 때만 교체 (드묾).
- **백업 tar**: 매 세션마다 커밋하지 말 것 (repo 비대화). 여기 있는 것은
  재해복구용 스냅샷 1개이며, **실기(로봇) 세션 뒤에는 갱신**하는 것이 안전하다 —
  `INDY7.cfg`는 앱 종료 시 엔코더 카운트를 써넣는 라이브 파일이라, 낡은 백업으로
  복원하면 E20/E24류(카운터/0점) 사고 위험이 있다. 갱신:
  ```bash
  sudo bash ~/ros2_ws/tools/board_backup.sh
  cp ~/board_backup_<최신>.tar.gz ~/ros2_ws/recovery/   # 옛 tar는 git rm
  ```

## 복구 개요 (상세는 tar 안 state/RESTORE.md 8단계)

1. Kria Ubuntu 22.04 플래시 → 2. **이 폴더의 -11 deb 설치**·재부팅 →
3. repo 2개 clone → 4. tar를 `/`에 복원 → 5. `depmod -a` + `flash-kernel`·재부팅 →
6. ROS2 Humble + apt(DPU 스택은 Xilinx PPA로 재설치 가능) + `colcon build` →
7. RAON 앱 `make` + ecat FIFO 설치 → 8. servo-off 스모크 → 인코더/HOME 확인.
