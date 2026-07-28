# Ready-seed IK reach map — 2026-07-28

`RAON-RT-Revision` `kv260-merge` `a0a2c26` 산출물. 해석은 `docs/RAON-RT/merge.md` §7 **E28/E29**, §9.

생성:

```
cd ~/RAON-RT-Revision/App/Indy7 && make reachmap
cd ~/RAON-RT-Revision
./App/Indy7/bin/reachmap.out --step 0.01 --z 0.15,0.20,0.25,0.30,0.35 --out sweep.csv
python3 tools/reachmap_plot.py sweep.csv -o reachmap.png
```

앵커(q_ready) = `[0.465492650 −0.414549592 −2.140960980 0.168932482 0.732451146 −0.557864957]`
— 2026-07-28 16:34 기록분(`~/.indy7_ready_seed.good-20260728T1634`). **지도는 앵커의 함수라
다른 HOME으로 기록하면 경계가 움직인다** — CSV 헤더에 q_ready가 박혀 있는 이유.

| 파일 | 내용 |
|---|---|
| `reachmap.png` | z = 0.15/0.20/0.25/0.30/0.35 m, 1 cm 격자, 판정별 색 |
| `sweep_z015-035_1cm.csv.gz` | 위 원본 28,515 셀 |
| `mustard_zoom.png` | 머스타드 주변 1 mm 정밀 스윕 (z = 0.30) |
| `mustard_zoom_1mm.csv.gz` | 위 원본 14,061 셀 |
| `field_replay.csv` | 실기 로그 좌표 23점 재생 — 지도 ↔ 로봇 일치 확인용 |
| `field_targets.txt` | 그 입력 좌표 |

## 숫자

전체 스윕: PASS 27,132 / **BRANCH 1,345** / LIMITS 38 / **TILT 0 / NO_SOLUTION 0**.
박스 안에서 "멀어서 못 간다"는 경우는 없고, 거부는 전부 **브랜치 플립**이다.

머스타드 라인(y = 0.234, z = 0.30): x 0.813까지 연속 PASS → **x 0.814~0.846이 혼합 지대**
(1 mm 단위로 PASS/BRANCH 교대) → 이후 연속 BRANCH. 머스타드는 x 0.828 = 한가운데.
지터 내성별 최소 이동량: ±5 mm → 14 mm, ±10 mm → 19 mm, ±15 mm → 25 mm.

## ⚠️ 이 지도가 모르는 것

**충돌 판정이 전혀 없다.** RBDL에는 형상이 없어서 테이블·카메라 마운트·물체·로봇 자기 링크가
모두 미반영이다. `PASS`는 **"IK가 이 점을 받아준다"**이지 **"팔이 안전하게 갈 수 있다"가 아니다.**
그림의 base 마커·반경 점선·물체 별표는 전부 주석일 뿐 모델이 아니다. 경로 안전 검증은 MuJoCo 몫
(merge.md §2).
