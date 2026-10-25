# 경로 변환·전역 진행거리 연결 결과 — 2026-10-25

[경로 변환 명세](PATH_CONVERSION_CONTRACT_2026-10-25.md)에 따라 `ctrl_control/ctrl_path_conversion.py`를 구현했다. 일반 경로, entry/return·chain 회피, local reference를 새 진행 관리자와 조향 계산에 전달할 수 있다.

## 구현

- `StationedPath`는 실제 경로 길이와 전역 station 라벨을 함께 보관하고 양방향으로 보간한다. 범위 밖 외삽은 거부한다.
- spawn 연결부의 음수 원점을 지원한다. local reference의 배열을 함께 검증·복사한다.
- 회피 전환의 시작·길이·초기 offset을 명시적으로 받고, 경계조건에 따른 5차 offset 프로파일을 새로 구현했다. chain과 entry/return을 같은 움직임 형식으로 처리한다.
- 회피 후 경로 길이를 다시 계산하면서 원래 전역 station 라벨을 유지한다. 잘못된/겹친 전환, 과도한 offset, 표본 수 초과, 진행 방향 반전을 거부한다.
- 기존 `Route`, `ReferenceLine`, 제어기나 회피 helper를 import하지 않는다. 새 조향/진행 모듈 자체는 이번에 수정하지 않았다.

offset 프로파일의 경계조건과 전체 XY 경로의 매끄러움을 구분한다. 표본 경로가 표현 가능하다는 검사이며, 도로 포함·차체 충돌·실제 조향 가능성까지 판정하지 않는다.

## 검증 결과

| 항목 | 결과 |
| --- | --- |
| 신규 경로 변환 검사 | 59 PASS |
| 기존 진행/조향 검사 | 162 PASS |
| 합계 | 221 PASS, 실패·오류·skip 0 |
| 변환→진행→조향 결합 합성 | 6개 장면, 5,550 step 모두 PASS |
| local↔global 거리 왕복 오차 | 합성에서 최대 약 9.24e-14m, 합격선 1e-8m |

| 장면 | 최대 오차 | 마지막 2초 RMS | 마지막 오차 |
| --- | ---: | ---: | ---: |
| 일반/음수 원점 연결 경로 | 0.80m | 0.00010m 이하 | 0.000064m 이하 |
| 좌/우 entry-return | 0.50m | 0.00782m 이하 | 0.00266m 이하 |
| 여러 번 이동하는 chain | 0.526m 이하 | 0.00693m 이하 | 0.00203m 이하 |
| 비균일 전역 라벨의 차로 변경 reference | 0.50m | 0.00669m 이하 | 0.00023m 이하 |

합성 오차의 정답은 직선·명시된 offset 다항식·코사인 차로 변경 곡선으로 계산했다. 진행 관리의 최대 전역 station 단조성, 양방향 거리 대응과 조향 제한도 통과했다. 검사 장면과 기준은 `tools/path_conversion_cases.json`, 기존 합성 차량 설정은 앞선 `lateral_cases.json`/`progress_cases.json`에 있다.

이는 조향 지연·미끄러짐·잡음·충돌 모델이 없는 합성 결과다. 실제 대회 경로 데이터, native ROS/catkin, MORAI 주행에서의 정확도와 성공률은 아직 확인하지 않았다.

## 재현과 다음 작업

저장소 루트 PowerShell에서:

```powershell
& .\ros1-port\.python310\python.exe -X utf8 -B .\ros1-port\tools\validate_path_conversion.py
```

매번 새 `verification/.work/independent-path-conversion-*` 폴더에 단위 log/XML, 매 step local/global station·오차 CSV, 소스 SHA256을 포함한 보고서를 생성한다. 기존 폴더를 출력 경로로 지정하면 중단한다.

이번 결과: `verification/.work/independent-path-conversion-20261025-030738Z-0b829c6d/`. 공유 요약은 [JSON 결과](path_conversion_result_2026-10-25.json)다. 기존 검증 결과·합격 기준·자산 manifest를 덮어쓰지 않았다.

다음은 **추종 요청 선택과 수명 관리**다. 기본/회피/reference 모드 충돌, 동일 reference ID의 변경, 명시적 해제, 오래된 요청과 유효한 전환 anchor를 처리한 뒤 기존 제어 출력에 영향을 주지 않는 병행 계산을 연결한다. 이번에는 production follower/launch 변경, 제어 송신, commit/push를 수행하지 않았다.
