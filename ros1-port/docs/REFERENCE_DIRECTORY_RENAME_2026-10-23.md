# 비교 원본 디렉터리 이름 변경 — 2026-10-23

## 목표·범위·방법

사용자가 선택한 `2026_CTRL_competition/`을 현재 비교 원본 경로로 통일한다.
기존 이름은 `2026foscar-competition/`이었다. 과거 자료 통합 작업의 후속으로
`cleanup/archive-legacy-20261023` 브랜치에서 진행했다.

영향 파일은 AGENTS·루트 README, ros1-port/tools의 비교·패키징·회귀 도구,
Docker의 실행·GPU 검사·격리 비교 준비 도구, verification의 실행 Python,
run_competition.sh 주석, 현재 개발 계획·문서 분류·ROS1 기술 안내,
과거자료/README의 복원 안내다. 문자열 경로를 바꾸며 원본의 내용과
production 코드·토픽·파라미터·주행 정책은 바꾸지 않는다.

회귀 래퍼는 커밋 전에도 Git 추적 목록의 옛 원본 경로를 새 경로로 해석해
원본 파일 보호 검사를 수행한다. 커밋 이후에는 새 경로를 직접 사용한다.
과거 날짜별 기록·기존 JSON/SHA256 manifest·보관 원문에는 당시 이름이 남는다.
resource_hashes.json과 source-manifest.json의 reference 메타데이터도 당시
출처 표시로 유지한다. 새 기준을 기록하거나 해시 기대값을 바꾸지 않았다.
이전 ARCHIVE_CONSOLIDATION 기록의 경로 불일치는 이 후속 변경으로 해소했다.

## 검증·완료 기준

새 경로를 실제로 찾는지 검사하고 현재 링크가 해석되며 비교 원본의 모든
파일 내용이 불변이면 경로 변경을 완료한다. 오프라인 회귀 실패는 별도로 기록한다.

- 변경 Python 구문·변경 shell의 `bash -n`: PASS.
- port_boundary/preflight/generate_packaging/check_model_equivalence/check_preservation의
  원본 경로 5개 import·존재 확인: PASS.
- MARKDOWN_INDEX의 로컬 링크: 누락 0.
- 비교 원본 530개 파일 이동 후 작업 전후 SHA256·파일 목록 동일: PASS.
- `git diff --check`: PASS.

실행 명령:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 ros1-port/tools/run_refactor_checks.py
git diff --check
```

Linux Python으로 실행했다. Windows 회귀 성공을 의미하지 않는다.
ROS1/catkin/native 메시지/TF/MORAI·GPU 추론·실제 제어 송신은 미실행이다.
Docker 실행은 shell 구문·원본 마운트 경로 확인 범위이며 컨테이너를 기동하지 않았다.
커밋·push·merge는 수행하지 않았다.

## 이번 오프라인 회귀 결과

새 출력: `verification/.work/identifier-refactor-20261023-123717/`.
`summary.json`의 scope 문구는 기존 래퍼의 Windows 고정 문구이며, 이번 실행은
Linux Python 3.8이다. Windows에서 실행했다고 해석하지 않는다.

| 검사 | 결과 |
| --- | --- |
| Python 구문 | 275개 PASS |
| ROS1 adapter 단위 | 27 PASS |
| 원본/이식본 개별 알고리즘 시험 | 원본 909 PASS, 이식본 944 PASS |
| 원본/이식본 차등 추적 비교 | FAIL, 추적값·시험 수 차이. 개별 시험 통과와 구분 |
| 모델 동등성 | PASS |
| packaging 단위 | 7 PASS / 1 ERROR. yolo_stoplines.json SHA256·behavior.yaml 크기 불일치 |
| identifier AST | FAIL, ground.py가 예전 식별자 정리 범위를 넘어 다름 |
| packaging 정적 | FAIL, 원본/이식본 메시지 바이트 불일치 |
| boundary/preflight | FAIL, 현재 구현과 옛 원본 변환·메시지 기준 불일치 |
| preservation | FAIL, 기존 source-manifest와 현 원본 스냅샷 불일치 |
| 실행 중 보호 파일 변경 | 633개 중 0개 |

실패 로그에서 옛 폴더를 찾지 못하는 오류는 없었고 새 비교 원본을 사용했다.
이름 변경 작업에서 production과 원본 바이트를 바꾸지 않았으며 위 실패를
해소하기 위한 기대값·해시·코드 수정은 수행하지 않았다. 전체 래퍼 종료코드는 1이다.
