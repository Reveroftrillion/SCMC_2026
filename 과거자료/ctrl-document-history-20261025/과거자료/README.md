# 과거 자료

2026-10-23에 루트의 과거 개발·계획·참고 자료를 이곳으로 모았다.
현재 개발·실행은 [ros1-port](../ros1-port/README.md), 비교 기준 원본은
`../2026_CTRL_competition/`이다. 현재 계획은
[DEVELOPMENT_WORKFLOW.md](../ros1-port/docs/DEVELOPMENT_WORKFLOW.md)를 따른다.

## 보관 내용

- `2026foscar/`, `2026foscar-team2/`, `2026foscar-records/`: 과거 ROS2 개발본·고유 시험·기록.
- `sim_2025/`: 이전 외부 참고 구현.
- `tunnel_tools/`, `측정테스트파일/`: 과거 분석·측정 도구.
- `Readme, 각종정보파일/`: 제공 자료·지도·초기 사양·계획.
- `개발계획서.md`, `할일.md`, `남은 할일.md`: 과거 계획. 미완료 체크박스는 현재 할 일이 아니다.
- `HANDOFF_터널_2026-10-12.md`: 과거 인계 기록.
- `global_path.txt`, `global_path_smoothed.txt`: 과거 경로 비교 입력.
- `규정집_v1.1.5_예비_대학생AISW모빌리티 경진대회.md`: 제공 규정 참고 자료.

원문·작성자·라이선스·과거 검사 결과는 수정하지 않았다. 이 디렉터리 안의
이전 자료 간 상대 배치는 유지했지만, 옛 절대 경로와 루트를 기준으로 하는
스크립트·문서 링크는 그대로 남아 있다. 현재 실행 안내로 사용하지 않는다.
원본의 `records_readme.py` 등도 이전 루트 경로를 참조하므로 과거 기록 도구를
그대로 실행하면 안 된다.

## 과거 경로 재현

과거 시험이 필요하면 현재 저장소와 분리한 사본에서 원래 배치를 복원한다.
아래는 복원 예시이며, 복사 후 해당 시험의 ROS2 환경·외부 입력을 별도로 준비해야 한다.
현재 주행 도구나 비교 원본을 변경할 필요는 없다.

```bash
replay_root=$(mktemp -d /tmp/rmt-legacy-replay-XXXXXX)
cp -a 과거자료/. "$replay_root/"
cp -a 2026_CTRL_competition "$replay_root/"
printf '%s\n' "$replay_root"
```

복원 예시 자체를 이번 작업에서 실행하거나 과거 시험 성공을 확인하지는 않았다.
이동 범위와 검증은 [통합 기록](../ros1-port/docs/ARCHIVE_CONSOLIDATION_2026-10-23.md)에 있다.
