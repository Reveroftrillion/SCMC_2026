# 에이전트 안내 (제출 전 삭제: practice/HANDOFF.md §1.4)

이 저장소에서 작업을 시작하면 **먼저 `practice/HANDOFF.md` 를 끝까지 읽는다.**
대회 규정, 시뮬·노트북 환경, 연습 도구, 현재 상태, 사용자 선호가 모두 거기 있다.

바로 지킬 것:
- 사용자와는 **한국어**로 대화한다.
- 사용자의 새 요청은 `practice/TuningPoint.md` **맨 위**에 있다. 처리하면 그 아래에 상태 표시와 결과 보고를 단다.
- 작업 브랜치는 `competition` (worktree `2026foscar-competition`). main 은 검증된 완성본이라 직접 커밋하지 않고, 머지는 사용자 동의 후에 한다.
- 대회 규정상 금지된 시뮬 채널(충돌·ObjectInfo·시나리오 로드·EgoVehicleStatus 전체판)은 `practice/` 안에서만 쓴다. 주행 코드(`src/`)는 `practice/` 를 참조하지 않는다.
- 카메라(`src/foscar_camera*`)는 사용자가 지시한 변경만 커밋한다. `src/foscar_bringup/routes/backup/` 은 커밋하지 않는다.
- 풀랩을 돌리면 랩타임은 러너가 GitHub main README 에 자동으로 올린다 (HANDOFF §3.2).
- 테스트는 종료코드로 판정한다(파이프로 가리지 않는다).
