# 과거 자료 통합 — 2026-10-23

## 목표·영향 파일·구현

루트의 과거 자료를 `과거자료/`로 모아 현재 개발 경로를 명확히 한다.
브랜치는 `cleanup/archive-legacy-20261023`이다.
이동 범위는 [보관 안내](../../과거자료/README.md)에 열거했다.
폴더 내부 배치를 유지하며 1,675개 파일을 이동했고 원문은 변경하지 않았다.
`2026foscar-competition/`과 `ros1-port/` 실행·리소스 경로는 유지한다.
루트 README와 MARKDOWN_INDEX의 안내·이동 링크를 갱신하고,
`tools/audit_unused_cleanup.py`에 새 보관 경로의 분류를 추가했다.
기존 날짜별 검증 문서·JSON·SHA256 목록은 당시 경로 기록으로 그대로 보존한다.

## 검증·완료 기준

- 이동 전후 1,675개 파일의 SHA256 일치 확인.
- 비교 원본 530개 파일의 SHA256 불변 확인.
- 변경 Python의 구문, 현재 안내의 보관 링크, diff whitespace 검사.
- ROS1/catkin/native/TF/MORAI·Windows 회귀·과거 ROS2 실행은 미실행.
  파일 이동·문서·분류 변경이며 주행 알고리즘을 변경하지 않았다.
- 기존 미추적 `roslaunch`, `drive:=true`는 사용자 파일로 보존.
- 삭제·commit·push·merge와 제어 송신은 수행하지 않았다.

재확인 명령:

```bash
python3 -c "import ast,pathlib; ast.parse(pathlib.Path('ros1-port/tools/audit_unused_cleanup.py').read_text())"
git diff --check
ls -1
```

## 남은 호환성

과거 도구의 하드코딩 경로·외부 ROS2 환경은 보존했으며 현 위치 실행을 보장하지 않는다.
보관 안내의 격리 복원 방법을 따른다. 원본의 records 갱신 도구가 참조하는 옛 경로도
자동으로 재작성하지 않았다. 과거 문서의 기존 깨진 링크와 기존 manifest의 옛 경로는
수정하지 않는다. 현재 Docker의 비교 원본 마운트와 production 파일 경로는 그대로다.

## 작업 중 외부 변경 발견

이동 전 원본 530개 파일의 불변 검사는 통과했으나, 이후 비교 원본 폴더가
`2026_CTRL_competition/`으로 변경된 상태를 발견했다. 이번 이동 목록에는 없는
변경이며 되돌리지 않았다. 현재 도구의 `2026foscar-competition/` 참조와
문서 분류의 원본 링크 32개는 이 이름 변경 때문에 해석되지 않는다.
나머지 문서 분류 링크는 확인했다. 변경 Python 구문·현재 안내 링크·
`git diff --check`는 통과했다. MARKDOWN_INDEX는 링크 수정과 함께 기존
CRLF를 LF로 통일했다. ROS 실행은 확인하지 않았다.
