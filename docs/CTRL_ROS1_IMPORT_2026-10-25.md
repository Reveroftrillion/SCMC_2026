# CTRL ROS1 작업 반영 — 2026-10-25

SCMC_2026 기준 main은 `f83e18c013453e45454a4afb050a2f8be17263b4`, 반영한 RMT 작업 커밋은 `412c47a6d54b8fff4d69b8ad90e759994e71fe2d`다. 두 저장소의 Git 이력을 병합해 기존 변경 이력과 기준 커밋을 유지했다. 기존 `src/`와 `docs/`의 파일은 그대로 남겼다.

## 이번 반영

`ros1-port/`의 기존 실행 구성과 새 조향/진행/경로 변환/요청 선택 모듈, 278개 단위 검사와 단계별 합성 결과, CTRL 개발 지침과 현재 문서 목차를 포함한다. 새 계산은 오프라인 기반이며 production follower에 아직 연결하지 않았다.

기존 SCMC 설명은 [이전 README](SCMC_PREVIOUS_README_2026-10-25.md)에 원래 Git blob 바이트로 보관했다. 현재 실행/진척은 [팀 README](../README.md)와 [개발 계획](../ros1-port/docs/DEVELOPMENT_WORKFLOW.md)을 따른다.

## 파일 보존

기존 SCMC 파일 중 README·ignore/attributes 관리 파일만 통합했다. 새 `ros1-port/`와 기존 루트 `src/`는 별개의 workspace로 취급한다. 가져온 코드/원문·검증 자료의 줄바꿈을 변환하지 않도록 해당 트리를 `.gitattributes`에서 별도 처리했다. 최초 문서 보관 사본 중 14개는 source 작업 폴더의 검증된 바이트로 복원해 기존 hash와 일치시켰다.

기존 RMT 이력의 빈 파일 `drive:=true`는 Windows에서 만들 수 없는 실행 인자 형태의 이름이라 새 병합 tip에서는 제외했다. 이 파일의 내용은 0바이트이고 이전 Git 이력은 그대로 보존된다. 코드·모델·실험 원본을 제거한 것은 아니다.

MORAI 주행이나 Linux catkin 빌드를 새로 수행한 반영은 아니다. 실제 제어 전환은 병행 계산, 기록 재생과 해당 환경 검증 뒤 진행한다.

업로드용 checkout에서 단위 278개와 연속 합성 2개 장면·2,500 step을 재확인했다. 기존 SCMC 파일 290개와 원문 사본 18개, 문서 링크를 확인한 [반영 검증 요약](ctrl_import_verification_2026-10-25.json)을 함께 보관한다. 이전 자산 목록의 작업 폴더 hash와 게시된 Git blob의 줄바꿈 차이는 새 요약에 구분해 기록하고 과거 기대값은 덮어쓰지 않았다.
