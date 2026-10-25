# 경로 요청 선택·수명 명세 — 2026-10-25

## 목표와 범위

DrivingTarget의 경로 필드를 불변 요청으로 복사하고, 앞서 만든 변환/진행/조향 계층에 전달할 경로를 선택한다. 새 `ctrl_path_selection.py`와 단위 검사·재현 도구를 추가한다. production follower/launch는 연결하지 않는다. 차체 충돌 검사·종방향 제어·실제 송신은 이번 범위 밖이다.

## 선택과 유지

- 모든 요청은 기준 경로와 같은 frame, 유한한 시각·횡오프셋·활성 전환 필드를 가져야 한다. 속도 상한은 0 이상 또는 +inf이며 정지 station은 유한한 값 또는 요청 없음이다. 속도/정지 정보도 같은 요청에서 원자적으로 갱신한다.
- reference payload는 ID/x/y/global station을 함께 검증한다. release 또는 회피 필드와 동시에 보낼 수 없다. 동일 ID는 세션 내 형상·전역 라벨이 같아야 한다. point 간격/개수와 세션 ID 개수를 설정으로 제한하며, ID 기록을 몰래 버리지 않는다.
- 새 reference 없이 reference mode가 활성인 경우 명시적 release 전까지 기존 경로를 유지한다. 새 속도/정지 메타데이터와 시각만 갱신할 수 있다. release와 회피 요청을 함께 보내는 것은 허용한다.
- reference가 없으면 nonempty chain이 entry/return/lat_offset보다 우선한다. 이는 DrivingTarget의 chain 계약이다. unused 필드도 수치 형식은 확인한다. 그다음 명시적 entry/return, 마지막으로 anchor가 필요한 legacy offset을 처리한다.
- entry와 chain의 초기 offset은 메시지의 '기준 경로에서 시작' 계약에 따라 0이다. entry 없는 offset은 `TransitionAnchor(start_global_s, length_m, initial_offset_m)`를 호출자가 제공해야 한다. 이동 중인 현재 위치에서 매 요청마다 anchor를 재생성하지 않는다.
- 같은 legacy offset/return 요청의 heartbeat는 최초 anchor와 경로 객체를 재사용한다. 같은 요청에서 다른 anchor를 제공하면 거부한다. 다른 legacy offset에는 새 anchor가 필요하다.
- 경로 형상이 같으면 revision과 경로 객체를 재사용한다. 형상이나 mode가 바뀌면 revision을 올린다. 생성 offset 경로는 selector 전용 ID를 사용하며 외부 reference는 이 ID 영역을 사용할 수 없다.

## 회피 완료와 기본 경로 복귀

회피 활성 중 일반 0-offset 요청만 들어오면 기존 회피 계획을 유지한다. `progress_global_s`가 제공되고 마지막 전환 끝을 지났으며 최종 offset이 0인 경우에만 기본 경로로 복귀한다. 잘못된 범위/완료 증거는 거부한다. 최종 offset이 0이 아니면 명시적인 return anchor 또는 전환이 필요하다. 진행 위치는 호출자의 검증된 새 진행 관리 결과에서 가져와야 한다.

## 시간·실패

- 요청 나이와 미래 허용 범위는 설정으로 받는다. duplicate/역순/만료/과도하게 미래인 packet은 채택하지 않으며 기존 요청의 수명을 연장하지 않는다.
- 새롭고 유효한 시각의 packet이 malformed이면 마지막 경로는 진단용으로 보존하되 출력은 차단한다. 실패한 packet보다 새로운 정상 요청만 차단을 해제할 수 있다. 실패한 요청의 ID나 경로를 registry에 부분 등록하지 않는다.
- `current(now_s)`는 마지막 요청이 유효 기간 안이고 차단되지 않았을 때만 결과를 반환한다. 실패/만료 결과에는 선택 상태가 없다. 과거 상태를 자동 fallback으로 내보내지 않는다.
- 현재 시계가 이전 관측보다 역행하면 reset이 필요한 오류로 고정한다. 시간이 다시 커져도 자동으로 풀지 않는다. 명시적 reset 후 새 요청이 필요하며 registry/진행 상태의 새 세션 처리도 호출자가 담당한다.
- selector는 진행 구간을 자동으로 복사하지 않는다. revision이 바뀌면 호출자는 새 경로에서 전역 station 범위를 명시해 진행 관리기를 재획득해야 한다. 조향의 이전 채택 각도는 별도로 유지한다.

## 구현 전 고정한 검사

일반 요청/metadata, reference 유지·교체·ID 변경·해제, 모드 충돌, chain 우선순위, anchor 재사용/변경, 회피 완료 증거, 동일 경로 revision 유지, 배열 복사, malformed 신규 요청의 출력 차단·원자성, stale/duplicate/future·시계 역행·reset, registry/point 한계와 ROS 비의존 메시지 변환을 검사한다. 기존 경로 변환·진행·조향 검사도 함께 실행한다.

재현은 새 결과 폴더만 사용한다. 오프라인 모듈 완료와 1단계 전체 완료를 구분한다. 남은 1단계 작업은 production 의미에 맞는 병행 계산 연결·기록 재생·Linux 빌드/ROS 확인·MORAI 추종 검증이다. 모델은 Sol Medium을 기본으로 사용한다.
