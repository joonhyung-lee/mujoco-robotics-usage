# 저장소의 RG2로 잡기 → Table A에서 B로 세로 막대 이송

UR5e와 Franka Panda가 테이블 A에 세워진 직육면체 막대를 집어 들어 테이블 B에
세워 놓고, 그리퍼를 열고 물러나는 MuJoCo 태스크다. 기존 interaction MPC를 공유한다.

| 케이스 | 수행 동작 |
|---|---|
| `avoid` | 잡기 → 들기 → 알려진 벽의 끝을 돌아 이동 → B에 내려놓기 → 놓고 물러나기 |
| `contact` | 잡기 → 들기 → 벽 방향으로 이동 → 반력 추정으로 접촉 감지 → 양보 → 후퇴 → 우회 → B에 내려놓기 → 놓고 물러나기 |

막대는 **36 × 36 × 300 mm, 기본 질량 0.4 kg**이다. 두 팔에 저장소에서
**RG2로 명명된 기존 모델**을 장착했다. 원본은
[`asset/ur5e/ur5e_rg2.xml`](../../asset/ur5e/ur5e_rg2.xml)이며, 저장소에 별도의
Robotiq 모델은 없다. Franka에도 이 RG2를 플랜지에 장착한다.

RG2의 원래 mesh, 관성, 6개 hinge joint, 5개 관절 연동식과 self-contact 제외 설정을
읽어 사용한다. 하나의 구동 관절이 연동된 양쪽 손가락을 움직이며, 원래 손끝 mesh와
물체 사이의 접촉·마찰로 잡는다. 구동 각도는 0.9 rad가 열림, 0 rad 명령이 닫힘이다.

원본 XML 파일은 변경하지 않는다. 실행용 모델에서 다음 설정을 적용한다.

| 항목 | 실행용 설정 |
|---|---|
| 그리퍼 모터 | 원본 `kp=50`, 추가 토크 한계 ±2 Nm |
| 관절 armature | 원본에서 상속되던 0.05 보존 |
| 관절 damping | 5 → 0.2: 토크 제한 아래에서 개폐할 수 있도록 조정 |
| 관절 연동식 | 원본 계수 유지, `solref="0.004 1"`, `solimp="0.999 0.9999 0.001"` |
| 손끝 접촉 | 원본 mesh 사용, `condim=6`, 마찰계수 기본 `1.2 0.015 0.0005` |
| EE 기준점 | RG2 base에서 `[0, 0.0013, 0.205] m` |
| 파지·이송 높이 | 막대 COM보다 13 cm 위를 잡고, 테이블에서 7 cm 들어 올림 |

긴 그리퍼의 작업공간과 knuckle 간섭을 고려한 설정이다. 토크 값과 동역학 튜닝을
실물 RG2의 사양이나 검증된 하드웨어 설정으로 해석해서는 안 된다.

## 실행

저장소 루트에서 기존 MPC와 같은 가상환경을 사용한다.

```bash
python3 -m venv .venv
.venv/bin/pip install -r code/manipulator/requirements_mpc.txt

# 두 로봇 × 두 케이스, MP4 포함
.venv/bin/python code/manipulator/demo_table_transfer.py --video

# 각각 실행
.venv/bin/python code/manipulator/demo_table_transfer.py --robot ur5e --case avoid --video
.venv/bin/python code/manipulator/demo_table_transfer.py --robot franka --case contact --video

# GUI가 있는 머신
.venv/bin/python code/manipulator/demo_table_transfer.py --robot franka --case contact --viewer

# 물체 질량, 접촉 마찰, RG2 모터 토크 한계 변경
.venv/bin/python code/manipulator/demo_table_transfer.py \
  --robot ur5e --case contact --mass 0.6 --friction 1.2 --grip-torque 2

# 잡을 힘이 없는 경우 실제로 실패하는지 확인: 실패 결과 저장 후 exit code 1
.venv/bin/python code/manipulator/demo_table_transfer.py \
  --robot ur5e --case avoid --grip-torque 0 --output outputs/rg2_no_grip

# 회귀 검증
.venv/bin/python code/manipulator/test_table_transfer.py -v
.venv/bin/python code/manipulator/test_interaction_mpc.py -v
```

`--max-time`은 시뮬레이션 제한 시간이며 기본 70초다. 정상 실행은 각 약 28–35초의
시뮬레이션으로 끝난다. Headless 영상은 EGL을 사용한다. 결과 디렉터리는 `--output`으로
변경할 수 있으며 기본 위치는 `outputs/table_transfer_rg2/{ur5e,franka}_{avoid,contact}/`다.

- `simulation.mp4`: `--video` 사용 시 생성. 파란 테이블이 A, 초록 테이블이 B,
  주황색 긴 막대가 옮길 물체, 갈색 판이 벽이다.
- `metrics.png`: 평면 이동 경로, 물체 높이, 좌우 파지력, 벽 반력, 토크 여유, A/B 지지력.
- `trace.npz`: 100 Hz 관절 상태·EE 위치·물체 자세·RG2의 6개 관절각·추정 wrench·접촉력·단계 로그.
- `summary.json`: 성공 여부, 실패 이유, 단계 전환 시간, 놓인 위치 오차, 지지력, 최대 반력 등.
- 상위의 `comparison.json`: 이번 명령으로 실행한 모든 케이스의 결과.

같은 출력 위치로 재실행하면 결과를 덮어쓴다. 실패한 실험도 결과를 저장하고 CLI는
하나라도 실패한 경우 0이 아닌 종료 코드를 반환한다.

## 잡기와 성공 판정

막대에는 처음부터 끝까지 **free joint**가 있다. Weld, attachment constraint,
adhesion, 이동 중 물체 pose 강제 갱신을 사용하지 않는다. 손가락을 닫은 후 양쪽
접촉이 확인되어야 들어 올리며, 실제 물체가 테이블에서 올라왔는지 검사한다.
운반 중 물체가 파지 위치에서 크게 벗어나면 grasp lost로 실패 처리한다.
RG2의 5개 equality는 그리퍼 관절끼리만 연동하며 물체를 구속하지 않는다.

팔이 목적지에 도착한 것만으로 성공 판정하지 않는다. 마지막에 다음을 확인한다.

- 물체 중심의 XY 위치가 B 중심에서 35 mm 이내이고, 바닥 높이 오차가 7 mm 이내
- 물체가 세워져 있고(`cos(tilt) > 0.98`), 자유 물체 속도가 충분히 작음
- B의 지지 반력이 물체 무게의 절반 이상이고 손가락 접촉이 해제됨
- 실제 들어 올리기가 확인되었으며, `contact` 케이스에서는 반력 감지·복구 단계도 수행됨

회귀 테스트는 기본 설정에 대해 더 엄격한 위치·토크·반력 기준을 확인한다.
`--grip-torque 0` 실패 테스트도 포함하므로 물체를 단순히 팔에 붙여 운반하는 구현과
구별할 수 있다. 접촉 및 물체 pose는 파지 확인·놓기 판정에 사용하는 시뮬레이션 정보다.
실기에서 이런 판정을 하려면 그리퍼/비전 등의 관측 수단이 필요하다.

## 벽 우회와 반력 대응

`avoid`는 알려진 하나의 직사각형 벽을 그리퍼·물체 여유거리 60 mm만큼 확장하고,
벽의 +x 끝을 도는 waypoint 경로를 만든다. 각 waypoint는 초기 IK로 도달 가능성을
확인하고, 실행 중에는 Cartesian MPC가 추종한다. 팔·그리퍼·막대와 벽 사이의
물리 충돌은 활성화되어 있으며, 팔/그리퍼의 벽 접촉력도 별도로 기록한다.
이는 이 장면을 위한 경로 생성이며 임의 장애물에 대한 전신 충돌 회피 planner는 아니다.

`contact`는 먼저 벽을 통과하는 목표를 준다. **추정된 -y 방향 힘이 5 N을 넘고
진행 방향 오차가 남는 상태가 0.12초 지속**되면 접촉으로 판정한다. 이 판정에
MuJoCo의 실제 벽 반력은 쓰지 않는다. 공통 MPC의 admittance와 reference slack으로
잠시 양보한 뒤, task supervisor가 후퇴 경로를 주고 알려진 벽을 우회한다.
벽을 만졌다는 판단은 반력에서 나오지만, 복구용 우회 경로는 알려진 벽 형상을 사용한다.
미지 환경의 지도를 새로 추정하는 기능은 포함하지 않았다.

기존 점질량 데모와 달리, 긴 막대의 접촉은 EE에 **모멘트도 전달**한다.
따라서 이 태스크는 `J = [J_position; J_rotation]`인 6D wrench observer를 사용한다.
MPC에는 추정한 generalized joint disturbance 전체를 전달하여 모멘트 성분도
동역학 보상에 포함한다. 순응 방향은 wrench의 병진 힘 성분에서 정한다.
구현은 기본 3D 인터페이스도 그대로 지원하므로 기존 점질량 데모와 테스트가 동작한다.

팔 MPC와 supervisor는 100 Hz, 물리와 observer는 500 Hz다. 물체 질량은 제어기의
nominal dynamics에 넣지 않는다. Nominal gripper는 구동각 0.4 rad에 해당하는
링크 자세로 고정한 근사 모델이며, 실제 장면에서는 원본의 관절 연동식으로 움직인다. 마찰, 파지력,
위치 추정, 모델 오차에 따라 결과가 달라질 수 있다. 실기 배포·일반적인 접촉 안정성
보장을 제공하는 구현은 아니다.

## 기본 설정 검증 결과

2026-09-23, RG2 / 기본 질량 0.4 kg / 마찰계수 1.2 / 모터 토크 제한 2 Nm으로 실행했다.
네 케이스 모두 실제 들어 올리기, B 위 지지, 손가락 해제, 안정된 최종 자세를 확인했다.

| 로봇 | 케이스 | B 중심에서의 최종 XY 오차 | 최대 막대-벽 반력 | MPC 실패 |
|---|---|---:|---:|---:|
| UR5e | 우회 | 0.14 mm | 0 N | 0 |
| UR5e | 접촉 후 복구 | 1.36 mm | 19.3 N | 0 |
| Franka | 우회 | 0.19 mm | 0 N | 0 |
| Franka | 접촉 후 복구 | 1.06 mm | 19.6 N | 0 |

팔/그리퍼의 벽 접촉은 네 케이스 모두 0 N이었다. 최소 토크 여유는 각 실행에서
약 62–63%였고, 우회는 약 27.86초, 접촉 후 복구는 약 34.3초의 시뮬레이션이 걸렸다.
무구동 토크 실패 테스트와 6D wrench 관측기 테스트도 통과했다.
이 수치는 기본 장면 한 조건의 결과이며 질량·형상·마찰을 바꾸면 재검증해야 한다.

## 코드

- [table_transfer.py](table_transfer.py): 장면·실제 그리퍼·free 물체, IK, 우회 경로, 접촉 평가
- [demo_table_transfer.py](demo_table_transfer.py): 단계별 태스크, 반력 감지·복구, 성공 판정, 영상·로그
- [interaction_mpc.py](interaction_mpc.py): 기존/새 데모가 공유하는 observer와 토크 MPC
- [test_table_transfer.py](test_table_transfer.py): 4개 태스크, 무파지력 실패, 6D 관측기 회귀 검증

접촉 설정은 MuJoCo의 [공식 마찰·grasp 모델링 설명](https://github.com/google-deepmind/mujoco/blob/main/doc/modeling.rst)을 참고했다.
