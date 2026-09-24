# UR5e / Franka: 외력 추정 + 토크 여유 기반 interaction MPC

실제 잡기·놓기와 테이블 A→B 이송은 후속 [TABLE_TRANSFER.md](TABLE_TRANSFER.md)를 참고한다.
아래 문서는 기존의 이미 잡은 점질량 하중 데모를 설명한다.

대화에서 말한 **이미 물체를 잡은 팔이 궤적을 추종하되, 외부 접촉 시 토크 여유를
고려해 추종 오차를 허용하는 것**을 실행 가능한 MuJoCo 연구용 예제로 구현했다.
하중·벽 접촉·둘의 조합에 **동일한 observer와 MPC**를 쓴다. RL, 모션 데이터,
태스크별 제어기 전환은 없다. MPC가 물체 질량이나 벽 위치를 받지 않는다.

## 실행

저장소 루트에서 실행한다. Python 3.10, MuJoCo 3.3.7, OSQP 1.1.3으로 검증했다.

```bash
python3 -m venv .venv
.venv/bin/pip install -r code/manipulator/requirements_mpc.txt

# 두 팔: 1.5 kg 하중 + 알려주지 않은 벽. 화면 없이 실행하고 MP4 저장.
.venv/bin/python code/manipulator/demo_interaction_mpc.py --video

# 하중 운반만 / 벽 접촉만
.venv/bin/python code/manipulator/demo_interaction_mpc.py --robot ur5e --scenario payload
.venv/bin/python code/manipulator/demo_interaction_mpc.py --robot franka --scenario wall

# 같은 동역학·제약·토크 비용의 고정 추종 설정과 비교, 총 16개 실험
.venv/bin/python code/manipulator/demo_interaction_mpc.py \
  --robot both --scenario all --controller compare

# 주어진 3차원 외력으로 제어: 월드 좌표계, 환경이 로봇에 가하는 힘 [N]
# 이 옵션은 제어기 입력만 바꾼다. 시뮬레이터에 그 힘을 추가하는 옵션이 아니다.
.venv/bin/python code/manipulator/demo_interaction_mpc.py \
  --robot franka --scenario payload --force-input 0 0 -14.715

# 물체 질량과 토크 예산 변경
.venv/bin/python code/manipulator/demo_interaction_mpc.py \
  --robot ur5e --scenario combined --payload-mass 2 --torque-scale 0.8

# 디스플레이가 있는 머신의 MuJoCo viewer
.venv/bin/python code/manipulator/demo_interaction_mpc.py --robot franka --viewer

# 회귀 검증 (stdlib unittest; 별도 테스트 프레임워크 불필요)
.venv/bin/python code/manipulator/test_interaction_mpc.py -v
```

`--output`으로 결과 디렉터리를 바꿀 수 있다. 기본 위치는
`outputs/interaction_mpc/<robot>_<scenario>_<controller>/`이다.
같은 이름으로 다시 실행하면 해당 실험 결과를 덮어쓴다.

| 파일 | 내용 |
|---|---|
| `simulation.mp4` | `--video` 사용 시 960×720, 25 FPS 영상 |
| `metrics.png` | 궤적, 외력 추정/정답, 추종 오차, 토크 여유, slack 범위, 접촉력 |
| `trace.npz` | 100 Hz 로그: 관절 상태·토크, EE 위치·목표, 외력, slack 등 |
| `summary.json` | 각 실험 지표, solver 실패 횟수, 제어 계산 시간 |
| 상위의 `comparison.json` | 이번 실행의 전체 실험 지표 |

초록 구는 EE 접촉 도구이자 점질량 물체를 나타내며, 주황 구는 **원래** 목표다.
벽 뒤로 목표가 가도 로봇은 벽 앞에서 오차를 허용한다. MP4 렌더링에는 OpenGL이
필요하며, 디스플레이 없는 서버에서는 기본적으로 EGL을 선택한다. GPU가 없는 환경은
OSMesa 설치 후 `MUJOCO_GL=osmesa`로 실행할 수 있다. 영상 없는 실험은 렌더러를 쓰지 않는다.

## 공통 모델과 외력의 의미

UR5e의 6개 관절과 Franka Panda의 7개 관절에 저장소의 기존 mesh/관성 모델을 쓴다.
Franka는 `panda_nohand.xml`을 사용한다. XML은 메모리에서만 토크 모터로 변경한다.
위치 servo용 과도한 관절 damping도 이 데모에서만 1 Nm·s/rad로 바꾼다.

`f`는 **EE에서 환경이 로봇에 가하는 월드 좌표계 3D 힘**이다. 예를 들어 정지한
1.5 kg 물체는 대략 `[0, 0, -14.715] N`, +x 진행을 막는 벽은 -x 반력이다.
힘을 로봇이 환경에 가하는 방향으로 입력하거나 tool 좌표계로 그대로 입력하면 안 된다.

```math
M(q)\ddot q+h(q,\dot q)=\tau+J_p(q)^T f,\qquad
h=\mathrm{bias}-\mathrm{passive}
```

시뮬레이터에는 물체 질량을 EE 중심에 실제로 추가한다. 제어기 모델에는 추가하지
않아서 물체 중력과 병진 관성이 외란으로 나타난다. 벽은 MuJoCo 접촉으로 구현한다.
접촉은 도구 구와 벽 사이에 설정되어 있으며, 팔의 다른 링크와 벽의 충돌 회피는
이 예제의 대상이 아니다. 손가락으로 쥐는 과정, 미끄러짐, 물체 회전 관성은 생략했다.

## Force observer

필요한 입력은 관절 위치, 관절 속도, 구동 토크와 nominal dynamics다.
MuJoCo의 `qacc`, 접촉 정답, 외력 주입값, 실제 payload mass를 읽지 않는다.

```math
r=M(q)\frac{\dot q_{t+1}-\dot q_t}{\Delta t}+h-\tau,\qquad
\hat f=(J_pJ_p^T+\lambda^2I)^{-1}J_p\,\mathrm{LPF}(r)
```

관절 damping의 implicit Euler 적분에 따른 보정도 포함했다. 저역통과 필터의
rate는 25 s⁻¹, DLS damping은 0.015다. 실제 contact force와 물체 반력 정답은
실험 실행 코드의 **평가 부분에서만** 계산한다. 물체 반력 정답에는 중력뿐 아니라
EE 병진 가속도도 포함한다.

여기서 “힘 센서 없이”는 F/T 센서 없이란 뜻이다. 관절 센서와 모터 토크 정보는
필요하다. 이 예제는 노이즈 없는 encoder와 이상적인 모터 토크를 사용하며,
미분 노이즈·모터 마찰·모델 오차가 있는 실기 observer는 별도 검증해야 한다.
회귀 테스트는 두 로봇에서 양·음 방향의 월드 힘을 직접 가해 부호와 좌표계를 검증한다.

## MPC와 동적 순응

100 Hz로 상태를 다시 읽고, 40 ms 간격 10단계(0.4초)의 QP를 푼다.
매 제어 주기마다 `M`, `h`, `J`를 갱신하되 예측 구간 안에서는 고정한다.
최적화 변수는 각 단계의 관절 가속도 `a`, Cartesian reference slack `s`,
최대 정규화 토크 `rho`다. 첫 단계 토크만 적용하고 다시 최적화한다.

```math
\tau_k=M a_k+h-J_p^T\hat f,\qquad
\rho_k\geq |\tau_{k,i}|/\tau_{\max,i},\quad 0\leq\rho_k\leq1
```

비용은 `||p_k-p_ref,k-s_k||²`, `||s_k-s_adm||²`, EE 자세 유지,
관절 속도·가속도·초기 자세로부터의 변화, 정규화 토크 제곱, `rho²`의 합이다.
따라서 총 토크만 줄이는 것이 아니라 가장 여유가 적은 관절도 비용에 반영한다.
토크 여유 지표는 `margin_i = 1 - |tau_i|/tau_max,i`로 계산한다.

외력이 소비하는 토크 비율과 그 방향으로 순응 크기를 조절한다.

```math
\ell=\max_i\frac{|(J_p^T\hat f)_i|}{\tau_{\max,i}},\qquad
\alpha=\frac{\ell^4}{\ell^4+0.25^4},\qquad
\epsilon_j=0.003+0.20\alpha|n_j|,\quad n=\hat f/\|\hat f\|
```

`s_eq = clip(alpha*f_hat/150, -epsilon, epsilon)`를 0.1초 시정수로
필터링해 `s_adm`을 만든다. 포화가 없으면 `15*s_dot + 150*s = alpha*f_hat`인
1차 admittance다. 접촉 해제 시 급히 당겨지지 않도록 slack 범위는 남은
`|s_adm| + 3 mm`보다 작아지지 않는다. 외력 방향으로 reference를 양보하므로
반력을 끝없이 보상하며 밀어붙이는 현상을 줄인다.

이것은 **외력에 따라 reference를 조절하는 admittance와 MPC의 조합**이다.
모든 순응 동작을 MPC가 처음부터 발견하는 순수 최적제어라고 주장하지 않는다.
`epsilon`은 virtual reference 이동의 범위다. 실제 추종 오차의 hard bound는 아니다.
중력 하중도 같은 식에 들어가므로 payload-only에서도 수 mm 처짐이 생길 수 있다.

관절 위치(한계에서 0.03 rad 안쪽), 속도(±1.5 rad/s), 가속도(±25 rad/s²),
토크 한계를 QP에 넣었다. 출력 토크에도 같은 한계를 적용한다. 예측 모델 오차가
있으므로 관절 상태 제약을 실제 시스템에 대해 보장하는 robust MPC는 아니다.
solver 실패 시 한계 내 중력 보상·관절 감쇠로 전환하고 실패 횟수를 기록한다.

| 주요 설정 | 기본값 / 수정 위치 |
|---|---|
| UR5e 토크 예산 | `[80, 80, 60, 12, 12, 12] Nm`, `ROBOTS` |
| Franka 토크 예산 | `[87, 87, 87, 87, 12, 12, 12] Nm`, `ROBOTS` |
| 공통 토크 배율 | `--torque-scale` |
| 물체 질량 | `--payload-mass`, 기본 1.5 kg |
| 예측 간격/길이 | `InteractionMPC(dt=0.04, horizon=10)` |
| 순응 강성/감쇠 | `InteractionMPC.command`의 150 N/m, 시정수 0.1초 |
| 순응 활성화 정도 | 위 식의 0.25 |
| reference 이동 범위 | 3 mm + 최대 200 mm의 방향별 성분 |
| 입력 궤적 | `demo_interaction_mpc.py`의 `trajectory()` |

토크 예산은 이 **시뮬레이션의 설정**이며 실제 로봇에서 허용된 토크 명령 범위라는
뜻이 아니다. 본 코드는 실기 드라이버·RTDE·libfranka 인터페이스를 구현하지 않는다.

## 실험과 해석

기본 실험은 13초다. 0–1초 정지, 1–5초 접근, 5–8초 유지, 8–12초 복귀,
12–13초 정지다. 원래 목표는 +x 17 cm, +y 3.5 cm, +z 2.5 cm 이동한다.
+x 최대 목표 속도는 약 6.7 cm/s다. 벽이 있을 때 구 중심은 +x 약 5.5 cm에서
접촉하므로 목표는 벽 너머로 계속 간다. 네 시나리오는 이 궤적을 공유한다.

`fixed`는 같은 힘 추정, 예측 모델, 제약, 토크 비용을 쓰되 `alpha=0`으로 고정한
비교 설정이다. 표준 impedance controller나 튜닝된 다른 논문과의 비교가 아니다.
접촉 중 QP 실패가 있는 실행은 fallback도 섞인 결과이므로 `solver_failures`를
반드시 함께 읽어야 한다. 비교 결과만으로 일반적 성능 우위나 연구 신규성을 주장할 수 없다.

`settled_contact_n`은 5.5–7.5초 접촉력 크기의 평균,
`settled_min_joint_margin`은 같은 구간에서 매 시점의 최소 관절 여유를 평균한 값이다.
최대 접촉력은 로그의 100 Hz 표본이 아니라 **500 Hz 물리 적분 전체**에서 측정한다.
벽을 미리 모르기 때문에 첫 충돌 피크는 반응형 제어로 미리 없애지 못한다.
회귀 테스트의 접촉력 한계는 이 설정의 회귀 기준이며 일반적인 안전 보장이 아니다.

2026-09-23 기본 설정의 16회 시뮬레이션을 실행했다. adaptive의 8개 로봇/환경 조합은
모두 QP 실패 없이 완료했고, 힘 부호·좌표계·입력 검증·불가능한 QP의 bounded fallback
회귀 검증도 통과했다. 하중+벽 실험은 다음과 같다.

| 로봇 | 설정 | 유지 구간 평균 접촉력 | 물리 적분 최대 접촉력 | 유지 구간 평균 최소 토크 여유 | QP 실패 / 1300회 |
|---|---|---:|---:|---:|---:|
| UR5e | fixed | 124.4 N | 206.8 N | 7.8% | 82 |
| UR5e | adaptive | 33.0 N | 49.2 N | 59.9% | 0 |
| Franka | fixed | 58.2 N | 310.3 N | 41.7% | 475 |
| Franka | adaptive | 23.9 N | 93.3 N | 53.6% | 0 |

**fixed 행은 실패 후 fallback이 포함된 전체 폐루프 결과다.** 특히 Franka wall-only는
866회 실패하여 유지 구간에서 접촉을 잃었다. 따라서 그 실험의 평균 접촉력 0 N을
좋은 제어 성능으로 해석하면 안 된다. 비교기는 실패를 숨기지 않는 기본 ablation이다.
adaptive도 Franka 접촉력에 잔진동이 남으며, 일정한 목표 접촉력을 정밀하게 유지하는
force controller로 검증한 것은 아니다.

adaptive의 자유 이동 위치 RMSE는 UR5e 0.66 mm / Franka 2.52 mm, 하중만 있을 때는
5.13 mm / 4.69 mm였다. 이 서버에서 제어 계산 p95는 실험에 따라 약 1.8–3.8 ms였다.
이는 측정치이며 Python/OSQP의 실시간 deadline을 보장하지 않는다. 원본 수치와 로그는
저장소 루트의 `outputs/interaction_mpc/`, 검증한 13초 영상은
`outputs/interaction_mpc_videos/{ur5e,franka}_combined_adaptive/simulation.mp4`에 있다.

3D 점힘 가정은 EE에 순수 토크가 걸리거나 여러 링크가 동시에 접촉하면 충분하지 않다.
그 경우 6D wrench와 해당 접촉 Jacobian, 또는 generalized disturbance를 사용해야 한다.
`observer_fit_error`는 추정한 점힘으로 설명되지 않는 관절 residual의 크기를 기록한다.
실제 로봇 배포, 잡기/놓기, 장애물 회피, 충격 예측, passivity·안정성 증명,
센서 노이즈와 모델 오차에 대한 검증은 포함하지 않았다.

## 코드와 참고 자료

- [interaction_mpc.py](interaction_mpc.py): 모델 구성, observer, 공통 QP
- [demo_interaction_mpc.py](demo_interaction_mpc.py): 환경, 궤적, 평가, 시각화, CLI
- [test_interaction_mpc.py](test_interaction_mpc.py): 힘의 부호·좌표계, 두 로봇의 네 시나리오 검증
- [MuJoCo 공식 dynamics 설명](https://mujoco.readthedocs.io/en/stable/computation/index.html): 운동방정식과 force convention
- [MuJoCo 3.3.7 API](https://mujoco.readthedocs.io/en/3.3.7/APIreference/APIfunctions.html): `mj_jacSite`, `mj_fullM`, `mj_contactForce`
- [OSQP 공식 MPC 예제](https://osqp.org/docs/examples/mpc.html): QP를 이용한 receding-horizon 제어
