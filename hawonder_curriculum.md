Hawonder 자율주행 자동차
임베디드 교육 커리큘럼
1개월 집중 과정  |  20일 × 7시간  |  총 140시간
# 📋 전체 일정 개요
하루 구성: 이론 3h (09:00–12:00)  /  실습 3h (13:00–16:00)  /  리뷰·Q&A 1h (16:00–17:00)
### 🌅 Morning | Theory (3h)
임베디드 시스템 정의: MCU vs MPU vs SoC 개념 비교
RPi 5 구조: BCM2712 (Cortex-A76), RP1 남부 칩셋, PCIe 2.0 슬롯
GPIO 경로 변화: RP1 우회 구조 → libgpiod 필수 이유, RPi.GPIO 불안정
ARM Cortex-A76: 파이프라인, 캐시 계층 (L1/L2/L3), AXI/APB 버스
RISC 명령어 세트 개론
### 🔬 Afternoon | Lab (3h) — Python
RPi 5 첫 부팅, SSH 환경 구성 (key-based 인증)
lscpu, /proc/cpuinfo, lsblk, vcgencmd 로 HW 파악 리포트 작성
gpiozero + libgpiod Python 바인딩으로 LED/버튼 제어
### 📝 Review (1h)
RPi 5 아키텍처 다이어그램 직접 그리기 퀴즈
### 🌅 Morning | Theory (3h)
RPi 5 부트 시퀀스: EEPROM 부트로더 → Kernel → systemd
파일시스템 계층: /dev, /proc, /sys, /boot/firmware 의미와 역할
프로세스 모델: fork/exec, PID, 부모-자식 관계, zombie 프로세스
파일 권한, udev 규칙, cron
셸 스크립팅: 변수, 조건, 반복, 함수
### 🔬 Afternoon | Lab (3h) — Python + Shell
systemd 서비스 작성 + 등록 (/etc/systemd/system/)
Python 스크립트를 systemd 서비스로 등록하는 패턴 실습
/proc/interrupts, /proc/iomem 탐색
udev 규칙: USB 장치 연결 시 자동 심볼릭 링크 생성
### 📝 Review (1h)
부트 순서 플로우차트 완성 퀴즈
### 🌅 Morning | Theory (3h)
전기 기초: 옴의 법칙, 전압 분배, 전류 제한 저항
풀업/풀다운 저항, 3.3V 로직 레벨 (RPi 5 IO 전압), 레벨 시프터 필요성
디커플링 커패시터 역할, 회로도 읽는 법
RPi 5 전원 설계: 5V/5A USB-C PD 필수, 전력 예산 계산
UART 디버그 콘솔 원리: TX/RX 핀, 보레이트, 흐름 제어
### 🔬 Afternoon | Lab (3h) — Python
UART 디버그 케이블로 직접 부팅 콘솔 접속 (minicom)
부팅 로그 단계별 관찰: EEPROM → Kernel → systemd
Python gpiozero / libgpiod GPIO 인터럽트 처리 실습
멀티미터로 전압 측정, 전류 소비 측정
### 📝 Review (1h)
회로도 해석 문제 3개
### 🌅 Morning | Theory (3h)
UART: 프레이밍, 스타트/스탑 비트, 보레이트 오차 허용 범위
SPI: CPOL/CPHA 4가지 모드, CS 라인, 체인 연결
I2C: 7/10비트 어드레싱, ACK/NACK, 클럭 스트레칭, 멀티마스터
CAN: 차동 신호 (CANH/CANL), 중재 메커니즘, 자동차 도메인 (개론)
USB: 열거 과정, descriptor 구조 ← Hailo PCIe 연결을 위한 배경 지식
### 🔬 Afternoon | Lab (3h) — Python
smbus2 로 I2C 센서 통신 (i2cdetect, i2cdump)
spidev 로 SPI 디바이스 통신 실습
pyserial 로 UART 통신 실습
USB 로직 애널라이저 (Saleae 클론 / PulseView) 로 UART·I2C 파형 캡처 및 수동 디코딩
의도적 보레이트 불일치 → 깨진 파형 관찰 및 원인 분석
### 📝 Review (1h)
프로토콜 선택 기준 토론 (속도 / 거리 / 디바이스 수)
### 🌅 Morning | Theory (3h)
Process vs Thread: 메모리 공간, PCB/TCB, 컨텍스트 스위칭 비용
Race condition, Critical section, Deadlock 발생 원리
동기화 기법: Mutex, Semaphore, Condition Variable
Python GIL 상세: threading (I/O bound) vs multiprocessing (CPU bound) 선택 기준
concurrent.futures 추상화 레이어
IPC: Pipe, Queue, Shared Memory (multiprocessing 모듈)
예고: ROS2 Executor 모델에서 이 개념이 직접 적용되는 방식
### 🔬 Afternoon | Lab (3h) — Python
의도적 race condition 재현 → threading.Lock() 으로 해결 before/after
multiprocessing.Pool 로 이미지 배치 병렬 처리 벤치마크
queue.Queue 생산자-소비자 패턴 구현
htop 으로 4코어 사용률 실시간 관찰
### 📝 Review (1h)
"카메라 콜백이 제어 루프를 블로킹하면 어떤 일이?" 예고 토론
### 🌅 Morning | Theory (3h)
DC 브러시드 모터: 로렌츠 힘 원리, Back-EMF, 정류자-브러시 구조
BLDC 모터: 3상 구조, 전자 정류(ESC) 원리, 브러시드 대비 장단점
H-Bridge: 방향 제어 원리, PWM 듀티비 ↔ 속도 관계, 데드타임
홀 효과 (Hall Effect): 전류 + 자기장 → 수직 전압 유도 원리
홀 센서 엔코더 (모터 내장형):
로터 자석 극성 변화 감지 메커니즘
3상 홀 신호 (U/V/W) → 정류 타이밍 + 속도/방향 계산
PPR (Pulse Per Revolution), 속도 측정 방법론
광학식 엔코더와 비교: 먼지/오일 강인성, 내장 구조 장점
### 🔬 Afternoon | Lab (3h) — Python
gpiozero / libgpiod 로 PWM 모터 전진/후진/정지 제어
GPIO 인터럽트로 홀 센서 펄스 카운팅 → RPM 실시간 계산
로직 애널라이저로 홀 3채널 동시 캡처 → 상 순서 확인
RPM 실시간 플로팅 (matplotlib 애니메이션)
### 📝 Review (1h)
홀 3채널 신호로 모터 방향 판별 로직 직접 설계 문제
### 🌅 Morning | Theory (3h)
MEMS 가속도계: 부유 질량체, 차동 커패시턴스 변화 → 가속도 측정
MEMS 자이로스코프: 코리올리 효과 — 진동 질량체 + 회전 → 수직 힘 발생
센서 노이즈 모델: White noise, Bias, Bias instability, Random Walk
Allan Variance 그래프로 IMU 품질 해석하는 방법
Euler angles vs Quaternion — 짐벌락(Gimbal Lock) 문제 시각화
### 🔬 Afternoon | Lab (3h) — Python
smbus2 로 IMU raw 가속도/각속도 데이터 수집
정지 상태 Bias 측정 및 소프트웨어 보정
Allan Variance 플로팅 실습
가속도계만으로 Roll/Pitch 계산 → 동적 환경에서의 한계 체감
### 📝 Review (1h)
"자이로 + 가속도를 어떻게 합치나?" → 칼만 필터 동기부여
### 🌅 Morning | Theory (3h)
왜 칼만 필터인가: 측정 불확실성 + 모델 불확실성 동시 처리
Predict 단계: 상태 전이, 공분산 전파 (Q 행렬 — 모델 신뢰도)
Update 단계: 칼만 게인, Innovation (측정 잔차), 공분산 업데이트 (R 행렬)
Q/R 파라미터의 물리적 의미: Q↑ = 모델 불신 / R↑ = 센서 불신
Complementary Filter와 비교 (단순하지만 주파수 도메인 직관 필요)
Extended Kalman Filter (EKF) 개념 소개: 비선형 시스템 선형화
### 🔬 Afternoon | Lab (3h) — Python (numpy)
1D 칼만 필터 Python 구현 (1D 위치 추정 예제)
IMU 가속도계 + 자이로 데이터 Complementary Filter로 융합
동일 데이터 칼만 필터로 융합 후 결과 비교 플로팅
Q/R 파라미터 변화 실험: 각각 10배 증가 시 응답 변화 관찰
### 📝 Review (1h)
Q↑ vs R↑ 각각 어떤 효과인지 직관적 설명 퀴즈
### 🌅 Morning | Theory (3h)
피드백 제어 시스템: 오차, 플랜트, 컨트롤러, 설정값(Setpoint)
P / I / D 각 항의 물리적 의미와 응답 특성
튜닝 전략: Ziegler-Nichols 개념, 수동 튜닝 순서 (P → PI → PID)
Integral Windup 문제 → Anti-windup 기법 (클램핑)
Derivative Kick 문제 → Derivative on Measurement 해결
속도 PID (홀 센서 RPM 피드백) vs 위치 PID 차이 — Day 6과 연결
차량 조향각 PID 적용 방법론
### 🔬 Afternoon | Lab (3h) — Python
Day 6 홀 센서 RPM 측정값을 피드백으로 속도 PID 완성
P만 → PI → PID 순서로 응답 변화 실시간 플로팅 비교
Anti-windup 적용 before/after 오버슈트 비교
차량 조향 PID 기초 구현
### 📝 Review (1h)
오버슈트 / 언더댐핑 / 임계감쇠 그래프 해석
### 🌅 Morning | Theory (3h)
① 1D LiDAR (1.5h)
ToF (Time-of-Flight) 방식: d = (c × t) / 2, 펄스 vs 연속파
위상편이 방식: 변조 주파수, 위상 비교로 거리 계산, 최대 비모호 거리
삼각측량 방식: IR LED + 수광 소자 (저가형 Sharp 계열 원리)
노이즈 특성: 검정면/유리 반사율 문제, 최소 거리 사각지대, 온도 드리프트
② Depth Camera (1.5h)
구조광 (Structured Light): IR 점/격자 패턴 투영 → IR 카메라로 변형 감지 → 삼각측량
위상편이 ToF: 변조 IR → 픽셀 배열 전체 위상 측정 → Depth Map 생성
연결 개념: "1D LiDAR ToF를 2D 픽셀 배열로 확장하면 Depth Camera"
구조광 vs ToF 비교: 근거리 정밀도 vs 원거리/프레임레이트
공통 약점: 직사광선, 다중 경로 간섭, 투명 물체
### 🔬 Afternoon | Lab (3h) — Python
1D LiDAR 연결 + 거리 실시간 플로팅
이동평균 필터 적용 → 칼만 필터 적용 비교 (Day 8 연결)
Depth Camera 스트림 수신 + Depth Map 시각화
RGB + Depth 정렬, 1D LiDAR vs Depth Camera 거리값 비교
### 📝 Review (1h)
각 센서 한계 정리 → 센서 퓨전 동기부여
### 🌅 Morning | Theory (3h)
핀홀 카메라 모델: 투영 변환, 초점 거리, 주점(Principal Point)
내부 파라미터 (Intrinsic): 카메라 행렬 K — 픽셀-미터 변환
외부 파라미터 (Extrinsic): 회전 행렬 R + 이동 벡터 t (세계→카메라)
왜곡 계수: 방사형(k1,k2,k3), 접선형(p1,p2)
Zhang's Method: 체스보드 다각도 촬영으로 캘리브레이션 원리
Homography, Bird's Eye View (IPM) 변환
Depth Camera 추가: 깊이 스케일 팩터, RGB-Depth 정렬 행렬
### 🔬 Afternoon | Lab (3h) — Python (OpenCV)
cv2.calibrateCamera() 로 직접 카메라 캘리브레이션 수행
왜곡 보정 전후 이미지 시각적 비교
getPerspectiveTransform() 으로 BEV 변환 구현
캘리브레이션 결과를 ROS2 형식의 .yaml 파일로 저장
### 📝 Review (1h)
재투영 오차(Reprojection Error) 해석 및 원인 토론
### 🌅 Morning | Theory (3h)
SLAM 문제 정의: Localization + Mapping 동시 수행의 닭-달걀 문제
Front-end: 특징 추출 (ORB/FAST), 휠 오도메트리, Visual Odometry
Back-end: 포즈 그래프 최적화 (g2o/GTSAM), 루프 클로저 감지
Occupancy Grid Map 구조: 격자별 점유 확률
RTAB-Map 핵심 아키텍처
센서별 역할: Depth Camera (주 오도메트리/매핑), IMU (회전 보정), 1D LiDAR (장애물 보조)
RPi 5 튜닝 파라미터: Mem/STMSize, Kp/MaxFeatures, RGBD/LinearUpdate, Rtabmap/TimeThr
### 🔬 Afternoon | Lab (3h) — Python + ROS2
하이원더 제공 RTAB-Map ROS2 런치 파일 분석 (파라미터 구조 이해)
rviz2 로 실시간 맵 생성 + 루프 클로저 발생 순간 관찰
ros2 bag 주행 데이터 레코딩 → 재생하며 파라미터 변경 비교
rtabmap-databaseViewer 로 생성된 맵 DB 탐색
Working Memory / LTM 이관 로그 관찰
### 📝 Review (1h)
루프 클로저 없는 맵 vs 있는 맵 비교 시연 + 파라미터 영향 토론
### 🌅 Morning | Theory (3h)
ROS1 → ROS2: Master 제거, DDS 기반 분산 디스커버리 구조
DDS 미들웨어: RTPS 프로토콜, Domain ID, Participant 개념
QoS 정책 상세: Reliability / Durability / History / Deadline
카메라 토픽 = Best Effort / 제어 토픽 = Reliable 선택 이유
Node / Topic / Service / Action / Parameter 각 용도와 차이점
Lifecycle Node: Unconfigured → Inactive → Active → Finalized 상태 전이
colcon 빌드 시스템, ament_python 패키지 구조
### 🔬 Afternoon | Lab (3h) — Python (rclpy)
커스텀 메시지 타입 정의 + colcon 빌드
Service 서버/클라이언트 구현 (Python)
Action 서버/클라이언트 구현 (장시간 작업 + 진행률 피드백)
QoS Best Effort vs Reliable 구독자 혼용 동작 실험
### 📝 Review (1h)
Service vs Action 선택 기준 퀴즈
### 🌅 Morning | Theory (3h)
TF2 좌표 변환 프레임워크: base_link / odom / map 프레임 관계
TransformBroadcaster / TransformListener / StaticTransformBroadcaster
Executor 모델 상세 (Day 5 OS 스레딩 개념의 직접 적용):
SingleThreadedExecutor vs MultiThreadedExecutor vs StaticSingleThreadedExecutor
MutuallyExclusiveCallbackGroup vs ReentrantCallbackGroup
Intra-process Communication: 같은 프로세스 내 제로카피 토픽 전송
rclpy GIL 한계 → 고성능 노드에서 rclcpp 전환 판단 기준
디버깅 도구: rqt_graph, ros2 topic hz, ros2 topic echo, PlotJuggler, Foxglove
### 🔬 Afternoon | Lab (3h) — Python (rclpy)
카메라 콜백 + 제어 루프 SingleThreaded 블로킹 상황 의도적 재현
MultiThreadedExecutor + ReentrantCallbackGroup 으로 해결 → 레이턴시 측정 비교
TF2 브로드캐스터: 차량 기준 LiDAR / Depth Camera 프레임 등록
rqt_graph 로 전체 노드 토폴로지 시각화
### 📝 Review (1h)
노드별 콜백 그룹 설계 패턴 팀 토론
### 🌅 Morning | Theory (3h)
ros2_control 프레임워크: Hardware Interface, Controller Manager
diff_drive_controller: 차동 구동 수식, cmd_vel → 홀 센서 오도메트리 연결 (Day 6/9)
Nav2 스택 아키텍처:
BT Navigator, Global Planner (NavFn/Smac), Local Planner (DWB)
Costmap2D: static / obstacle / inflation layer
Recovery Behaviors
URDF/XACRO 기초: 차량 기하학 기술, 센서 위치 정의
### 🔬 Afternoon | Lab (3h) — Python + ROS2
하이원더 차량 키트 전체 셋업 코드 첫 실행
전체 ROS2 노드 기동 확인, rqt_graph 토폴로지 파악
cmd_vel 수동 퍼블리시 → 전진/후진/회전 제어
팀별 코드 분해 담당 모듈 배정 + 발표 가이드라인 공유
### 📝 Review (1h)
발표 구성 가이드: rqt_graph + 토픽 플로우 다이어그램 + 개선 아이디어
하루 종일 팀 발표 (7h)
각 팀 발표 후 Q&A 15분
강사 마무리: 각 모듈 개선 포인트 + Week 4 작업과의 연결고리 설명
### 🌅 Morning | Theory (3h)
색공간 변환: BGR → HSV / Grayscale, 각각 사용하는 경우
이진화: Otsu's Method, Adaptive Threshold
형태학적 연산: Erosion, Dilation, Opening, Closing
Canny Edge Detection: Gaussian smoothing → Gradient → Non-max suppression → Hysteresis
Hough Line Transform: 허프 공간, 투표 메커니즘
Perspective Transform → BEV(Bird's Eye View) → 슬라이딩 윈도우
Polynomial Fitting: 2차 곡선으로 곡선 차선 처리
### 🔬 Afternoon | Lab (3h) — Python (OpenCV)
HSV 마스킹으로 차선 색상 분리
Canny + Hough 기반 차선 경계 추출
BEV 변환 + 슬라이딩 윈도우 구현 (Day 11 캘리브레이션 파일 활용)
차선 중심점 → cross-track error 계산
### 📝 Review (1h)
조명 변화 취약성 원인 분석 + 강건화 방법 토론
### 🌅 Morning | Theory (3h)
차선 감지 실패 시 Fallback 전략 설계
센서 퓨전 기초:
카메라 차선 감지 + 1D LiDAR 장애물 거리 → 통합 의사결정 구조
Depth Camera 거리 + 2D 박스 → 3D 위치 추정
Safety 설계: Watchdog timer 원리, 비상 정지 로직, fail-safe 상태 머신
### 🔬 Afternoon | Lab (3h) — Python + ROS2
차선 감지 파이프라인을 ROS2 노드로 패키징
sensor_msgs/Image 구독 → 조향 오차 Float32 퍼블리시
LiDAR 거리 + 차선 감지 결합 의사결정 노드 구현
실제 트랙에서 차선 추종 + 장애물 감지 정지 시험 주행
ros2 bag 으로 성공/실패 케이스 레코딩
### 📝 Review (1h)
실패 케이스 ros2 bag 재생으로 원인 분석
### 🌅 Morning | Theory (3h)
① YOLOv5 구조 + YOLO 공식 메트릭 (1h)
YOLOv5 아키텍처: CSPNet Backbone, PANet Neck, Detection Head
YOLO 공식 메트릭:
Precision / Recall / mAP@0.5 / mAP@0.5:0.95
Confusion Matrix, PR Curve, F1 Curve
results.csv 해석 방법 (학습 모니터링)
② 전이학습 (Transfer Learning) (1h)
왜 전이학습인가: 소량 데이터 + 사전학습 가중치 활용의 효율성
Freeze 전략: Backbone 동결 → Head만 학습 → 점진적 해제
커스텀 데이터셋 구성: YOLO txt 라벨 형식, data.yaml 작성
데이터 라벨링 도구: labelImg / Roboflow
과적합 방지: Augmentation, Early Stopping, Dropout
③ 양자화 (Quantization) (1h)
왜 양자화인가: FP32 → INT8, 모델 크기 1/4, 추론 속도 향상
Post-Training Quantization (PTQ): 학습 후 적용, 캘리브레이션 데이터 필요
Quantization-Aware Training (QAT): 학습 중 양자화 시뮬레이션
정확도 트레이드오프: mAP 변화 정량 평가
Hailo 컴파일러와의 연결: .hef 변환 시 INT8 PTQ 내장 ← Day 20 직결
### 🔬 Afternoon | Lab (3h) — Python
커스텀 데이터셋으로 YOLOv5 전이학습 실행:
python train.py --weights yolov5s.pt --data custom.yaml --freeze 10 --epochs 50
results.csv + PR Curve 로 학습 결과 분석
PTQ 적용 전후 mAP 비교 측정
ONNX 변환 후 OpenCV DNN 추론 → FPS 측정
### 📝 Review (1h)
"소프트웨어 최적화의 한계" 확인 → Hailo-10 도입 필연성 토론
### 🌅 Morning | Theory + Lab (3h)
Hailo-10 아키텍처 + 컴파일 파이프라인 (1.5h)
Hailo-10 NPU 구조: 데이터플로우 모델, 온칩 SRAM 구조
RPi 5 PCIe 2.0 ↔ Hailo M.2 HAT 연결 구조 (USB 대비 대역폭 우위)
컴파일 파이프라인: PyTorch → ONNX → Hailo Dataflow Compiler → .hef → HailoRT API
.hef 내부에 INT8 PTQ 포함 ← Day 19 양자화가 여기서 직접 연결
전처리 / 추론 / 후처리 파이프라이닝 전략
Lab (1.5h) — Python
Hailo Model Zoo YOLOv5 .hef 파일 로드 + 추론 실행
CPU (~1 FPS) vs Hailo (~20–30 FPS) 비교 측정표 작성
ROS2 노드 래핑: 카메라 이미지 구독 → 감지 결과 퍼블리시
### 🌇 Afternoon | 최종 통합 데모 (3h)
각 팀 차량으로 전체 통합 주행 데모:
차선 감지 + PID 조향 자율 주행
Hailo-10 YOLOv5 실시간 물체 감지 동작 확인
1D LiDAR 장애물 감지 + 정지 반응
RTAB-Map 실시간 맵 생성 병행
FPS / 레이턴시 / 안정성 지표 측정 및 팀별 발표
### 📝 최종 회고 (1h)
KPT 회고: Keep / Problem / Try
Hailo 커스텀 모델 컴파일 자율 실습 가이드 문서 배포
이후 심화 학습 로드맵:
3D LiDAR + PointPillars 기반 3D 객체 감지
강화학습 (RL) 기반 자율 제어
ROS2 Real-time 패치 (Xenomai / PREEMPT-RT)
Hailo QAT + 커스텀 모델 컴파일 최적화
# 📎 부록: Hailo 커스텀 모델 컴파일 가이드
아래 내용은 Day 20 이후 자율 실습용으로 제공하는 가이드입니다.

| 플랫폼 | Raspberry Pi 5 (BCM2712, Cortex-A76) |
|---|---|
| 주요 언어 | Python (rclpy / OpenCV / numpy / PyTorch) |
| 프레임워크 | ROS2 Humble |
| AI 가속 | Hailo-10 NPU (M.2 HAT, PCIe 2.0) |
| SLAM | RTAB-Map (Depth Camera + IMU + 1D LiDAR) |
| 일정 | 주 5일 × 7시간 = 140시간 |


| Day | 주제 | 핵심 실습 산출물 | 연결고리 |
|---|---|---|---|
| 1 | RPi 5 + ARM 아키텍처 개론 | HW 파악 리포트 | HW 기반 |
| 2 | Linux 기초 + 부트 시퀀스 | systemd 서비스 등록 | OS 기반 |
| 3 | 전자기초 + UART 디버그 + GPIO | UART 콘솔 부팅 성공 | 신호 기반 |
| 4 | 통신 프로토콜 + 신호 디버깅 | 오실로스코프 파형 캡처 | 프로토콜 |
| 5 | 멀티스레딩 / 멀티프로세싱 | Race condition 해결 실습 | → Day 14 Executor |
| 6 | 모터 구조 + 홀 센서 엔코더 | RPM 실시간 측정 | → Day 9 PID |
| 7 | IMU + MEMS 원리 | Allan Variance 플롯 | → Day 8 칼만 |
| 8 | 칼만 필터 | 필터 구현 + 비교 플롯 | → SLAM 오도메트리 |
| 9 | PID 제어 + 엔코더 피드백 완성 | 속도 PID 튜닝 결과 | → ros2_control |
| 10 | 1D LiDAR + Depth Camera (ToF/구조광) | 장애물 감지 노드 | → 센서 퓨전 |
| 11 | 카메라 기하학 + 캘리브레이션 | 캘리브레이션 .yaml 파일 | → OpenCV |
| 12 | SLAM 개론 + RTAB-Map 분석 | 맵 생성 + bag 재구성 | → Nav2 |
| 13 | ROS2 ① DDS / QoS / 메시지 | 커스텀 Action 구현 | ROS2 기반 |
| 14 | ROS2 ② Executor / 동시성 | 블로킹 재현 → 해결 | Day 5 적용 |
| 15 | ros2_control + Nav2 + 차량 셋업 | 차량 첫 주행 | 통합 시작 |
| 16 | 팀별 코드 분해 발표 | 팀 발표 자료 | 통합 이해 |
| 17 | OpenCV + 차선 감지 파이프라인 | BEV + 슬라이딩 윈도우 | 인지 |
| 18 | 차선 감지 ROS2 통합 + 센서 퓨전 | 차선 추종 시험 주행 | 통합 |
| 19 | YOLOv5 + 전이학습 + 양자화 | FPS 비교 측정표 | → Day 20 |
| 20 | Hailo-10 + 최종 통합 데모 | 통합 주행 데모 | 완성 |


| WEEK 1  |  Hardware / Linux / OS / Communication |
|---|


| Day 1 | 임베디드 개론 + Raspberry Pi 5 + ARM 아키텍처 |
|---|---|


| Day 2 | Linux 기초 심화 + 부트 시퀀스 |
|---|---|


| Day 3 | 전기전자 기초 + UART 디버그 부팅 + GPIO |
|---|---|


| Day 4 | 통신 프로토콜 이론 + 신호 디버깅 |
|---|---|


| Day 5 | 멀티스레딩 & 멀티프로세싱 (OS 수준) |
|---|---|


| WEEK 2  |  액추에이터 + 센서 + 제어 이론 + 인지 기초 |
|---|


| Day 6 | 모터 구조 + 홀 센서 엔코더 |
|---|---|


| Day 7 | IMU + MEMS 원리 |
|---|---|


| Day 8 | 칼만 필터 |
|---|---|


| Day 9 | PID 제어 + 엔코더 피드백 루프 완성 |
|---|---|


| Day 10 | 1D LiDAR + Depth Camera (ToF & 구조광) |
|---|---|


| Day 11 | 카메라 기하학 + 캘리브레이션 |
|---|---|


| WEEK 3  |  SLAM + ROS2 심화 + 차량 셋업 + 발표 |
|---|


| Day 12 | SLAM 개론 + RTAB-Map 구조 분석 |
|---|---|


| 구성요소 | 설명 |
|---|---|
| Front-end | RGB-D Odometry (ICP + Visual Feature) + IMU 퓨전 |
| Working Memory (WM) | 현재 처리 중인 노드 집합 |
| Long-Term Memory (LTM) | 처리 한계 초과 노드 이관 → RPi 실시간성 유지 핵심 |
| Loop Closure Detection | Bag-of-Words 시각 사전으로 장소 재인식 |
| Graph Optimization | g2o / GTSAM 기반 포즈 그래프 최적화 |
| Occupancy Grid | Depth 포인트 클라우드 → 2D/3D 맵 투영 |


| Day 13 | ROS2 아키텍처 심화 ①  — DDS / QoS / 메시지 |
|---|---|


| Day 14 | ROS2 아키텍처 심화 ②  — Executor / 동시성 |
|---|---|


| Day 15 | ros2_control + Nav2 + 차량 첫 셋업 |
|---|---|


| Day 16 | 팀별 코드 분해 발표 세션 |
|---|---|


| 팀 | 담당 모듈 | 발표 포함 내용 |
|---|---|---|
| 팀 A | 모터 드라이버 + ros2_control + 홀 오도메트리 | 노드 그래프, 홀 신호 처리 로직, 개선 아이디어 |
| 팀 B | 카메라 노드 + Depth 스트림 퍼블리싱 | 이미지 파이프라인, QoS 설정, 최적화 방향 |
| 팀 C | IMU + 1D LiDAR 노드 + TF2 프레임 구성 | 센서 토픽 플로우, 좌표 변환 다이어그램 |
| 팀 D | 전체 Launch 파일 + 파라미터 관리 + RTAB-Map 연동 | 런치 의존성 그래프, 파라미터 구조 |


| WEEK 4  |  OpenCV + YOLO + 양자화 + Hailo-10 + 최종 통합 |
|---|


| Day 17 | OpenCV 기초 + 차선 감지 파이프라인 |
|---|---|


| Day 18 | 차선 감지 ROS2 통합 + 센서 퓨전 + Safety |
|---|---|


| Day 19 | YOLOv5 + 전이학습 + 양자화 |
|---|---|


| Day 20 | Hailo-10 + AI 추론 최적화 + 최종 통합 데모 |
|---|---|


| 단계 | 내용 |
|---|---|
| 1. 모델 등록 | hailo_model_zoo 에 커스텀 모델 YAML 설정 파일 작성 |
| 2. 컴파일러 파라미터 | 배치 크기, 최적화 레벨 (0–2), 캘리브레이션 이터레이션 수 |
| 3. 캘리브레이션 데이터셋 | PTQ 정확도에 직결 — 실제 환경 이미지 최소 100장 권장 |
| 4. .hef 프로파일링 | hailortcli run 으로 레이턴시 / 처리량 측정 |
| 5. 커스텀 전처리 연결 | HailoRT Python API 전처리 콜백 등록 방법 |
| 6. 참고 문서 | https://hailo.ai/developer-zone/ (Hailo Developer Zone) |
