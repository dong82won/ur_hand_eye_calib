# UR Hand-in-Eye Calibration (with Intel RealSense)

이 프로젝트는 **유니버설로봇(Universal Robots, UR)** 과 **Intel RealSense 카메라**를 이용한 Hand-in-Eye(Eye-in-Hand) 캘리브레이션을 수행하는 Python 패키지입니다.
로봇의 끝단(Flange)과 카메라 사이의 $4 \times 4$ 변환 행렬(`T_flange_to_cam`)을 계산하여 로봇 비전 시스템의 정확한 3D 좌표계를 구축합니다.

## ✨ 주요 기능
- **A4 정밀 체크보드 생성**: 설정 파일에 맞춘 왜곡 없는 인쇄용 체크보드 이미지(300 DPI) 자동 생성
- **실시간 무결성 검증**:
  - UR 로봇과의 통신 상태(RTDE) 실시간 모니터링
  - 활성 TCP 오프셋 `(0, 0, 0, 0, 0, 0)` 여부 자동 검증 (순수 플랜지 기준 캘리브레이션 강제)
  - 로봇 이동 중(`MOVING`) 캡처 방지 및 안전장치
- **손쉬운 데이터 수집**: GUI 화면 오버레이를 통해 수집 상태, 로봇 좌표, 정지 여부를 실시간으로 확인하며 키보드로 손쉽게 제어
- **다양한 캘리브레이션 알고리즘 지원**: OpenCV 기반의 TSAI, PARK, HORAUD 등 여러 알고리즘 선택 가능 및 오차 검증(표준편차) 리포트 출력

## 📂 프로젝트 구조

```text
ur-hand-eye-calib/
├── config/
│   └── settings.yaml             # IP, 카메라 해상도, 체크보드 규격 설정
├── data/
│   ├── image/                    # 수집된 각 포즈의 실시간 카메라 캡처 이미지 (옵션)
│   ├── checkerboard_a4_*.png     # 자동 생성된 인쇄용 체크보드 이미지
│   ├── hand_eye_data.npz         # 수집된 로봇-카메라 포즈 쌍 및 카메라 파라미터 원본 백업
│   └── T_flange_to_cam.npy       # 최종 산출된 4x4 변환 행렬 (실제 Pick & Place 적용용)
├── src/ur_hand_eye_calib/
│   ├── collect_and_calib.py      # 실시간 데이터 수집 및 캘리브레이션 수행 스크립트
│   └── make_checkerboard.py      # 설정값 기반 체크보드 이미지 생성 스크립트
├── pyproject.toml / uv.lock      # uv 패키지 매니저 설정 파일
└── README.md
```

## 🛠 환경 설정 및 준비

### 1. 소프트웨어 의존성 설치
이 프로젝트는 빠르고 고립된 환경 구성을 위해 `uv` 패키지 매니저를 사용합니다.
```bash
# 프로젝트 디렉터리에서 필수 패키지 설치 및 가상환경 동기화
uv sync
```

### 2. 하드웨어 설정 (UR 로봇)
- **네트워크 연결**: PC와 UR 로봇 컨트롤러(이더넷 포트)가 동일한 네트워크 대역에 있어야 합니다. (기본 설정: `192.168.0.150`)
- **RTDE 활성화**: 티치 펜던트의 `설정 -> 보안 -> 서비스`에서 **RTDE**를 활성화합니다.
- **TCP 초기화**: 티치 펜던트의 `설치 -> TCP`에서 활성 TCP 설정값이 모두 `0` (X, Y, Z, RX, RY, RZ = 0)으로 되어 있어야 합니다.

### 3. 환경 설정 (`config/settings.yaml`)
실제 환경에 맞게 `settings.yaml` 파일을 수정합니다.
```yaml
robot:
  ip: "192.168.0.150"     # UR 로봇 IP
checkerboard:
  cols: 8                 # 내부 코너 가로 개수
  rows: 5                 # 내부 코너 세로 개수
  square_size_m: 0.030    # 사각형 한 칸의 실제 길이 (m)
```

## 🚀 사용 방법

### Step 1: 캘리브레이션 보드 생성 및 인쇄
설정된 규격에 맞는 A4 사이즈 캘리브레이션 보드를 생성합니다.
```bash
uv run src/ur_hand_eye_calib/make_checkerboard.py
```
- `data/` 폴더에 생성된 `.png` 이미지를 **'실제 크기(비율 100%)'**로 인쇄한 뒤, 평평한 판에 부착하여 로봇 작업 영역 내에 고정합니다.

### Step 2: Hand-in-Eye 데이터 수집 및 캘리브레이션
RealSense 카메라와 UR 로봇을 연결하여 캘리브레이션을 진행합니다.
```bash
uv run src/ur_hand_eye_calib/collect_and_calib.py
```
카메라 창이 열리면 로봇을 **수동 조작(Freedrive)**하여 체크보드를 다양한 각도와 위치(Roll, Pitch, Yaw 변화 필수)에서 바라보게 합니다.

**[단축키 안내]**
- `SPACE` : 현재 포즈의 로봇 좌표와 카메라 영상 데이터 저장 (로봇이 완전히 정지(`UR STOP`)했을 때만 작동)
- `D` : 직전에 잘못 찍힌 데이터 1개 취소 및 삭제
- `Q` : 수집을 종료하고 캘리브레이션 계산 수행 (최소 10개 이상 권장, 15~20개 수집 시 정밀도 상승)
- `ESC` : 저장하지 않고 프로그램 강제 종료

### Step 3: 결과 확인
`Q`를 눌러 수집을 종료하면 터미널에 계산 결과가 출력됩니다.
- **산출물**: `data/T_flange_to_cam.npy`
- **정밀도 검증**: 각 포즈별로 계산된 체크보드 원점의 3D 좌표 오차(표준편차)가 출력됩니다. 이 값이 1~2mm 이내라면 캘리브레이션이 성공적으로 완료된 것입니다.
