import socket
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import pyrealsense2 as _rs
import rtde_receive
import yaml

rs: Any = _rs

# 프로젝트 최상위 루트 경로 고정 (ur_hand_eye_calib/)
PROJECT_ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = PROJECT_ROOT / "config" / "settings.yaml"
DATA_DIR = PROJECT_ROOT / "data"
IMAGE_DIR = DATA_DIR / "image"

CALIB_METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def load_config(config_path: Path) -> dict:
    if not config_path.exists():
        raise FileNotFoundError(f"설정 파일을 찾을 수 없습니다: {config_path}")
    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def main():
    cfg = load_config(CONFIG_PATH)

    ur_ip = cfg["robot"]["ip"]
    cam_w = cfg["camera"]["width"]
    cam_h = cfg["camera"]["height"]
    cam_fps = cfg["camera"]["fps"]
    save_images = cfg["camera"]["save_images"]

    cb_cols = cfg["checkerboard"]["cols"]
    cb_rows = cfg["checkerboard"]["rows"]

    square_size = float(cfg["checkerboard"]["square_size_m"])
    checkerboard_size = (cb_cols, cb_rows)

    method_name = cfg["calibration"]["method"].upper()
    if method_name not in CALIB_METHODS:
        raise ValueError(
            f"지원하지 않는 알고리즘입니다: {method_name} (선택 가능: {list(CALIB_METHODS.keys())})"
        )
    calib_method = CALIB_METHODS[method_name]
    min_samples = cfg["calibration"]["min_samples"]

    # 저장 디렉터리 생성
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    if save_images:
        IMAGE_DIR.mkdir(parents=True, exist_ok=True)

    # ==================================================
    # 1. UR 로봇 연결 및 초기 무결성 검증 (미연결 시 즉시 종료)
    # ==================================================
    print(f"[1/2] UR 로봇({ur_ip}) 연결 중...")
    try:
        with socket.create_connection((ur_ip, 30004), timeout=2.0):
            pass
        rtde_r = rtde_receive.RTDEReceiveInterface(ur_ip)
    except (OSError, RuntimeError) as e:
        print(f"[오류] UR 로봇({ur_ip}:30004)에 연결할 수 없어 프로그램을 종료합니다.")
        print(f"       상세 원인: {e}")
        sys.exit(1)

    if not rtde_r.isConnected():
        raise ConnectionError(f"UR로봇({ur_ip})과 RTDE 연결에 실패했습니다.")

    # 활성 TCP가 (0, 0, 0, 0, 0, 0)인지 검증 (순수 플랜지 기준 확인)
    tcp_offset = np.array(rtde_r.getTCPOffset(), dtype=np.float64)
    if not np.allclose(tcp_offset, 0.0, atol=1e-5):
        rtde_r.disconnect()
        raise ValueError(
            f"활성 TCP가 0이 아닙니다. {np.round(tcp_offset, 5)}\n"
            "티치 팬던트 [설치 -> TCP]에서 모든 값이 0인 TCP를 활성화 해야 한다!!"
        )
    print(f"UR 연결 확인 완료 (초기 Pose: {np.round(rtde_r.getActualTCPPose(), 4)})")

    # ==================================================
    # 2. RealSense 카메라 초기화
    # ==================================================
    print("[2/2] RealSense 카메라 초기화 중...")
    pipeline = rs.pipeline()
    rs_config = rs.config()
    rs_config.enable_stream(rs.stream.color, cam_w, cam_h, rs.format.bgr8, cam_fps)

    try:
        profile = pipeline.start(rs_config)
    except RuntimeError as e:
        rtde_r.disconnect()
        print(
            f"[오류] RealSense 카메라를 시작할 수 없어 프로그램을 종료합니다.\n       상세 원인: {e}"
        )
        sys.exit(1)

    color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = color_stream.get_intrinsics()

    camera_matrix = np.array(
        [[intr.fx, 0.0, intr.ppx], [0.0, intr.fy, intr.ppy], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    dist_coeffs = np.array(intr.coeffs, dtype=np.float64)

    # 체크보드 3D 월드 좌표 생성 (Z=0 평면)
    objp = np.zeros((cb_cols * cb_rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cb_cols, 0:cb_rows].T.reshape(-1, 2)
    objp *= square_size

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    R_gripper2base_list = []
    t_gripper2base_list = []
    R_target2cam_list = []
    t_target2cam_list = []

    print("\n==================================================")
    print(" [SPACE] : 현재 포즈 수집 (로봇 완전 정지 후 입력)")
    print(" [D]     : 직전 수집 데이터 1개 삭제")
    print(" [Q]     : 수집 종료 및 캘리브레이션 계산")
    print(" [ESC]   : 저장하지 않고 강제 종료")
    print("==================================================\n")

    abort = False
    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            img = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            display_img = img.copy()

            ret, corners = cv2.findChessboardCorners(
                gray,
                checkerboard_size,
                flags=(
                    cv2.CALIB_CB_ADAPTIVE_THRESH
                    + cv2.CALIB_CB_FAST_CHECK
                    + cv2.CALIB_CB_NORMALIZE_IMAGE
                ),
            )

            corners_sub = None
            if ret:
                corners_sub = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1), criteria
                )
                cv2.drawChessboardCorners(
                    display_img, checkerboard_size, corners_sub, ret
                )

            ur_connected = rtde_r.isConnected()
            ur_pose = rtde_r.getActualTCPPose() if ur_connected else [0.0] * 6
            ur_speed = rtde_r.getActualTCPSpeed() if ur_connected else [0.0] * 6
            is_stationary = float(np.linalg.norm(ur_speed)) < 1e-4

            # 상태 오버레이 표시
            status_color = (0, 255, 0) if ret else (0, 0, 255)
            status_text = "Board: DETECTED" if ret else "Board: NOT FOUND"
            cv2.putText(
                display_img,
                f"Captured: {len(R_gripper2base_list)} | {status_text}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                status_color,
                2,
            )
            cv2.putText(
                display_img,
                "[SPACE]: Capture | [D]: Undo | [Q]: Calibrate | [ESC]: Quit",
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 0),
                2,
            )
            ur_text = (
                f"UR: X={ur_pose[0]:.3f} Y={ur_pose[1]:.3f} Z={ur_pose[2]:0.3f} | "
                f"{'UR STOP' if is_stationary else 'UR MOVING'}"
                if ur_connected
                else "UR: DISCONNETED"
            )

            cv2.putText(
                display_img,
                ur_text,
                (20, 105),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255) if (ur_connected and is_stationary) else (0, 0, 255),
                2,
            )

            cv2.imshow("UR Hand-in-Eye Calibration", display_img)
            key = cv2.waitKey(1) & 0xFF

            # [SPACE] 데이터 캡처
            if key == ord(" "):
                if not ur_connected:
                    print("[오류] UR 로봇 연결이 끊어졌습니다. 네트워크를 확인하세요.")
                    continue
                if not is_stationary:
                    print(
                        "[경고] 로봇이 아직 움직이고 있습니다. 완전히 정지 한 후 다시 누르세요."
                    )
                    continue
                if not ret or corners_sub is None:
                    print("[경고] 체크보드가 인식되지 않았습니다.")
                    continue

                # 캡처 순간이 최신 UR 끝단 좌표 읽기: [x, y, z, rx, ry, rz] (m, rad)
                latest_pose = rtde_r.getActualTCPPose()
                t_g2b = np.array(latest_pose[:3], dtype=np.float64).reshape(3, 1)
                rvec_g2b = np.array(latest_pose[3:], dtype=np.float64).reshape(3, 1)
                R_g2b, _ = cv2.Rodrigues(rvec_g2b)

                success, rvec_t2c, tvec_t2c = cv2.solvePnP(
                    objp,
                    corners_sub,
                    camera_matrix,
                    dist_coeffs,
                    flags=cv2.SOLVEPNP_ITERATIVE,
                )
                if success:
                    R_t2c, _ = cv2.Rodrigues(rvec_t2c)

                    R_gripper2base_list.append(R_g2b)
                    t_gripper2base_list.append(t_g2b)
                    R_target2cam_list.append(R_t2c)
                    t_target2cam_list.append(tvec_t2c)

                    idx = len(R_gripper2base_list)
                    if save_images:
                        img_file = IMAGE_DIR / f"pose_{idx:02d}.png"
                        cv2.imwrite(str(img_file), img)
                    print(
                        f"[{idx:02d}번 수집]UR Pos(m): {np.round(latest_pose[:3], 4)} | Rot(rad): {np.round(latest_pose[3:], 4)}"
                    )

            # [D] 직전 데이터 삭제
            elif key in (ord("d"), ord("D")):
                if R_gripper2base_list:
                    idx = len(R_gripper2base_list)
                    R_gripper2base_list.pop()
                    t_gripper2base_list.pop()
                    R_target2cam_list.pop()
                    t_target2cam_list.pop()

                    if save_images:
                        img_file = IMAGE_DIR / f"pose_{idx:02d}.png"
                        if img_file.exists():
                            img_file.unlink()

                    print(
                        f"[삭제] {idx:02d}번 데이터가 취소되었습니다. (현재 {len(R_gripper2base_list)}개)"
                    )
                else:
                    print("[안내] 삭제할 데이터가 없습니다.")

            # [Q] 수집 종료 및 계산 진행
            elif key in (ord("q"), ord("Q")):
                break

            # [ESC] 강제 종료
            elif key == 27:
                print("\n사용자에 의해 작업이 취소되었습니다.")
                abort = True
                break

    finally:
        pipeline.stop()
        rtde_r.disconnect()
        cv2.destroyAllWindows()

    if abort:
        return

    num_samples = len(R_gripper2base_list)
    if num_samples < min_samples:
        print(
            f"\n[오류] 수집된 포즈({num_samples}개)가 최소 기준({min_samples}개)보다 부족합니다."
        )
        return

    # 3. Hand-in-Eye 캘리브레이션 계산
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base=R_gripper2base_list,
        t_gripper2base=t_gripper2base_list,
        R_target2cam=R_target2cam_list,
        t_target2cam=t_target2cam_list,
        method=calib_method,
    )

    T_flange_to_cam = np.eye(4, dtype=np.float64)
    T_flange_to_cam[:3, :3] = R_cam2gripper
    T_flange_to_cam[:3, 3] = t_cam2gripper.flatten()

    # 4. 결과 및 원본 데이터 저장 (.npz, .npy)
    npz_path = DATA_DIR / "hand_eye_data.npz"
    npy_path = DATA_DIR / "T_flange_to_cam.npy"

    np.savez(
        npz_path,
        R_g2b=np.array(R_gripper2base_list),
        t_g2b=np.array(t_gripper2base_list),
        R_t2c=np.array(R_target2cam_list),
        t_t2c=np.array(t_target2cam_list),
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
    )
    np.save(npy_path, T_flange_to_cam)

    print("\n==================================================")
    print(f" 캘리브레이션 완료 (Method: {method_name}, Samples: {num_samples})")
    print("==================================================")
    print("T_flange_to_cam (4x4 Matrix, 단위: m):")
    print(np.array2string(T_flange_to_cam, precision=6, suppress_small=True))
    print(f"\n- 원본 데이터 저장: {npz_path}")
    print(f"- 변환 행렬 저장:   {npy_path}")

    # 5. Base 기준 체크보드 원점 일관성 오차 검증
    target_positions = []
    for i in range(num_samples):
        T_base_to_flange = np.eye(4)
        T_base_to_flange[:3, :3] = R_gripper2base_list[i]
        T_base_to_flange[:3, 3] = t_gripper2base_list[i].flatten()

        T_cam_to_target = np.eye(4)
        T_cam_to_target[:3, :3] = R_target2cam_list[i]
        T_cam_to_target[:3, 3] = t_target2cam_list[i].flatten()

        T_base_to_target = T_base_to_flange @ T_flange_to_cam @ T_cam_to_target
        target_positions.append(T_base_to_target[:3, 3])

    target_positions = np.array(target_positions)
    mean_pos = np.mean(target_positions, axis=0)
    std_mm = np.std(target_positions, axis=0) * 1000.0
    euclidean_err_mm = np.linalg.norm(target_positions - mean_pos, axis=1) * 1000.0

    print("\n[정밀도 검증] Base 기준 체크보드 원점 편차:")
    print(
        f"- 축별 표준편차: X={std_mm[0]:.3f} mm, Y={std_mm[1]:.3f} mm, Z={std_mm[2]:.3f} mm"
    )
    print(
        f"- 평균 위치 오차: {np.mean(euclidean_err_mm):.3f} mm (최대 오차: {np.max(euclidean_err_mm):.3f} mm)"
    )


if __name__ == "__main__":
    main()
