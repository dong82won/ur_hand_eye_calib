from pathlib import Path

import cv2
import numpy as np
import yaml

# 프로젝트 최상위 루트 경로 고정 (ur_hand_eye_calib/)
PROJECT_ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = PROJECT_ROOT / "config" / "settings.yaml"
DATA_DIR = PROJECT_ROOT / "data"


def load_config(config_path: Path) -> dict:
    if not config_path.exists():
        raise FileNotFoundError(f"설정 파일을 찾을 수 없습니다: {config_path}")
    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def main() -> None:
    cfg = load_config(CONFIG_PATH)

    cols = int(cfg["checkerboard"]["cols"])
    rows = int(cfg["checkerboard"]["rows"])
    square_mm = float(cfg["checkerboard"]["square_size_m"]) * 1000.0
    dpi = 300

    px_per_mm = dpi / 25.4
    a4_w_px = round(297.0 * px_per_mm)
    a4_h_px = round(210.0 * px_per_mm)
    sq_px = round(square_mm * px_per_mm)

    num_x, num_y = cols + 1, rows + 1
    board_w, board_h = num_x * sq_px, num_y * sq_px

    if board_w > a4_w_px or board_h > a4_h_px:
        raise ValueError(
            f"체크보드 크기({board_w}x{board_h}px)가 A4 용지({a4_w_px}x{a4_h_px}px)를 초과합니다."
        )

    # 흰색 A4 배경 생성 (가로 방향)
    canvas = np.full((a4_h_px, a4_w_px), 255, dtype=np.uint8)
    start_x = (a4_w_px - board_w) // 2
    start_y = (a4_h_px - board_h) // 2

    for r in range(num_y):
        for c in range(num_x):
            if (r + c) % 2 == 0:
                x1 = start_x + c * sq_px
                y1 = start_y + r * sq_px
                canvas[y1 : y1 + sq_px, x1 : x1 + sq_px] = 0

    DATA_DIR.mkdir(parents=True, exist_ok=True)
    output_path = DATA_DIR / f"checkerboard_a4_{cols}x{rows}_{square_mm:g}mm.png"
    cv2.imwrite(str(output_path), canvas)
    print(f"저장 완료: {output_path} ({cols}x{rows} corners, {square_mm:g}mm)")


if __name__ == "__main__":
    main()
