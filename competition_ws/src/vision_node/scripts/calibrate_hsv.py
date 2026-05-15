#!/usr/bin/env python3
"""
HSV 阈值实时标定工具

用法:
  python calibrate_hsv.py                          # USB 摄像头
  python calibrate_hsv.py --image test.jpg          # 静态图片
  python calibrate_hsv.py --camera 2                # 指定摄像头编号

操作:
  滑动 trackbar 调整 HSV 上下限，观察右侧二值化结果
  满意后按 's' 保存到 color_params.yaml
  按 'c' 切换颜色通道 (Red → Green → Blue)
  按 'q' 退出
"""

import cv2
import argparse
import os


def nothing(x):
    pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=0, help="摄像头编号")
    parser.add_argument("--image", type=str, default=None, help="静态图片路径")
    args = parser.parse_args()

    cap = None
    static_img = None
    if args.image:
        static_img = cv2.imread(args.image)
        if static_img is None:
            print(f"无法读取图片: {args.image}")
            return
        cv2.imshow("Original", static_img)
    else:
        cap = cv2.VideoCapture(args.camera)
        if not cap.isOpened():
            print(f"无法打开摄像头 {args.camera}")
            return

    window = "HSV Calibration"
    cv2.namedWindow(window)
    cv2.resizeWindow(window, 800, 480)

    # H_min, H_max, S_min, S_max, V_min, V_max
    # 红色特殊：需要两个 H 范围，切换时生效
    cv2.createTrackbar("H_lo", window, 0, 180, nothing)
    cv2.createTrackbar("H_hi", window, 180, 180, nothing)
    cv2.createTrackbar("S_lo", window, 70, 255, nothing)
    cv2.createTrackbar("S_hi", window, 255, 255, nothing)
    cv2.createTrackbar("V_lo", window, 50, 255, nothing)
    cv2.createTrackbar("V_hi", window, 255, 255, nothing)

    # 红色第二段
    cv2.createTrackbar("H_lo2", window, 170, 180, nothing)
    cv2.createTrackbar("H_hi2", window, 180, 180, nothing)

    colors = ["red", "green", "blue"]
    color_idx = 0

    # 预设阈值
    presets = {
        "red":   ([0, 100, 80], [10, 255, 255], [160, 100, 80], [180, 255, 255]),
        "green": ([40, 80, 60], [80, 255, 255], None, None),
        "blue":  ([95, 80, 60], [135, 255, 255], None, None),
    }

    def set_preset(color):
        p = presets[color]
        cv2.setTrackbarPos("H_lo",  window, p[0][0])
        cv2.setTrackbarPos("H_hi",  window, p[1][0])
        cv2.setTrackbarPos("S_lo",  window, p[0][1])
        cv2.setTrackbarPos("S_hi",  window, p[1][1])
        cv2.setTrackbarPos("V_lo",  window, p[0][2])
        cv2.setTrackbarPos("V_hi",  window, p[1][2])
        if p[2] is not None:
            cv2.setTrackbarPos("H_lo2", window, p[2][0])
            cv2.setTrackbarPos("H_hi2", window, p[3][0])

    set_preset("red")

    print("=" * 50)
    print("  HSV 阈值标定工具")
    print("  c = 切换颜色  s = 保存  q = 退出")
    print("=" * 50)

    while True:
        if static_img is not None:
            frame = static_img.copy()
        else:
            ret, frame = cap.read()
            if not ret:
                continue

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        h_lo  = cv2.getTrackbarPos("H_lo", window)
        h_hi  = cv2.getTrackbarPos("H_hi", window)
        s_lo  = cv2.getTrackbarPos("S_lo", window)
        s_hi  = cv2.getTrackbarPos("S_hi", window)
        v_lo  = cv2.getTrackbarPos("V_lo", window)
        v_hi  = cv2.getTrackbarPos("V_hi", window)
        h_lo2 = cv2.getTrackbarPos("H_lo2", window)
        h_hi2 = cv2.getTrackbarPos("H_hi2", window)

        lower = (h_lo, s_lo, v_lo)
        upper = (h_hi, s_hi, v_hi)

        mask = cv2.inRange(hsv, lower, upper)

        # 红色双范围
        if color_idx == 0:
            mask2 = cv2.inRange(hsv, (h_lo2, s_lo, v_lo), (h_hi2, s_hi, v_hi))
            mask = cv2.bitwise_or(mask, mask2)

        # 形态学降噪
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        result = cv2.bitwise_and(frame, frame, mask=mask)

        # 找到最大轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area > 100:
                x, y, w, h = cv2.boundingRect(largest)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, f"{area:.0f} px", (x, y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # 显示
        hstack = cv2.hconcat([frame, result])
        cv2.imshow(window, hstack)
        cv2.setWindowTitle(window, f"HSV Calibration — {colors[color_idx]}")

        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            color_idx = (color_idx + 1) % 3
            set_preset(colors[color_idx])
        elif key == ord('s'):
            save_params(colors[color_idx],
                        [h_lo, s_lo, v_lo], [h_hi, s_hi, v_hi],
                        [h_lo2, s_lo, v_lo] if color_idx == 0 else None,
                        [h_hi2, s_hi, v_hi] if color_idx == 0 else None)
            print(f"已保存 {colors[color_idx]} 阈值")

    cv2.destroyAllWindows()
    if cap:
        cap.release()


def save_params(color, lower, upper, lower2, upper2):
    """写入 color_params.yaml"""
    config_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "config", "color_params.yaml"
    )

    import yaml
    try:
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
    except Exception:
        config = {}

    config.setdefault(color, {})
    if color == "red" and lower2 is not None:
        config[color]["lower_1"] = lower
        config[color]["upper_1"] = upper
        config[color]["lower_2"] = lower2
        config[color]["upper_2"] = upper2
    else:
        config[color]["lower"] = lower
        config[color]["upper"] = upper

    with open(config_path, "w") as f:
        yaml.dump(config, f, default_flow_style=None, allow_unicode=True)

    print(f"  lower: {lower}")
    print(f"  upper: {upper}")
    if lower2 is not None:
        print(f"  lower2: {lower2}")
        print(f"  upper2: {upper2}")


if __name__ == "__main__":
    main()
