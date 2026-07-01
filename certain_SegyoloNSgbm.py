import cv2
import numpy as np
import torch
from ultralytics import YOLO
import time
from WIFI_TCP import ESP32Sender
import csv
import time
# ESP地址设定和传输时间
esp_sender = ESP32Sender(ip="192.168.4.1", port=80)
last_send_time = 0
send_interval = 0.5# 每 0.5 秒发送一次，避免过度占用 ESP32 资源

# 确保使用 GPU
device = torch.device('cuda')

# 1. 加载并深度优化模型
model = YOLO('best.pt')
model.to(device)
model.fuse()
model.half()  # 启用 FP16 半精度推理


# 使用启动时间命名文件，确保不会覆盖旧记录
start_time_str = time.strftime("%Y%m%d_%H%M%S")
csv_filename = f"health_data_{start_time_str}.csv"

# 创建文件并写入表头
with open(csv_filename, mode='w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['Local_Time', 'BPM', 'FALL', 'OBJ', 'DIST'])

print(f"数据将保存至: {csv_filename}")
# 相机参数 (保持不变)
left_camera_matrix = np.array([[4.892808363373980e+02, 0, 3.054519252318725e+02],
                               [0, 4.902054830959276e+02, 2.498290257795890e+02],
                               [0., 0., 1.]])
right_camera_matrix = np.array([[4.926937454900473e+02, 0, 3.142384415244637e+02],
                                [0, 4.934942810655972e+02, 2.366664786568870e+02],
                                [0., 0., 1.]])
left_distortion = np.array([[-0.038822986381982, 0.041116015873942, 0.003124788606114, -0.001709954151797,0]])
right_distortion = np.array([[-0.074650854569648,0.171902588520910, 0.001691140404196, 0.006054018780774, 0]])
R = np.array([[0.999743840019977,0.012758688618252, -0.018694122254138],
              [-0.012632182492944, 0.999896612292648, 0.006869693672254],
              [0.018779837794164,-0.006631786367800, 0.999801648879415]])
T = np.array([-58.785659219025874,0.946941156318870, 0.708340796287995])
size = (640, 480)

# 立体矫正映射
R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(
    left_camera_matrix, left_distortion, right_camera_matrix, right_distortion, size, R, T, alpha=0
)
left_map1, left_map2 = cv2.initUndistortRectifyMap(left_camera_matrix, left_distortion, R1, P1, size, cv2.CV_16SC2)
right_map1, right_map2 = cv2.initUndistortRectifyMap(right_camera_matrix, right_distortion, R2, P2, size, cv2.CV_16SC2)

# 创建 SGBM
stereo = cv2.StereoSGBM_create(minDisparity=1,
                               numDisparities=128,
                               blockSize=5,
                               P1=600,
                               P2=2400,
                               mode=cv2.STEREO_SGBM_MODE_HH)

capture = cv2.VideoCapture(0)
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

window_displayed = False
start_time_program = time.time()

while True:
    start_time = time.time()
    ret, frame = capture.read()
    if not ret: break

    # 1. 图像切割与矫正
    frame1, frame2 = frame[0:480, 0:640], frame[0:480, 640:1280]
    img1_rect = cv2.remap(frame1, left_map1, left_map2, cv2.INTER_LINEAR)
    img2_rect = cv2.remap(frame2, right_map1, right_map2, cv2.INTER_LINEAR)

    # 裁剪并对齐
    x1, y1, w1, h1 = validPixROI1
    img1_rect = img1_rect[y1:y1 + h1, x1:x1 + w1]
    img2_rect = img2_rect[y1:y1 + h1, x1:x1 + w1]

    # 2. 视差计算 (CPU)
    imgL, imgR = cv2.cvtColor(img1_rect, cv2.COLOR_BGR2GRAY), cv2.cvtColor(img2_rect, cv2.COLOR_BGR2GRAY)
    disparity = stereo.compute(imgL, imgR)

    # --- 深度图可视化处理 ---
    # 归一化到 0-255 并转换为 8位图
    # --- 黑白深度图可视化处理 ---
    # 1. 归一化：将原始视差值映射到 0-255 的灰度范围
    # 注意：disparity 越大代表物体越近，因此 255（白）对应近处，0（黑）对应远处
    disp_gray = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    # 2. 噪点处理：使用中值滤波或高斯滤波让深度图看起来更平滑
    disp_gray = cv2.GaussianBlur(disp_gray, (3, 3), 0)

    # 3. 无效区域遮盖：将无法匹配的区域（视差<=0）强制设为纯黑
    disp_gray[disparity <= 0] = 0


    # 3. 三维重建 (送入 GPU)
    threeD = cv2.reprojectImageTo3D(disparity, Q) * 16.0
    threeD_gpu = torch.from_numpy(threeD).to(device).half()

    # 4. YOLO 预处理与推理
    h_rect, w_rect = img1_rect.shape[:2]
    pad_t, pad_l = (640 - h_rect) // 2, (640 - w_rect) // 2
    img1_padded = cv2.copyMakeBorder(img1_rect, pad_t, 640 - h_rect - pad_t, pad_l, 640 - w_rect - pad_l,
                                     cv2.BORDER_REPLICATE)# 做边界填充

    img_tensor = torch.from_numpy(img1_padded).permute(2, 0, 1).unsqueeze(0).to(device).half() / 255.0
    results = model.predict(img_tensor, verbose=False, half=True)

    annotated_frame = results[0].plot()
    annotated_frame = annotated_frame[pad_t:pad_t + h_rect, pad_l:pad_l + w_rect]

    # 初始化最小距离和最小距离人物
    min_distance = float('inf')
    min_d_good = "None"


    # 5. GPU 深度逻辑运算
    if results[0].masks is not None:
        masks_gpu = results[0].masks.data
        for i in range(len(masks_gpu)): # 布尔化
            mask = masks_gpu[i][pad_t:pad_t + h_rect, pad_l:pad_l + w_rect] > 0.5
            z_values = threeD_gpu[:, :, 2][mask]
            valid_z = z_values[torch.isfinite(z_values) & (z_values > 0)] # 排除无效值
            # 平均深度
            if valid_z.numel() > 0:
                avg_depth = torch.mean(valid_z).item() / 1000.0
                label = model.names[int(results[0].boxes.cls[i])]

                if avg_depth < min_distance:
                    min_distance = avg_depth
                    min_d_good = label

                coords = torch.nonzero(mask) #索引坐标
                if coords.numel() > 0:
                    y_c, x_c = torch.mean(coords.float(), dim=0).int().tolist()
                    cv2.putText(annotated_frame, f"{label}: {avg_depth:.2f}m", (x_c, y_c),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if min_d_good != "None" and min_distance != float('inf'):
            print(f"最近物体:{min_d_good} and {min_distance:.2f}m")
            current_time = time.time()
            if current_time - last_send_time > send_interval:
                resp = esp_sender.send_detection_data(min_d_good, min_distance)

                if resp:  # 如果成功接收到回传数据
                    last_send_time = current_time
                    print(f">>> 收到回传并存入CSV")

                    # --- 解析回传字符串并写入 CSV ---
                    try:
                        # 按照冒号和逗号切割字符串
                        # 示例格式: BPM:75,FALL:0.00,OBJ:p,DIST:1.25
                        parts = {item.split(':')[0]: item.split(':')[1] for item in resp.split(',')}

                        with open(csv_filename, mode='a', newline='') as f:
                            writer = csv.writer(f)
                            writer.writerow([
                                time.strftime('%H:%M:%S'),  # 电脑本地时间
                                parts.get('BPM', '0'),
                                parts.get('FALL', '0.00'),
                                min_d_good,
                                f"{min_distance:.2f}"
                            ])
                    except Exception as e:
                        print(f"解析回传数据失败: {e}")
        else:
            print(f"没找到任何东西")

    # 6. 显示与帧率
    fps = 1.0 / (time.time() - start_time)
    cv2.putText(annotated_frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    cv2.imshow('YOLO Detection', annotated_frame)
    cv2.imshow('Depth Map Visual', disp_gray)  # 显示灰色 色深度图

    if not window_displayed:
        window_displayed = True
        print(f"Total startup time: {time.time() - start_time_program:.2f}s")

    if cv2.waitKey(1) & 0xFF == ord('q'): break

capture.release()
cv2.destroyAllWindows()