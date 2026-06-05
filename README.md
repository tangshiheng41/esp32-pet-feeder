# ESP32-S3 Smart Pet Feeder

基于 **ESP32-S3 DevKitC-1** 的智能宠物喂食器：使用 **OV2640** 摄像头 + **Edge Impulse** 视觉识别，配合 **HX711** 称重与 **MG996R** 舵机实现按需投喂，并通过 **Web 页面** 实时查看识别结果与喂食状态。

## 功能概览

- AI 视觉识别：Edge Impulse 模型推理，识别目标宠物类别与置信度
- 智能喂食：缺粮 + 识别到宠物 + 未超限时自动开舵机投喂
- 重量监测：HX711 实时读取食盆重量，达到目标重量或超时自动停止
- Web 监控：连接 Wi-Fi 后，浏览器访问设备 IP，查看标签、置信度、重量、喂食状态
- 舵机测试：Web 页面一键测试舵机动作

## 硬件清单

| 模块 | 型号/说明 |
|------|-----------|
| 主控 | ESP32-S3-DevKitC-1（N16R8） |
| 摄像头 | OV2640（自带晶振，无 XCLK 引脚） |
| 称重 | HX711 + 称重传感器（Load Cell） |
| 执行器 | MG996R 舵机（出粮闸门） |
| 电源 | ESP32：USB 5V；舵机：**独立 5V 电源**（必须与 ESP32 共地） |

## 快速开始

### 1. 安装依赖库

在 Arduino IDE 中安装：

- `esp32` 开发板支持包（Espressif）
- `esp32-camera`
- `HX711`（by Bogdan Necula）
- `ESP32Servo`

### 2. 导入 Edge Impulse 模型库

1. 在 [Edge Impulse](https://edgeimpulse.com/) 训练并导出 Arduino 库
2. 将导出的库文件夹放到 Arduino `libraries/` 目录，命名为 `pet_feeder_inferencing`
3. 确保 `#include <pet_feeder_inferencing.h>` 可正常编译

> 模型库体积较大，未包含在本仓库中。请自行导出并放置。

### 3. 配置 Wi-Fi

```bash
cp firmware/Final/secrets.h.example firmware/Final/secrets.h
```

编辑 `firmware/Final/secrets.h`，填入你的 Wi-Fi 名称和密码。

### 4. Arduino IDE 设置

| 选项 | 推荐值 |
|------|--------|
| Board | ESP32S3 Dev Module |
| USB CDC On Boot | Enabled |
| PSRAM | OPI PSRAM（若板子带 PSRAM） |
| Partition Scheme | 按模型大小选择（模型大时用更大 App 分区） |
| Upload Speed | 921600（若不稳定可降到 460800） |

### 5. 编译与烧录

1. 打开 `firmware/Final/Final.ino`
2. 编译并上传到开发板
3. 打开串口监视器（115200 baud）
4. 记录串口输出的 IP 地址，浏览器访问 `http://<设备IP>/`

## Web 接口

| 路径 | 说明 |
|------|------|
| `/` | 监控页面（标签、置信度、重量、喂食状态） |
| `/metrics` | JSON 格式实时数据 |
| `/testServo` | 触发舵机测试（喂食进行中会返回 409） |

`/metrics` 返回示例：

```json
{
  "label": "pet front",
  "confidence": 0.912345,
  "petDetected": true,
  "weight_g": 35.2,
  "feedState": "IDLE",
  "cooldownRemaining": 0,
  "ts": 123456
}
```

## 接线说明

完整接线表见 [docs/wiring.md](docs/wiring.md)。  
系统框图见 [docs/circuit_block.svg](docs/circuit_block.svg)。

## 关键参数（可在代码中修改）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `TARGET_LABEL` | `"pet front"` | Edge Impulse 目标类别名 |
| `CONFIDENCE_THRESHOLD` | `0.78` | 触发喂食的最低置信度 |
| `LOW_FOOD_THRESHOLD` | `20.0` g | 低于此值视为缺粮 |
| `FULL_FOOD_THRESHOLD` | `500.0` g | 高于此值禁止喂食 |
| `TARGET_FEED_WEIGHT` | `100.0` g | 单次投喂目标重量 |
| `HX711_CALIBRATION_FACTOR` | `430.0` | 称重标定系数 |

## 项目结构

```text
esp32-pet-feeder/
├── README.md
├── LICENSE
├── .gitignore
├── docs/
│   ├── wiring.md
│   └── circuit_block.svg
└── firmware/
    └── Final/
        ├── Final.ino          # 主程序（Web + AI + 喂食）
        ├── secrets.h.example  # Wi-Fi 配置模板
        └── secrets.h          # 本地配置（勿提交）
```

## 常见问题

### 编译报错 `FRAMESIZE_QQVGA2 was not declared`

部分 `esp32-camera` 版本没有 `FRAMESIZE_QQVGA2`，请改用 `FRAMESIZE_QQVGA`。

### 摄像头初始化失败

- 检查并口线与 SCCB（SDA/SCL）是否接对
- 确认 3.3V 供电与 GND 共地
- 本模块无 XCLK 引脚，代码中 `XCLK_GPIO_NUM = -1`

### 编译/烧录非常慢

Edge Impulse 模型库较大，首次编译和上传会较慢。可考虑导出更小输入尺寸（如 96×96）的模型。

### 舵机动作时 ESP32 重启

舵机必须使用**独立 5V 电源**，并与 ESP32 **共地**。不要从开发板 5V 引脚直接带 MG996R。

### 重量读数不准

使用标准砝码重新标定 `HX711_CALIBRATION_FACTOR`，并在空载时执行 `tare()`。

## 演示流程建议

1. 上电后查看串口 `[SELF CHECK]` 是否 Camera / HX711 / Servo / WiFi 均为 OK
2. 浏览器打开设备 IP，确认识别标签与置信度刷新
3. 点击 **Test Servo** 验证舵机
4. 放置宠物模型或真实宠物，观察 `petDetected` 与自动喂食逻辑

## 许可证

MIT License — 见 [LICENSE](LICENSE)

## 发布到 Gitee / GitHub

见 [docs/publish-gitee.md](docs/publish-gitee.md)

## 致谢

- [Edge Impulse](https://edgeimpulse.com/) — 嵌入式 ML 推理
- [Espressif esp32-camera](https://github.com/espressif/esp32-camera) — OV2640 驱动
