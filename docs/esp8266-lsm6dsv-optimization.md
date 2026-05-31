# ESP-12F ESP8266 + LSM6DSV 优化讨论

本文面向 ESP-12F/ESP8266 + LSM6DSV 的 SlimeVR Tracker 方案，聚焦功耗、性能、稳定性以及三者之间的取舍。结论基于当前代码树和公开硬件资料，偏向实际可落地的改动顺序。

## 当前基线

- ESP8266 构建环境使用 `esp12e` board，适合 ESP-12F 这一类 ESP8266 模组；默认 PlatformIO 环境是 `BOARD_WEMOSD1MINI`。
- LSM6DSV 当前走 soft-fusion，不走 IMU 内部 DMP。驱动配置为 4 g accel、1000 dps gyro、HAODR table 1、gyro 240 Hz、accel 120 Hz、temperature 60 Hz，FIFO 连续模式，每次最多读 16 条 FIFO entry。
- soft-fusion 层按 100 Hz 节奏读取 FIFO、更新 VQF、发送旋转和线性加速度。当前默认启用 `OPTIMIZE_UPDATES`，并使用 buffered packet bundling。
- I2C 默认 400 kHz。ESP8266EX 数据手册把 I2C 描述为软件实现且标称最高 100 kHz，因此这里的 400 kHz 应视为需要板级验证的实用配置，而不是保守规格。ESP8266 默认电源策略是 `POWER_SAVING_LEGACY`，代码注释明确指出 `POWER_SAVING_MINIMUM` 会造成 sporadic data pauses。
- `POWER_SAVING_MAXIMUM` 在 ESP8266 分支直接 `#error`，当前不能作为可用选项。

关键代码位置：

- `src/sensors/softfusion/drivers/lsm6dsv.h`: LSM6DSV ODR、量程、FIFO 配置。
- `src/sensors/softfusion/drivers/lsm6ds-common.h`: FIFO 状态读取、overrun 检测、每条 FIFO entry 解包。
- `src/sensors/softfusion/softfusionsensor.h`: 100 Hz 读取/融合/发送节奏。
- `src/debug.h`: `POWERSAVING_MODE`、`PACKET_BUNDLING`、`I2C_SPEED`、`OPTIMIZE_UPDATES`。
- `src/network/wifihandler.cpp`: ESP8266 modem/light sleep 映射。

## 优先级结论

1. 稳定性优先时，保留当前 240/120 Hz IMU ODR、100 Hz 输出和 `POWER_SAVING_LEGACY`，先强化 FIFO/监控与供电完整性。
2. 功耗优先时，主要收益来自 ESP8266 的 Wi-Fi、射频功率、稳压器、LED/USB 串口/电池分压等系统功耗；LSM6DSV 本身不是主要耗电点。
3. 性能优先时，当前 LSM6DSV 采样率对 100 Hz 输出有余量。进一步优化应先减少网络抖动和 FIFO backlog，而不是盲目提高 ODR。
4. 最大潜力但风险最高的方向是接入 LSM6DSV 的 SFLP/传感器侧融合，让 IMU 输出 quaternion 来降低 ESP8266 CPU 负担；这会触碰协议、校准和融合行为，适合独立实验分支。

## 可优化项与取舍

| 方向 | 建议 | 收益 | 代价/风险 | 优先级 |
| --- | --- | --- | --- | --- |
| FIFO 抗抖动 | 将 `MaxFifoReadings` 从 16 试到 24 或 32，并增加正常运行时 FIFO backlog/overrun 计数 | Wi-Fi 或主循环偶发停顿后更容易追上 FIFO | 单次 I2C burst 更长，局部栈占用增加；`readBytes` 参数是 `uint8_t`，不要让单次读取超过 255 bytes | 高 |
| FIFO 读取策略 | 如果 `fifo_bytes > bytes_to_read`，在时间预算内循环补读，或记录 backlog 并下一帧优先 drain | 减少 accumulated latency 和 overrun | 可能挤占网络处理时间；需要给 ESP8266 watchdog/yield 留空间 | 高 |
| I2C 可靠性 | 默认保持 400 kHz；若出现超时、FIFO overrun、姿态卡顿，先检查上拉/线长/地线，再用 `-DESP8266_I2C_SPEED=100000` 回退诊断 | 400 kHz 降低 bus time，适合 FIFO burst | 降速会增加每 10 ms 内 I2C 占用；过弱上拉在 400 kHz 下更容易出错 | 高 |
| Wi-Fi 电源策略 | `POWER_SAVING_MINIMUM` 只作为实测选项；`MODERATE`/`MAXIMUM` 不建议用于低延迟 tracker | 可能降低平均电流 | 会增加延迟、丢包、发现服务器失败或数据暂停风险；代码已标注问题 | 中 |
| RF 输出功率 | 在强信号场景小步降低输出功率，记录 RSSI、丢包和体感延迟 | 降低 TX 峰值和热量 | 信号边缘会更不稳定；人体遮挡时更明显 | 中 |
| Packet/数据内容 | 保持 buffered bundling；功耗模式可测试关闭 `SEND_ACCELERATION`；telemetry 默认关闭，需要 `-DENABLE_TELEMETRY=1` 显式启用 | 减少 UDP 包和 CPU/射频活动 | 失去线性加速度或诊断能力；telemetry 开启会增加网络流量和日志泄露面 | 中 |
| IMU ODR 降档 | 实验 120 Hz gyro / 60 或 120 Hz accel 的 profile | 降低 I2C、融合计算和传感器功耗 | 快速动作相位裕量下降，VQF timestep/校准必须同步；可能增大延迟和漂移 | 中 |
| LSM6DSV SFLP | 研究读取 sensor-fusion low-power quaternion，作为 soft-fusion 的替代输入 | 最大 CPU 省电潜力，可能降低 ESP8266 负载 | 集成复杂，需重做校准、坐标、协议和精度验证；不要直接替换稳定路径 | 低到中 |
| 编译优化 | 当前已经 `-O2`。可比较 `-Os`、LTO、关闭不需要的功能宏 | 可能降低 flash/CPU 或改善时序 | `-O2` 已是性能取向；收益不确定，需实际 profile | 低 |
| 供电硬件 | ESP-12F 供电按 Wi-Fi 峰值设计，使用低压差、低静态电流且瞬态能力足够的稳压器；IMU 与 ESP 供电/地线布局尽量干净 | 对重启、掉线、I2C 错误改善最大 | 需要硬件改版或飞线验证 | 高 |
| 热与校准 | 让 IMU 远离 ESP8266/稳压器热源；开机预热后做静止校准；保留温度梯度校准 | 降低 gyro bias 漂移和姿态漂移 | 结构设计限制；校准需要用户配合 | 高 |

## 推荐 profile

### 稳定优先

- `POWERSAVING_MODE = POWER_SAVING_LEGACY` 或在连接问题严重时测试 `POWER_SAVING_NONE`。
- 保留 `GyrFreq = 240`、`AccFreq = 120`、100 Hz 输出。
- 保留 `I2C_SPEED = 400000`，但硬件上保证短线、合适上拉、干净 3.3 V；若板子离散性大，把 100 kHz 作为稳定性回退 profile。
- 把 FIFO 监控做完整：记录最大 backlog、overrun 次数、每次 bulkRead 读取数量、sensor timeout 次数。
- 若 normal usage 出现 `FIFO OVERRUN!`，优先增加 `MaxFifoReadings` 和 drain 策略，而不是降 ODR。

### 平衡模式

- 基本沿用当前配置。
- `MaxFifoReadings = 24` 可作为低风险试点。
- 保留 packet bundling buffered 和 `OPTIMIZE_UPDATES = true`。
- 仅在强 Wi-Fi 环境下测试轻微 RF attenuation，保留回退开关。

### 功耗优先

- 先从硬件功耗做起：低静态电流稳压器、去掉常亮 LED、避免 USB-UART 常供电、增大电池分压电阻或改为受控分压。
- 软件上测试 `POWER_SAVING_MINIMUM`，但必须把 TPS、丢包、重连、姿态卡顿作为硬门槛。
- 关闭不必要的 telemetry，按需求关闭 `SEND_ACCELERATION`。
- 再实验 IMU ODR 降档。任何 ODR 改动都必须同步调整 `GyrTs`/`AccTs`、FIFO BDR 和运行时采样率校准预期。

### 实验模式

- 单独分支接入 LSM6DSV SFLP。目标不是先省电，而是先证明姿态质量、坐标系、延迟、校准兼容。
- 通过同一套动作和静止测试与 VQF soft-fusion 对比：静止漂移、快速转动延迟、人体遮挡 Wi-Fi 环境、温升后的 bias 行为。

## 编译开关

这些开关用于把高风险或诊断型功能放到 build profile，而不是固定写死在源码里：

```sh
# 默认：telemetry 关闭，ESP8266 I2C 400 kHz
platformio run -e BOARD_WEMOSD1MINI

# I2C 稳定性回退 profile
SLIMEVR_EXTRA_BUILD_FLAGS="-DESP8266_I2C_SPEED=100000" \
  platformio run -e BOARD_WEMOSD1MINI

# telemetry 单播到诊断机器；日志也会转发
SLIMEVR_EXTRA_BUILD_FLAGS="-DENABLE_TELEMETRY=1 -DTELEMETRY_HOST=\\\"192.168.1.10\\\"" \
  platformio run -e BOARD_WEMOSD1MINI

# telemetry 只发 performance，不转发日志
SLIMEVR_EXTRA_BUILD_FLAGS="-DENABLE_TELEMETRY=1 -DTELEMETRY_HOST=\\\"192.168.1.10\\\" -DENABLE_TELEMETRY_LOGS=0" \
  platformio run -e BOARD_WEMOSD1MINI

# 明确允许广播；不建议用于日常固件
SLIMEVR_EXTRA_BUILD_FLAGS="-DENABLE_TELEMETRY=1 -DTELEMETRY_BROADCAST=1" \
  platformio run -e BOARD_WEMOSD1MINI
```

`I2C_SPEED` 仍可作为全平台覆盖；`ESP8266_I2C_SPEED` 只用于 ESP8266 默认值，更适合 ESP-12F profile。

## 硬件生命周期备注

Espressif 在 ESP8266EX 数据手册中已把该芯片标为 not recommended for new designs。若硬件方案还没有锁定，低功耗与稳定性目标更高时应同时评估 ESP32-C3/S3/C6 等新平台；如果目标是复用 ESP-12F 库存或兼容现有 SlimeVR ESP8266 生态，上述优化仍然成立。

## 建议的验证方法

每个改动至少记录这些指标：

- 电流：空闲、连接服务器后静止、持续运动、Wi-Fi 弱信号、校准期间。最好在电池输入端测，不要只看 USB 口。
- 数据质量：SlimeVR Server 侧 TPS、姿态卡顿、丢包、重连次数。
- 固件日志：`FIFO OVERRUN!`、sensor timeout、Wi-Fi reconnect、I2C scan/error。
- 热稳定：冷启动 0-10 分钟 gyro bias 漂移；预热后静止 5 分钟姿态漂移。
- 兼容性：一个 IMU 与两个 IMU 都要测。两个 IMU 会放大 I2C、CPU、packet bundling 的压力。

## 推荐改动顺序

1. 加监控而不是先改行为：FIFO backlog 最大值、overrun 计数、bulkRead 读取 entry 数、sensor loop 用时。
2. 调大 `MaxFifoReadings` 到 24/32，并在 ESP8266 上做长时间稳定性测试。
3. 做供电与 I2C 硬件检查：3.3 V 跌落、上拉阻值、线长、地回路、IMU 与 ESP 热距离。
4. 仅在稳定基线通过后，测试 `POWER_SAVING_MINIMUM` 与 RF 输出功率降低。
5. 再做 ODR 降档 profile。
6. 最后考虑 SFLP 替代 soft-fusion。

## 外部资料

- ST LSM6DSV product page: https://www.st.com/en/mems-and-sensors/lsm6dsv.html
- ST LSM6DSV datasheet: https://www.st.com/resource/en/datasheet/lsm6dsv.pdf
- Espressif ESP8266EX datasheet: https://www.espressif.com/sites/default/files/documentation/0a-esp8266ex_datasheet_en.pdf
- Espressif ESP8266 hardware design guidelines: https://www.espressif.com/sites/default/files/documentation/esp8266_hardware_design_guidelines_en.pdf
- ESP8266 Arduino core Wi-Fi API: https://arduino-esp8266.readthedocs.io/en/latest/esp8266wifi/readme.html
