# ESP8266 + LSM6DSV SFLP 实验分支

本分支是专用实验分支，不追求通用 IMU 兼容。目标硬件固定为 ESP-12F/ESP8266 + 单颗 LSM6DSV，默认 I2C 引脚沿用当前验证组合：SCL GPIO12、SDA GPIO14、INT GPIO13，IMU 地址 `0x6a`。

## 分支目标

1. 抛开其他 IMU、其他板型和多传感器组合，先把 ESP8266 + LSM6DSV 这一套跑稳。
2. 对比三种融合 profile：
   - `sflp`: LSM6DSV 内部 SFLP game rotation vector，ESP8266 只读取 quaternion。
   - `vqf_fast`: 主控 VQF，但用 `VQF_NO_MOTION_BIAS_ESTIMATION` 关闭运动中 3x3 bias Kalman。
   - `vqf_full`: 当前完整主控 VQF 基线。
3. 判断 SFLP 在 ESP8266 无 FPU 条件下是否用“传感器内部时序稳定性”换来更好的实际姿态稳定性。

## 当前实验功能

- `ESP8266_LSM6DSV_EXPERIMENT=1`
  - 动态 sensor 构建只接受 `LSM6DSV`。
  - 其他 `IMU_*` 宏在本 profile 下映射到 `ErroneousSensor`，避免误用其他传感器配置。
- `LSM6DSV_SFLP_EXPERIMENT=1`
  - LSM6DSV 初始化时启用 embedded-function bank 中的 SFLP game rotation。
  - 配置 SFLP ODR，默认 `LSM6DSV_SFLP_ODR_HZ=120`。
  - FIFO 只 batch SFLP game rotation vector 和 temperature，不再 batch raw gyro/accel。
  - FIFO tag `0x13` 解码为 3 个 half-float 的 quaternion vector part，并在 ESP8266 侧重建 `w = sqrt(1 - x^2 - y^2 - z^2)`。
  - SFLP 模式下不再把 raw gyro/accel 喂给主控 VQF，降低 ESP8266 软件浮点压力。
  - SFLP 模式下禁用 soft-fusion 校准、runtime calibration、rest calibration detector 和温度 gradient bias 调整；这些路径需要 raw gyro/accel。
  - SFLP profile 默认关闭 `SEND_ACCELERATION`，避免发送没有 raw accel 支撑的占位线性加速度。

## 构建方式

GitHub Action `ESP12F LSM6DSV Build` 增加了 `fusion_mode`：

- `sflp`: 默认实验模式。
- `vqf_fast`: 主控 VQF fast profile。
- `vqf_full`: 当前主控 VQF 基线。

本地构建示例：

```sh
SLIMEVR_OVERRIDE_DEFAULTS='{"SENSORS":[{"protocol":"I2C","imu":"IMU_LSM6DSV","int":"13","rotation":"DEG_270","scl":"12","sda":"14"}],"BATTERY":{"type":"BAT_EXTERNAL","r1":100,"r2":220,"shieldR":180,"pin":"A0"},"LED":{"LED_PIN":"2","LED_INVERTED":true}}' \
SLIMEVR_EXTRA_BUILD_FLAGS="-DESP8266_LSM6DSV_EXPERIMENT=1 -DLSM6DSV_SFLP_EXPERIMENT=1 -DLSM6DSV_SFLP_ODR_HZ=120 -DESP8266_I2C_SPEED=400000 -DSEND_ACCELERATION=0" \
  platformio run -e BOARD_WEMOSD1MINI
```

对比 VQF fast：

```sh
SLIMEVR_OVERRIDE_DEFAULTS='{"SENSORS":[{"protocol":"I2C","imu":"IMU_LSM6DSV","int":"13","rotation":"DEG_270","scl":"12","sda":"14"}],"BATTERY":{"type":"BAT_EXTERNAL","r1":100,"r2":220,"shieldR":180,"pin":"A0"},"LED":{"LED_PIN":"2","LED_INVERTED":true}}' \
SLIMEVR_EXTRA_BUILD_FLAGS="-DESP8266_LSM6DSV_EXPERIMENT=1 -DVQF_NO_MOTION_BIAS_ESTIMATION -DESP8266_I2C_SPEED=400000" \
  platformio run -e BOARD_WEMOSD1MINI
```

## 实验矩阵

每组固件至少测试 10 分钟，记录 SlimeVR Server 侧 TPS、姿态卡顿、重连、日志和主观漂移。

| Profile | 目的 | 必测项 | 通过标准 |
| --- | --- | --- | --- |
| `vqf_full` | 当前基线 | 静止漂移、快速转动、温升、FIFO overrun | 作为对照组保存日志和视频 |
| `vqf_fast` | 降低主控浮点开销 | 同 `vqf_full`，重点看持续运动后回正 | 不应明显增加卡顿；漂移可小幅变差 |
| `sflp` 120 Hz | 验证 sensor-side fusion | 坐标方向、静止 yaw 漂移、快速转身延迟、温升 | 坐标正确；无连续卡顿；漂移不明显差于 `vqf_fast` |
| `sflp` 60 Hz | 降低 FIFO/I2C 压力 | 快速动作相位、延迟、漂移 | 延迟不能体感明显；TPS 稳定 |
| `sflp` 240 Hz | 压力测试 | FIFO backlog、I2C 稳定、CPU 空余 | 不出现 normal usage FIFO overrun |

## 验证步骤

1. 闪 `vqf_full`，冷启动静止 5 分钟，记录 yaw/pitch/roll 漂移和 TPS。
2. 做 30 秒快速转身和摆动，观察姿态延迟、跳变、丢包。
3. 闪 `vqf_fast`，重复同样动作。
4. 闪 `sflp` 120 Hz，先检查坐标轴方向：
   - 水平静止时 tracker 不应倒置。
   - 绕每个轴缓慢转动时，SlimeVR Server 中方向应与 VQF 基线一致。
5. 坐标确认后做温升测试：冷启动后佩戴或靠近 ESP8266 热源 10 分钟，记录 yaw 漂移。
6. 若 `sflp` 坐标不一致，先只改 quaternion 轴映射/符号，不要同时改 ODR 或滤波策略。

## 已知限制

- SFLP 是 6 轴 game rotation vector，没有磁力计，yaw 仍不可观测，不能期望绝对航向不漂。
- 当前 SFLP 模式关闭 raw accel FIFO，GitHub Action 的 `sflp` profile 也关闭 acceleration packet；线性加速度不作为实验判据。
- SFLP quaternion 的坐标系需要实机确认。若方向错，优先在 SFLP 输出到 SlimeVR 之前做轴/符号映射。
- SFLP 内部 bias 估计是黑盒，当前实验固件不会沿用 soft-fusion/runtime gyro calibration。
- `w` 由 ESP8266 侧重建并取正值；这通常符合 game rotation vector 输出，但仍需用连续转动检查是否出现 quaternion 符号跳变。

## 参考资料

- ST LSM6DSV datasheet: https://www.st.com/resource/en/datasheet/lsm6dsv.pdf
- ST LSM6DSV platform independent driver: https://github.com/STMicroelectronics/lsm6dsv-pid
