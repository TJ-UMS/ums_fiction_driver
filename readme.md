##  通用控制板驱动 

### fiction.yaml 标准控制板

#### 参数说明
## 参数说明

| 参数名            | 说明                                 | 单位 | 默认值 |
|----------------|------------------------------------|---|---|
| con_baudrate   | 波特率                                | bps | 2000000 |
| con_port       | 端口号                                | - | /dev/ttyUSB0 |
| enable_imu     | 开启imu数据发布                          | - | true |
| enable_odom    | 开启轮式里程计数据发布                        | - | true |
| enable_tf_odom | 开启轮式里程计Tf广播数据发布(odom->base_link)   | - | true |
| IMU_Z          | IMU Z 轴校准值                         | s/rad | -0.154333333671093 |
| KD             | 速度环 D 增益                           | - | 0.20000000298023224 |
| KI             | 速度环 I 增益                           | - | 0.6000000238418579 |
| KMTT           | 运动学模型                              | - | 3 |
| KP             | 速度环 P 增益                           | - | 1.7999999523162842 |
| LA             | 底盘尺寸 轮间距/2                         | m | 0.13500000536441803 |
| LB             | 底盘尺寸 轴间距/2                         | m | 0.10000000149011612 |
| LC_read_write  | 读写模式 read不覆盖原始参数 write使用yaml覆盖原始参数 | - | read |
| MPC            | 速度测算 轮圆周长                          | m | 0.2042035162448883 |
| MPE            | 速度测算 脉冲周数                          | - | 60000.0 |

## 运动学 KMTT参数说明
| 运动学模型类型 | 宏定义 |
|---|---|
| 直接功率控制模式 BDC | `KMT MOD MOT POW 0` |
| 直接速度控制模式 BDC | `KMT MOD MOT SPD 1` |
| 四轮麦克纳姆轮 4x45deg BDC | `KMT MOD MCN_445 2` |
| 两后驱阿克曼底盘 BDC | `_AKM_2D 3` |
| 两轮差速底盘 BDC | `KMT MOD DIF_DDF 4` |
| 两轮差速底盘中菱轮毂电机8015控制器 | `KMT MOD DIF Z5K 5` |
| 四轮差速底盘 BDC | `KMT_MOD_DIF_QDF 6` |
| 导航台架 | `7` |

## 启动
```
ros2 launch ums_fiction_driver ums_fiction_v1.launch.py
```