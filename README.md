# primitive-jaka-lumi-chassis-rbnx

JAKA Lumi 差速底盘原语的 Robonix 软件包。把底盘自带的 **JSON-over-TCP**
接口（`192.168.10.10:31001`）适配成标准的 `robonix/primitive/chassis/*` 能力。

```text
lumi_chassis/
├── tcp_client.py   厂商字符串指令协议：一条指令一个 TCP 连接，一条 JSON 应答
├── driver.py       配置校验 + 里程计发布 + 速度刷新 + 闭环前进/旋转
└── main.py         Primitive 生命周期与 gRPC 入口
```

## 能力

| 契约 | 模式 | 传输 | 端点 |
|---|---|---|---|
| `robonix/primitive/chassis/odom` | `topic_out` | ros2 | `/odom` |
| `robonix/primitive/chassis/twist_in` | `topic_in` | ros2 | `/cmd_vel` |
| `robonix/primitive/chassis/move` | `rpc` | grpc | `ExecuteMoveCommand` |

细节（坐标系、单位、`move` 语义、错误语义）见 [CAPABILITY.md](CAPABILITY.md)，
配置字段见 [config.spec](config.spec)。

## 前置条件

- Robonix 已安装并登记源码根：`rbnx setup ~/robonix`
- ROS 2 Humble（`/opt/ros/humble`），含 `nav_msgs`、`tf2_msgs`、`colcon`
- 与底盘连通：WiFi 连接上需要额外加一个 `192.168.10.x/24` 静态地址
  （`nmcli connection modify Lumi10000023 +ipv4.addresses 192.168.10.200/24`），
  否则 `10.5.5.x` 是 AP 自己的池，连不到底盘。

## 构建

```bash
bash scripts/build.sh
# 等价于：venv + rbnx codegen --mcp --ros2 + colcon build（ros2_idl 覆盖层）
```

只验证清单不构建：

```bash
rbnx validate .
```

## 单独启动（调试）

```bash
rbnx start -p . --endpoint 127.0.0.1:50051 --set motion_enabled=false
```

`motion_enabled` 默认 `false`，此时只发布 `/odom`，所有运动指令被拒绝并打
warning。确认里程计与急停状态无误后再打开：

```bash
rbnx start -p . --set motion_enabled=true --set max_linear_m_s=0.1
```

## 验证

```bash
# 1) provider 与能力出现在 Atlas
rbnx caps -v | rg -A6 lumi_chassis

# 2) 里程计在动（需要底盘真的在动，或用 move 命令驱动）
ros2 topic echo /odom --once

# 3) TF 是否连通
ros2 run tf2_ros tf2_echo odom base_link

# 4) 一次性命令（会真的移动，先在空地上试）
#    经由 rbnx ask 或任意 gRPC 客户端调用 chassis/move，forward_m=0.1
```

## 已知限制

- **没有轮式编码器**：`/odom` 发布的是厂商 SLAM 位姿，不是轮速积分。漂移由
  Mapping 的 `map → odom` 校正；`twist` 是位姿差分得到的近似值。
- **不做避障**：`/cmd_vel` 走 `/api/joy_control`，底盘在该模式下不启用自身的
  导航避障。要避障请用 `move` 走厂商的 `/api/move`（另一条通路），或在
  Robonix 侧给 Nav2 提供 scan。
- **单实例**：一个进程只驱动一个底盘；多实例需要不同的 `host`/`port`。
- 厂商 API 无版本协商，字段变化只能靠 `CAPABILITY.md` 里的错误表定位。

## 许可

Apache-2.0。
