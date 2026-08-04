# FTDX10 转接板（ESP32-S3）使用说明

FTDX10 转接板采集 FTDX10 电台的频谱数据（SPI 从机），经 **USB CDC** 与 **WiFi** 双路转发给上位机（频谱显示、远程接入等）。

---

## 1. 硬件连接

| 电台（FTDX10） | ESP32-S3 引脚 | 说明 |
|---|---|---|
| Pin5（SCK） | GPIO10 | SPI 时钟（模式 1，2MHz） |
| Pin4（MOSI） | GPIO11 | SPI 数据（电台→转接板） |
| Pin6（CS） | GPIO13 | SPI 片选（低有效，已上拉+毛刺滤波） |
| Pin12（电台 TX） | GPIO38 | UART 心跳接收（115200 8N1） |
| Pin11（电台 RX） | GPIO39 | UART 心跳回复（降驱动强度防串扰） |
| — | GPIO0（BOOT 键） | 按住开机 → 进入 WiFi 配网模式 |
| — | GPIO19/20 | USB-OTG 数据口（TinyUSB CDC） |
| — | 板载 USB-UART 桥 | 日志口（烧录与诊断） |

> **MISO 不需要接**（转接板只收不发）。
> 接线后建议用 `WIFI_STATUS` 或启动日志确认 SPI 帧正常（见第 6 节）。

---

## 2. 固件烧录（Arduino IDE）

1. **开发板**：`ESP32S3 Dev Module`（或对应的 S3 开发板型号）
2. **关键板级设置**：
   - **USB Mode = USB-OTG（Hardware CDC and JTAG）**——必须选 OTG，否则 USB 数据口无法工作（固件启动日志会打印 `USB.begin() → FAILED`）
   - **USB CDC On Boot = Disabled**（固件手动初始化 TinyUSB，两种配置均可编译，此值最稳）
   - Flash Size：按你的板子选 8MB 或 16MB
3. **烧录口**：选择板载 USB-UART 桥对应的 COM 口（即日志口）
4. **波特率**：115200

烧录完成后打开串口监视器（115200），应看到启动信息：

```
╔═══════════════════════════════════╗
║  FTDX10 转接板  —  ESP32-S3      ║
╚═══════════════════════════════════╝
UART: GPIO38(RX)←Pin12 | GPIO39(TX)→Pin11 | 115200 8N1
SPI:  GPIO10(SCK)←Pin5 | GPIO11(MOSI)←Pin4 | GPIO13(CS)←Pin6
WiFi: TCP端口:51234 | UDP端口:51235 | 配网:USB命令/长按BOOT键
[USB] USB.begin() → OK (TinyUSB 已启动)
...
[WiFi] 就绪 | IP: 192.168.x.x | RSSI: -xx dBm
[UDP] 通道就绪 :51235 (等待 HELLO 注册)
```

---

## 3. WiFi 配网

WiFi 仅支持 **2.4GHz**。

### 方式一：USB 命令（推荐）
1. 转接板 USB 数据口连电脑（或通过上位机 main.py 的 USB 模式）
2. 打开串口工具（或上位机「WiFi 配置」按钮），发送：
   ```
   WIFI_SET <SSID> <PASSWORD>
   ```
   设备保存配置并自动重启连接。

### 方式二：配网热点
按住 **BOOT 键** 开机/复位 → 出现热点 `FTDX10_AP` → 手机连接该热点 → 浏览器自动弹出配网页 → 输入路由器 SSID/密码。

### 常用命令（通过日志口/数据口发送）

| 命令 | 作用 |
|---|---|
| `WIFI_SET <SSID> <PWD>` | 设置 WiFi 并重启 |
| `WIFI_RESET` | 清除 WiFi 配置并重启 |
| `WIFI_STATUS` | 查询 WiFi/链路/UDP/TCP 状态 |
| `HELP` | 显示帮助 |

---

## 4. 数据通道

| 通道 | 端口 | 特点 | 用途 |
|---|---|---|---|
| **USB CDC** | —（USB 数据 COM 口） | 完整帧流，稳定可靠 | 有线直连 |
| **WiFi UDP** | 51235 | **满帧率 25fps**，无发送缓冲限制，实时性最好 | 无线主通道（推荐） |
| **WiFi TCP** | 51234 | 可靠有序，但受固件 lwip 发送缓冲限制（约 5~18fps） | 无线兼容/备用 |

**帧格式**（所有通道统一）：`66 CC FF + 长度(2字节大端) + XOR校验(1字节) + 载荷(4096字节)`

**UDP 分片协议**：每帧分 3 片报文（1366/1366/1364 字节），报文 = `66 CC FF + 帧序号(1B) + 片号(1B) + 载荷`。客户端向 :51235 发送 `HELLO` 完成注册后，固件按源 IP/端口单播数据（端口被占用时随机端口也支持）。

> 详细协议见 `docs/protocol-zh.md`。

---

## 5. 使用方法

### 5.1 上位机（带 GUI，`PC_Software/main.py`）

运行：`python main.py`（依赖见 `requirements.txt`）

- **USB 模式**：选择数据 COM 口（TinyUSB CDC 口，不是日志口）→ 连接
- **WiFi 模式**：输入转接板 IP → 协议选择：
  - **UDP**（推荐）：满帧率实时
  - **TCP**：低帧率兼容
- WiFi 模式可开启「串口转发」把数据写到 com0com 虚拟串口，供 wfview 等串口软件使用

### 5.2 无 GUI 命令行转发（`PC_Software/remote.py`）

适合脚本化调用、接入你自己的程序：

```bash
# UDP 源 → 管道给其它程序
python remote.py udp 192.168.1.100 | 频谱程序

# USB 串口源 → 虚拟串口（wfview 等串口软件读取）
python remote.py usb COM3 --out com:CNCA0

# WiFi TCP 源 → 本地 TCP 服务器（其它程序连接 127.0.0.1:5200）
python remote.py tcp 192.168.1.100 --out tcp:5200

# 数据存文件
python remote.py udp 192.168.1.100 --out file:data.bin
```

不带参数运行则交互式选择数据源与输出方式。输出端：`stdout | com:COMx | tcp:PORT | udp:PORT | file:PATH`。所有输出统一为完整帧流（66 CC FF 格式）。

---

## 6. 日志与状态解读

### 周期状态（每 5 秒一行）
```
[WiFi] 5s状态: 客户1 发送+90 丢帧+36 UDP+125 RSSI-31 | 链路已连接 192.168.1.100
```
| 字段 | 含义 | 正常值 |
|---|---|---|
| 客户 | TCP 客户端数 | 0~4 |
| 发送+/丢帧+ | TCP 5s 内成功/被拒帧数 | 丢帧越多说明 TCP 缓冲受限 |
| UDP+ | UDP 5s 发送帧数 | **≈125（满帧率 25fps）** |
| RSSI | 信号强度 | 大于 -60 良好 |

### 连接事件
```
[UDP] 客户端192.168.1.50:51235 注册     ← UDP 通道建立 ✓
[WiFi] 客户端192.168.1.50 已连接 → 槽0  ← TCP 客户端接入
[WiFi] 槽0 缓冲满丢帧(降帧率)            ← TCP 缓冲满，正常降帧（不影响 UDP）
[WiFi] 槽0 sndbuf不可查询 errno=109      ← 正常（预编译库限制，UDP 不受影响）
```

### 查询状态（发送 `WIFI_STATUS`）
```
[CMD] WiFi: 已连接 | IP: 192.168.1.100 | RSSI: -30 dBm | 链路: 正常 | 自愈: 0 | 队列丢: 0 | TCP发: 1523 | TCP客户: 0 | UDP发: 12500 | UDP丢片: 3 | UDP客户: 已注册
```

---

## 7. 常见问题排查

| 现象 | 排查 |
|---|---|
| 没有 USB 数据 COM 口 | ① 确认 USB Mode = USB-OTG；② 启动日志 `USB.begin()` 是否为 OK；③ 设备管理器刷新/重插 |
| 两个 COM 口分不清 | 数据口 = TinyUSB CDC（固件 USB 数据）；日志口 = 板载 USB-UART 桥（115200 打印启动信息） |
| 上位机收不到 UDP 数据 | ① 日志有无 `[UDP] 客户端... 注册`；② IP 是否正确；③ 防火墙是否放行 UDP 51235 |
| UDP 花屏/丢帧 | 信号弱或空口干扰——看 RSSI（应 >-60）；确认无 2.4GHz 干扰；必要时临时切 TCP |
| 频谱校验失败 | SPI 数据问题：接线是否正确、电台心跳是否回复（`REPLY_HEARTBEAT=1`）、SPI 模式是否为 1 |
| 电台停止发数据 | 心跳回复超时（10ms 内必须回复）——确认 UART 接线、`REPLY_HEARTBEAT=1` |
| WiFi 反复掉线 | 距离/干扰；日志观察 `掉线, 等待自动重连` 频率与 RSSI；固件有自动自愈机制（重建服务器） |

---

## 8. 固件关键配置速查

| 配置 | 值 | 说明 |
|---|---|---|
| TCP 端口 | 51234 | WiFi TCP 服务器 |
| UDP 端口 | 51235 | WiFi UDP 通道（HELLO 注册） |
| 帧大小 | 4096 字节 | SPI 单帧 |
| SPI | 模式 1 / 2MHz / GPIO10,11,13 | 从机接收 |
| UART 心跳 | 115200 / GPIO38,39 / `alive\0` | 必须 10ms 内回复 |
| `RUNTIME_LOG` | 0 | 运行期日志开关（1=输出诊断，0=静音） |
| `REPLY_HEARTBEAT` | 1 | 心跳回复（必须为 1） |
| `HEARTBEAT_LED` | 0 | 心跳 LED（GPIO48）开关 |
| WiFi 配网 | 长按 BOOT 开机 / `WIFI_SET` 命令 | 2.4GHz only |

