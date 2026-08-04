// ============================================================
//  文件: DX10_WF.ino
//  描述: ESP32-S3 转接板固件 — FTDX10 电台数据采集
//  电台 ACC Pinout:
//    Pin4  (MOSI)  → 数据   Pin5  (SCLK) → 时钟
//    Pin6  (CS)    → 片选   Pin11 (CNT RX) ← ESP32 TX
//    Pin12 (CNT TX)→ ESP32 RX
//  协议: UART 115200 心跳( alive\0 ) | SPI 模式1 2MHz 从机
//  输出: USB CDC + WiFi TCP (双路并发)
//  配网: 热点配网(WiFiManager) / USB串口配网
//  架构: Core 1 = SPI接收(6) + USB发送(5) + WiFi发送(3) + TinyUSB事件(24)
//        — SPI 与 USB-CDC 数据传送同核, SPI 实时性最高;
//        Core 0 = 串口心跳/命令(1), 与数据链路完全隔离
//  无线自愈: STA 掉线自动重连 + TCP 服务器重建; 发送非阻塞 (缓冲不足丢帧不卡任务)
// ============================================================

#include <driver/spi_slave.h>
#include <driver/gpio.h>
#include <driver/gpio_filter.h>
#include <lwip/sockets.h>   // send/select: core 3.3.11 socket 层 API / socket-level API
#include <errno.h>          // EAGAIN/EWOULDBLOCK 判断 / errno for non-blocking send
#include <WiFiManager.h>   // 包含 WiFi.h
#include <Preferences.h>
#include <tusb.h>          // TinyUSB 原生 API: 不依赖 DTR / native API, no DTR dependency
#include <USB.h>           // ESPUSB USB 对象: USB.begin() 强制启动 TinyUSB / ESPUSB object

// 手动注册 TinyUSB CDC 接口: CDCOnBoot=0 时 core 不创建 USBCDC 对象 → 接口未注册
// → USB.begin() 枚举裸设备 → Windows 显示"其他设备"/无 COM 口!
// 此对象保证任何编译环境 (Arduino IDE / PlatformIO / 默认配置) 下 CDC 接口都存在。
// manual CDC interface registration: when CDCOnBoot=0 the core creates no USBCDC
// object → no CDC interface → bare device → no COM port! This guarantees the
// CDC interface exists under ANY build config (IDE / PlatformIO / defaults).
#if !ARDUINO_USB_CDC_ON_BOOT
USBCDC usb_cdc;   // 构造函数注册 CDC 接口 itf 0 / ctor registers CDC iface 0
#endif

// 日志口 = UART0 (板载 USB-UART 桥 → COM)
// USB-OTG 模式下 core 3.x 无 USBSerial 对象, 用宏映射到 Serial0
// log port = UART0 (onboard USB-UART bridge → log COM port);
// USBSerial object absent in USB-OTG mode, alias it to Serial0
#define USBSerial Serial0

// ============================================================
//  常量
// ============================================================
#define TCP_PORT          51234
#define MAX_TCP_CLIENTS   4

#define PACKET_SIZE       4096

// 运行期日志开关: 0=全静音 (高速数据链路零干扰), 1=输出诊断
// 启动期 banner / USB 命令响应不受此开关控制 (数据流开始前/用户主动触发)
// runtime log switch: 0=silent (zero interference with data path), 1=diagnostic;
// startup banners & USB command replies stay on (pre-stream / user-triggered)
#define RUNTIME_LOG 0
#define HEARTBEAT_LEN     6
static const uint8_t HEARTBEAT[HEARTBEAT_LEN] = {0x61, 0x6C, 0x69, 0x76, 0x65, 0x00};

// 心跳回复开关: 1=回复 (必须! 电台 10ms 内无回复 → 2-3s 后停止 SPI 数据)
// 0=不回复 (仅测试用; 会导致电台停止 SPI)
// heartbeat reply: 1=reply (REQUIRED — radio stops SPI 2-3s after 10ms
// missed reply window); 0=no reply (test only, radio will halt SPI)
#define REPLY_HEARTBEAT 1

// 心跳指示 LED 开关: 0=禁用 (验证排除 GPIO48 翻转干扰; 心跳回复不受影响)
// heartbeat indicator LED: 0=off (isolate GPIO48 toggle as a variable; reply unaffected)
#define HEARTBEAT_LED 0

// 引脚
// UART1 换到芯片对侧 (GPIO38/39): 原 GPIO1/2 与 SPI (GPIO10/11/13) 同侧相邻 →
// 心跳回复翻转经内部电源/封装耦合干扰 SPI 采样 (实测: 错位0但载荷错; 外置串口无干扰)。
// 对侧引脚跨芯片衰减, 配合降驱动强度根治。
// UART1 moved to opposite die side (GPIO38/39): old GPIO1/2 sat adjacent to SPI
// (GPIO10/11/13) → reply toggles coupled into SPI sampling (proven: payload
// corruption, misalign=0; external UART clean). Opposite side kills the path.
#define RADIO_UART_RX     38
#define RADIO_UART_TX     39
#define SPI_SLAVE_SCLK   10
#define SPI_SLAVE_MOSI   11
#define SPI_SLAVE_CS     13
#define BOOT_BTN_GPIO     0    // 按住此引脚开机 → 进入配网模式

// ========== 全局 ==========
HardwareSerial RadioSerial(1);
static uint32_t heartbeat_count = 0;   // 心跳计数 (诊断用, 运行期不打印) / heartbeat counter (diagnostic only)

// SPI 数据质量计数 — 全局化: [USB] 60s 日志跨任务读取 (spi_rx_task 写, usb_tx_task 读)
// SPI quality counters — global so the [USB] 60s log can read them
// (written by spi_rx_task, read by usb_tx_task)
static volatile uint32_t glitch_count = 0;    // CS 毛刺短事务 (<32B) / CS-glitch short xfer (<32B)
static volatile uint32_t misalign_count = 0;  // 标准帧帧头错位 (诊断数据错位) / misaligned standard frame
static volatile uint32_t trunc_count = 0;     // 4097→4096 计数修正 / 4097→4096 count clamp
static volatile uint32_t non_std_count = 0;   // 非标准帧已转发 (STM32 策略) / non-standard forwarded (STM32 policy)
static volatile int hb_watch_frames = 0;      // 心跳回复后的观察帧窗口 (Core0 写, Core1 读) / frames watched after heartbeat reply
static volatile uint32_t hb_bad_frames = 0;   // 观察窗口内出现的异常帧 (毛刺/非标准/错位) / bad frames inside watch window

// SPI 三缓冲 (DMA 对齐) — 3 事务流水线: CS 高 <4ms 响应窗口极短,
// 队列任意时刻 ≥2 备用事务, 处理再慢也不空窗
// triple buffers — 3-txn pipeline: CS-high window <4ms is tight,
// queue always keeps >=2 spare txns, no blank window even if slow
static uint8_t spi_buf0[PACKET_SIZE] __attribute__((aligned(64)));
static uint8_t spi_buf1[PACKET_SIZE] __attribute__((aligned(64)));
static uint8_t spi_buf2[PACKET_SIZE] __attribute__((aligned(64)));

// WiFi TCP (无线数据主通道: 可靠有序, 单播全帧率) / primary wireless data channel
static WiFiServer tcp_server(TCP_PORT);
static WiFiClient tcp_clients[MAX_TCP_CLIENTS];
static volatile uint32_t wifi_tx_count = 0;     // TCP 成功发送次数 (每活动客户端每帧+1) / successful TCP writes
static volatile uint32_t wifi_client_count = 0; // 当前活动 TCP 客户端数 / active TCP client count
static volatile bool wifi_link_down = false;     // STA 掉线标记 (WiFi 事件回调置位) / set by disconnect event
static volatile uint32_t wifi_reconnects = 0;    // 链路恢复自愈次数 / link-recovery count
static uint8_t wifi_client_stall[MAX_TCP_CLIENTS] = {0};  // 客户端连续写不进计数 / consecutive unwritable counter
// 帧组装缓冲 (头6B+载荷连续): 单次 send 原子发送完整帧, 防残帧/防断开
// frame assembly buffers (6B header + payload contiguous): atomic single send
static uint8_t wifi_frame_buf[MAX_TCP_CLIENTS][PACKET_SIZE + 6];

// UDP 数据通道 (满帧率实时主通道) — Arduino 预编译 lwip 的 TCP_SND_BUF=5744
// 锁死 → TCP 每 ACK 周期只能发 1 帧 (~5fps); UDP 无发送缓冲限制 → 25fps 全速,
// 强信号 (RSSI>-50) 下丢包极少。上位机 UdpWorker 协议:
//   注册: 客户端向 :51235 发 HELLO → 固件记录源 IP:port → 单播
//   数据: 每帧分 3 片 (1366/1366/1364), 报文 = 66 CC FF + seq(1B) + slice(1B) + payload
// UDP channel — full-rate realtime path (prebuilt lwip TCP_SND_BUF=5744
// caps TCP at ~1 frame/ACK; UDP has no send-buffer limit → full 25fps)
#define UDP_PORT           51235   // 与上位机 UDP_PORT_DEFAULT 一致
static int udp_sock = -1;                  // lwip UDP socket (绕过 NetworkUDP 逐字节 write)
static struct sockaddr_in udp_peer;        // 对端地址 (HELLO 注册源)
static bool udp_peer_valid = false;        // 已有注册客户端
static uint32_t udp_tx_count = 0, udp_drop_count = 0;   // 帧/丢片计数
static uint8_t udp_seq = 0;                // 帧序号 0-255 循环
static uint8_t udp_pkt_buf[5 + 1366];      // 头5B+单片 (static 防栈溢出)

// 数据队列: SPI → WiFi (TCP 单通道)
typedef struct {
  uint8_t data[PACKET_SIZE];
  size_t  len;
} wifi_queue_item_t;
static QueueHandle_t tcp_queue = NULL;   // TCP 队列 (无线主通道) / TCP queue (wireless channel)
static uint32_t wifi_dropped = 0;

// 数据队列: SPI → USB (异步发送, 不阻塞 SPI 接收 / async send, never blocks SPI RX)
typedef struct {
  uint8_t data[PACKET_SIZE];
  size_t  len;
} usb_queue_item_t;
static QueueHandle_t usb_queue = NULL;
static uint32_t usb_dropped = 0;
static volatile bool usb_ready = false;   // USB.begin() 成功才允许 tud_* 写 / set by USB.begin() result

// USB 串口命令缓冲区
#define CMD_BUF_SIZE 160
static char cmd_buf[CMD_BUF_SIZE];
static uint8_t cmd_idx = 0;

// ============================================================
//  帧封装
// ============================================================
static size_t write_frame(Print &stream, const uint8_t *data, size_t len) {
  uint8_t xor_sum = 0;
  for (size_t i = 0; i < len; i++) xor_sum ^= data[i];

  // 头部 6 字节合并为一次写 (减少 write 调用次数)
  // merge 6-byte header into one write (fewer write calls)
  uint8_t hdr[6] = {0x66, 0xCC, 0xFF,
                    (uint8_t)(len >> 8), (uint8_t)len, xor_sum};
  size_t w = stream.write(hdr, 6);
  w += stream.write(data, len);
  return w;   // 实际写入字节数: 调用方据此断开僵尸客户端 / bytes written: caller drops zombies
}

static void usb_enqueue(const uint8_t *data, size_t len) {
  if (usb_queue == NULL) return;

  static usb_queue_item_t item;   // 4KB 移出任务栈 / 4KB off the task stack
  item.len = (len > PACKET_SIZE) ? PACKET_SIZE : len;
  memcpy(item.data, data, item.len);

  if (xQueueSend(usb_queue, &item, 0) != pdTRUE) {
    // 满: 丢最旧换新 → 时间轴连续降帧率, 无跳变 (主机恢复后无突发 → 无红线)
    // full: drop oldest for newest — timeline drops frames smoothly, no jump
    // (host recovery replays latest only, no burst → no red line)
    static usb_queue_item_t discard;   // 4KB 静态区 / static saves stack
    xQueueReceive(usb_queue, &discard, 0);
    xQueueSend(usb_queue, &item, 0);
    usb_dropped++;
  }
}

// ============================================================
//  主机长时间不读 → 丢弃积压旧帧 (丢旧保新)
//  防恢复后 25+ 帧瞬间灌出 → 下游时间轴突发滚动 → 抖动
//  Host stalls → drop queued stale frames (keep-latest):
//  prevents burst replay after recovery → timeline jitter
// ============================================================
static void usb_flush_queue() {
  static usb_queue_item_t tmp;   // 4KB 静态区, 防任务栈溢出 / static saves task stack
  while (xQueueReceive(usb_queue, &tmp, 0) == pdTRUE) {
    usb_dropped++;
  }
}

// ============================================================
//  USB 发送任务 (Core 1) — 参考 STM32 策略: 收发解耦
//  慢速 USB (4KB≈64ms) 不影响 SPI 接收任务
//  USB TX task — like STM32: decouple RX from TX;
//  slow USB (4KB≈64ms) never stalls SPI receiving
// ============================================================
static void usb_tx_task(void *pvParameters) {
  static usb_queue_item_t item;   // 4KB 静态区, 省任务栈 / static saves stack
  static uint8_t last_frame[PACKET_SIZE];   // 最近成功发送的完整帧 (丢帧重发副本) / last good frame
  static bool last_valid = false;           // 副本有效标记 / replay copy valid

  uint32_t tx_frames = 0;         // 出队帧数 / dequeued frames
  uint32_t tx_bytes = 0;          // 实际写入字节 / bytes written
  uint32_t wait_loops = 0;        // 缓冲满等待次数 / busy-wait loops
  uint32_t replayed = 0;          // 副本补发成功次数 / successful replays
  unsigned long last_log = 0;

  while (true) {
    if (xQueueReceive(usb_queue, &item, portMAX_DELAY) != pdTRUE) continue;
    tx_frames++;

    // TinyUSB 未启动时禁止调用 tud_* (未初始化栈可能崩溃) / never touch dead stack
    if (!usb_ready) {
      usb_dropped++;
      continue;
    }

    // 帧封装 + 发送 (带副本补发): 原帧发送失败时重发上一帧副本,
    // 下游 (wfview/上位机) 帧率与时间轴保持连续 → 丢帧变成"画面停留一瞬"
    // frame + replay-on-drop: on failure resend last good frame copy so
    // downstream frame rate stays continuous → loss looks like a still frame
    bool replayed_frame = false;
    for (int attempt = 0; attempt < 2; attempt++) {
      if (attempt == 1) {
        if (!last_valid || replayed_frame) break;   // 无副本或已重发 → 放弃 / no copy or already replayed
        memcpy(item.data, last_frame, PACKET_SIZE);
        item.len = PACKET_SIZE;
        replayed++;
        replayed_frame = true;
      }

      uint8_t xor_sum = 0;
      for (size_t i = 0; i < item.len; i++) xor_sum ^= item.data[i];
      uint8_t hdr[6] = {0x66, 0xCC, 0xFF,
                        (uint8_t)(item.len >> 8), (uint8_t)item.len, xor_sum};

      // TinyUSB 原生写: 不受 DTR 限制, 主机打开串口即收到
      // native TinyUSB write: ignores DTR, host gets data right after opening
      // 超时保护: 主机拔出/不读时 FIFO 永满 → 丢弃本帧, 防任务无限阻塞
      // timeout guard: host unplugged/unreading → FIFO never drains → drop frame
      size_t off = 0;
      uint32_t stall = 0;
      bool send_ok = true;
      while (off < 6) {
        size_t space = tud_cdc_n_write_available(0);
        if (space == 0) {
          tud_cdc_n_write_flush(0);
          if (++stall > 500) { tud_cdc_n_write_clear(0); usb_dropped++; usb_flush_queue(); send_ok = false; break; }  // ~1s 无进展, 清残帧+丢积压防下游错乱 / ~1s stall, clear + flush stale

          wait_loops++;
          vTaskDelay(pdMS_TO_TICKS(1));
          continue;
        }
        size_t chunk = 6 - off; if (chunk > space) chunk = space;
        size_t sent = tud_cdc_n_write(0, hdr + off, chunk);
        if (sent == 0) {
          tud_cdc_n_write_flush(0);
          if (++stall > 500) { tud_cdc_n_write_clear(0); usb_dropped++; usb_flush_queue(); send_ok = false; break; }
          wait_loops++;
          vTaskDelay(pdMS_TO_TICKS(1));
          continue;
        }
        off += sent; tx_bytes += sent; stall = 0;
      }
      if (!send_ok) continue;   // 触发副本重发 / trigger replay

      // 数据载荷发送 (TX 缓冲 4096B, 通常一次写完)
      // payload send (TX buffer 4096B, usually one shot)
      off = 0;
      stall = 0;
      while (off < item.len) {
        size_t space = tud_cdc_n_write_available(0);
        if (space == 0) {
          tud_cdc_n_write_flush(0);
          if (++stall > 500) { tud_cdc_n_write_clear(0); usb_flush_queue(); send_ok = false; break; }  // ~1s 无进展, 清残帧+丢积压 / ~1s stall, clear + flush stale
          wait_loops++;
          vTaskDelay(pdMS_TO_TICKS(1));
          continue;
        }
        size_t chunk = item.len - off; if (chunk > space) chunk = space;
        size_t sent = tud_cdc_n_write(0, item.data + off, chunk);
        if (sent == 0) {
          tud_cdc_n_write_flush(0);
          if (++stall > 500) { tud_cdc_n_write_clear(0); usb_flush_queue(); send_ok = false; break; }
          wait_loops++;
          vTaskDelay(pdMS_TO_TICKS(1));
          continue;
        }
        off += sent; tx_bytes += sent; stall = 0;
      }
      if (!send_ok) continue;   // 触发副本重发 / trigger replay

      // 完整发送成功 → 仅标准完整帧保存副本 (短帧副本会劣化后续重发)
      // save replay copy for standard full frames only (short-frame copy degrades replays)
      if (item.len == PACKET_SIZE) {
        memcpy(last_frame, item.data, PACKET_SIZE);
        last_valid = true;
      }
      break;
    }
    tud_cdc_n_write_flush(0);

    // 60 秒聚合状态: 极低频打印, 日志传输期间不再干扰数据流节奏
    // 60s stats: ultra-low-frequency log, zero interference with data flow
    if (millis() - last_log > 60000) {
#if RUNTIME_LOG
      USBSerial.printf("[USB] 帧:%u(frames) | 字节:%u(bytes) | 等待:%u(waits) | 丢帧:%u(dropped) | 重发:%u(replays) | 错位:%u(misalign) | 毛刺:%u(glitch) | 非标准:%u(non-std) | 心跳:%u(hb) | 心后坏:%u(hb-bad) | TCP发:%u(tcp-tx) | TCP客户:%u(tcp-clients) | 就绪:%d(ready)\n",
                       tx_frames, tx_bytes, wait_loops, usb_dropped, replayed,
                       misalign_count, glitch_count, non_std_count, heartbeat_count, hb_bad_frames,
                       wifi_tx_count, wifi_client_count, usb_ready);
#endif
      last_log = millis();
    }
  }
}

static void udp_init(void) {
  udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (udp_sock < 0) {
    USBSerial.printf("[UDP] socket失败 (socket failed) errno=%d\n", errno);
    return;
  }
  struct sockaddr_in local;
  local.sin_family = AF_INET;
  local.sin_addr.s_addr = htonl(INADDR_ANY);
  local.sin_port = htons(UDP_PORT);
  if (bind(udp_sock, (struct sockaddr *)&local, sizeof(local)) != 0) {
    USBSerial.printf("[UDP] bind失败 (bind failed) errno=%d\n", errno);
    udp_sock = -1;
    return;
  }
  USBSerial.printf("[UDP] 通道就绪 (Channel ready) :%d (等待 HELLO 注册 / waiting for HELLO)\n", UDP_PORT);
}

static void udp_poll(void) {
  if (udp_sock < 0) return;
  uint8_t buf[64];
  struct sockaddr_in peer;
  socklen_t plen = sizeof(peer);
  ssize_t n;
  while ((n = recvfrom(udp_sock, buf, sizeof(buf), MSG_DONTWAIT,
                       (struct sockaddr *)&peer, &plen)) >= 0) {
    if (n >= 5 && memcmp(buf, "HELLO", 5) == 0) {
      udp_peer = peer;          // 记录源 IP:port (随机端口也支持)
      udp_peer_valid = true;
      USBSerial.printf("[UDP] 客户端%s:%d 注册 (Client registered)\n",
                       IPAddress(peer.sin_addr.s_addr).toString().c_str(),
                       ntohs(peer.sin_port));
    }
  }
}

static void udp_send_frame(const uint8_t *data, size_t len) {
  if (udp_sock < 0 || !udp_peer_valid) return;
  const uint8_t *p = data;
  size_t remain = len;
  uint8_t slice = 0;
  while (remain > 0) {
    size_t chunk = (remain > 1366) ? 1366 : remain;   // 3 片: 1366/1366/1364 = 4096
    udp_pkt_buf[0] = 0x66; udp_pkt_buf[1] = 0xCC; udp_pkt_buf[2] = 0xFF;
    udp_pkt_buf[3] = udp_seq;
    udp_pkt_buf[4] = slice;
    memcpy(udp_pkt_buf + 5, p, chunk);
    ssize_t w = sendto(udp_sock, udp_pkt_buf, 5 + chunk, MSG_DONTWAIT,
                       (struct sockaddr *)&udp_peer, sizeof(udp_peer));
    if (w < 0) {
      udp_drop_count++;         // 丢片: 上位机缺片丢帧 (实时优先, 不补发)
      return;                   // 中断本帧剩余片 (帧不完整, 上位机自然丢弃)
    }
    p += chunk;
    remain -= chunk;
    slice++;
  }
  udp_tx_count++;
  udp_seq++;
}

static void wifi_send_packet(const uint8_t *data, size_t len) {
  uint32_t active = 0;
  for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
    if (!tcp_clients[i] || !tcp_clients[i].connected()) continue;
    active++;

    int sfd = tcp_clients[i].fd();
    if (sfd < 0) { wifi_dropped++; continue; }

    // 完整帧 (头6B+载荷) 组装到连续缓冲后【单次】send — lwip send 全写或全拒:
    // 全拒(缓冲满) = 零字节入流 → 丢帧, 连接保持; 成功 = 帧原子完整。
    // 切勿拆成两次 send (头/载荷): 头部入流后载荷被拒 → 残帧 → 只能断开。
    // whole frame in one contiguous buffer, single send — lwip send is
    // all-or-none: rejection = zero bytes in stream → drop frame, keep the
    // connection; success = atomic frame. Never split header/payload.
    uint8_t xor_sum = 0;
    for (size_t j = 0; j < len; j++) xor_sum ^= data[j];
    uint8_t *f = wifi_frame_buf[i];
    f[0] = 0x66; f[1] = 0xCC; f[2] = 0xFF;
    f[3] = (uint8_t)(len >> 8);
    f[4] = (uint8_t)len;
    f[5] = xor_sum;
    memcpy(f + 6, data, len);

    ssize_t w = send(sfd, f, len + 6, MSG_DONTWAIT);
    if (w < 0) {
      int err = errno;
      wifi_dropped++;
      if (err == EAGAIN || err == EWOULDBLOCK) {
        // 内核缓冲满 → 丢帧降帧率, 连接保持; 每段连续失败期仅打印一次
        if (wifi_client_stall[i] == 0)
          USBSerial.printf("[WiFi] 槽%d 缓冲满丢帧 (TX buffer full, frame dropped)\n", i);
        if (++wifi_client_stall[i] >= 500) {   // ~20s 写不进 → 对端卡死 → 断开
          USBSerial.printf("[WiFi] 槽%d 连续20s写不进, 断开 (Stalled 20s, disconnect)\n", i);
          tcp_clients[i].stop();
          wifi_client_stall[i] = 0;
        }
      } else {
        // 真错误 (对端 RST/FIN/网络错误) → 断开, main.py 自动重连
        USBSerial.printf("[WiFi] 槽%d send失败 (send failed) errno=%d, 断开 (disconnect)\n", i, err);
        tcp_clients[i].stop();
        wifi_client_stall[i] = 0;
      }
      continue;
    }
    wifi_client_stall[i] = 0;
    wifi_tx_count++;
  }
  wifi_client_count = active;
}

// ============================================================
//  数据队列
// ============================================================
static void wifi_enqueue_data(const uint8_t *data, size_t len) {
  if (tcp_queue == NULL) return;

  static wifi_queue_item_t item;   // 4KB 移出任务栈 / 4KB off the task stack
  item.len = (len > PACKET_SIZE) ? PACKET_SIZE : len;
  memcpy(item.data, data, item.len);

  // TCP 队列: 满丢最旧换新, 时间轴连续降帧率 / TCP: drop-oldest, smooth frame-rate drop
  if (xQueueSend(tcp_queue, &item, 0) != pdTRUE) {
    static wifi_queue_item_t discard;   // 4KB 静态区 / static saves stack
    xQueueReceive(tcp_queue, &discard, 0);
    xQueueSend(tcp_queue, &item, 0);
    wifi_dropped++;
  }
}

// ============================================================
//  USB 串口命令处理 (Core 0, uart_task 中运行)
// ============================================================
static void handle_usb_commands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmd_idx > 0) {
        cmd_buf[cmd_idx] = '\0';

        // 解析命令
        if (strncmp(cmd_buf, "WIFI_SET ", 9) == 0) {
          // WIFI_SET <ssid> <password>
          char *rest = cmd_buf + 9;
          char *space = strchr(rest, ' ');
          if (space) {
            *space = '\0';
            const char *ssid = rest;
            const char *pass = space + 1;

            USBSerial.printf("[CMD] 保存 WiFi (Saving WiFi): %s\n", ssid);

            Preferences pref;
            pref.begin("ftdx10", false);
            pref.putString("wifi_ssid", ssid);
            pref.putString("wifi_pass", pass);
            pref.end();

            // 下次启动自动连接
            USBSerial.println("[CMD] 配置已保存, 重启中... (Saved, rebooting...)");
            delay(100);
            ESP.restart();
          } else {
            USBSerial.println("[CMD] 格式 (Usage): WIFI_SET <SSID> <PASSWORD>");
          }
        }
        else if (strcmp(cmd_buf, "WIFI_RESET") == 0) {
          USBSerial.println("[CMD] 清除 WiFi 配置, 重启... (Config cleared, rebooting...)");
          Preferences pref;
          pref.begin("ftdx10", false);
          pref.remove("wifi_ssid");
          pref.remove("wifi_pass");
          pref.end();

          WiFiManager wm;
          wm.resetSettings();

          delay(100);
          ESP.restart();
        }
        else if (strcmp(cmd_buf, "WIFI_STATUS") == 0) {
          USBSerial.printf("[CMD] WiFi: %s | IP: %s | RSSI: %d dBm | 链路(Link): %s | 自愈(Recovers): %u | 队列丢(Q-drops): %u | TCP发(TX): %u | TCP客户(Clients): %u | UDP发(TX): %u | UDP丢片(Drops): %u | UDP客户(Client): %s\n",
                        WiFi.isConnected() ? "已连接/Connected" : "未连接/Disconnected",
                        WiFi.localIP().toString().c_str(),
                        WiFi.RSSI(),
                        wifi_link_down ? "断开中/Down" : "正常/Normal",
                        wifi_reconnects,
                        wifi_dropped, wifi_tx_count, wifi_client_count,
                        udp_tx_count, udp_drop_count,
                        udp_peer_valid ? "已注册/Registered" : "无/None");
        }
        else if (strcmp(cmd_buf, "HELP") == 0) {
          USBSerial.println("[CMD] 可用命令 (Available commands):");
          USBSerial.println("  WIFI_SET <SSID> <PWD>  — 设置WiFi并重启 (Set WiFi & reboot)");
          USBSerial.println("  WIFI_RESET             — 清除WiFi配置并重启 (Clear config & reboot)");
          USBSerial.println("  WIFI_STATUS            — 查询WiFi状态 (Query status)");
          USBSerial.println("  HELP                   — 显示帮助 (Show help)");
        }
        else {
          if (cmd_buf[0] != '\0') {
            USBSerial.printf("[CMD] 未知命令 (Unknown): %s (输入 HELP / type HELP)\n", cmd_buf);
          }
        }

        cmd_idx = 0;
      }
    } else if (cmd_idx < CMD_BUF_SIZE - 1) {
      cmd_buf[cmd_idx++] = c;
    }
  }
}

static void wifi_event_cb(WiFiEvent_t event, arduino_event_info_t info) {
  // STA 掉线 → 置标记 (回调仅置标志; 不在回调内调用任何 WiFi 栈 API, 防 lwip 锁死)
  // disconnect → flag only (never call WiFi-stack APIs inside the callback)
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    wifi_link_down = true;
  }
}

// ============================================================
//  WiFi TCP 任务 (Core 1, 优先级 3) — 无线数据主通道
//  TCP task — primary wireless data channel
// ============================================================
static void wifi_task(void *pvParameters) {
  tcp_server.begin();
  tcp_server.setNoDelay(true);
  udp_init();   // UDP 满帧率通道 / full-rate UDP channel
  WiFi.onEvent(wifi_event_cb);   // 注册 STA 掉线事件 / register STA disconnect event

#if RUNTIME_LOG
  USBSerial.printf("[WiFi] TCP 服务器 (TCP server) | %s:%d | 最大客户端 (max clients): %d\n",
                WiFi.localIP().toString().c_str(), TCP_PORT, MAX_TCP_CLIENTS);
#endif

  bool last_link_down = false;   // 掉线沿检测 (首次打印一次) / drop-edge detect

  while (true) {
    // ---- 链路自愈: 掉线 → 等自动重连 → 清旧连接/积压 → 重建服务器 ----
    if (wifi_link_down) {
      if (WiFi.status() == WL_CONNECTED) {
        for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
          if (tcp_clients[i]) {
            USBSerial.printf("[WiFi] 链路恢复 (Link restored), 断开槽%d旧连接 (closing slot %d)\n", i, i);
            tcp_clients[i] = WiFiClient();   // 释放引用 → 关闭 socket
            wifi_client_stall[i] = 0;
          }
        }
        static wifi_queue_item_t stale;   // 4KB 移出任务栈 / 4KB off task stack
        while (xQueueReceive(tcp_queue, &stale, 0) == pdTRUE) wifi_dropped++;  // 丢积压旧帧
        tcp_server.close();          // 旧监听绑定旧 IP, 已失效
        tcp_server.begin();          // 用新 IP 重新监听
        tcp_server.setNoDelay(true);
        wifi_link_down = false;
        wifi_reconnects++;
        udp_peer_valid = false;   // 等 PC 重新 HELLO 注册 / wait re-register
        USBSerial.printf("[WiFi] 服务器已重建 (Server rebuilt) | IP: %s | RSSI: %d dBm\n",
                         WiFi.localIP().toString().c_str(), WiFi.RSSI());
      } else if (!last_link_down) {
        USBSerial.printf("[WiFi] 掉线 (Link lost), 等待自动重连 (auto-reconnecting)... 状态 (status) %d\n", WiFi.status());
        last_link_down = true;
      }
      vTaskDelay(pdMS_TO_TICKS(250));   // 掉线期降频轮询
      continue;
    }
    last_link_down = false;

    udp_poll();   // UDP 注册轮询 (HELLO) / poll UDP register

    // ---- 接收新客户端 ----
    WiFiClient new_client = tcp_server.available();
    if (new_client) {
      int slot = -1;
      for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
        if (!tcp_clients[i] || !tcp_clients[i].connected()) {
          if (tcp_clients[i]) tcp_clients[i].stop();
          tcp_clients[i] = new_client;
          tcp_clients[i].setNoDelay(true);
          slot = i;
          break;
        }
      }
      if (slot >= 0) {
        int sndbuf = 16384;   // 大 sndbuf: 4KB 整帧单次入流概率更高
        int sfd = tcp_clients[slot].fd();
        if (sfd >= 0) setsockopt(sfd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
        USBSerial.printf("[WiFi] 客户端 (Client) %s 已连接 (connected) → 槽 (slot) %d\n",
                         new_client.remoteIP().toString().c_str(), slot);
        // 诊断: 实测 sndbuf — Arduino 预编译库 LWIP_SO_SNDBUF 未启用时
        // setsockopt 失败, 缓冲固定 TCP_SND_BUF=5744 → 每 ACK 周期只能发 1 帧
        // diag: actual sndbuf — prebuilt lib lacks SO_SNDBUF → fixed 5744
        int cur = 0;
        socklen_t clen = sizeof(cur);
        if (sfd >= 0 && getsockopt(sfd, SOL_SOCKET, SO_SNDBUF, &cur, &clen) == 0)
          USBSerial.printf("[WiFi] 槽%d sndbuf=%d\n", slot, cur);
        else
          USBSerial.printf("[WiFi] 槽%d sndbuf不可查询 (not queryable) errno=%d (预编译库固定值 / fixed by prebuilt lib)\n", slot, errno);
      } else {
        USBSerial.printf("[WiFi] 客户端%s 被拒 (rejected): 槽位已满 (slots full)\n",
                         new_client.remoteIP().toString().c_str());
        new_client.stop();
      }
    }

    // ---- 出队 → 发送 (全帧率, 丢最旧) ----
    static wifi_queue_item_t item;   // 4KB 移出任务栈 / 4KB off task stack
    if (xQueueReceive(tcp_queue, &item, pdMS_TO_TICKS(50)) == pdTRUE) {
      udp_send_frame(item.data, item.len);    // UDP 满帧率通道 (先发, 实时优先)
      wifi_send_packet(item.data, item.len);  // TCP 兼容通道 (尽力发, 缓冲限制降帧)
    }

    // ---- 清理断开客户端 (释放槽位) ----
    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
      if (tcp_clients[i] && !tcp_clients[i].connected()) {
        USBSerial.printf("[WiFi] 槽%d 客户端断开 (client disconnected), 槽位释放 (slot released)\n", i);
        tcp_clients[i] = WiFiClient();   // 释放引用 → 关闭 socket / release handle
        wifi_client_stall[i] = 0;
      }
    }

    // ---- 周期状态日志已移除 (日志静音, 仅事件级打印) ----
    // periodic status log removed (silent runtime, event-level prints only)
  }
}

// ============================================================
//  WiFi 初始化 (WiFiManager + Preferences 双通道)
// ============================================================
static void wifi_init() {
  // ---- 运行时连接/重连策略: 不写 NVS (防 flash 磨损), 掉线自动重连 ----
  // runtime strategy: no NVS writes on runtime connect, auto-reconnect on drop
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);

  // ---- 检查 GPIO0 (BOOT按钮) — 按住进入配网模式 ----
  pinMode(BOOT_BTN_GPIO, INPUT_PULLUP);
  if (digitalRead(BOOT_BTN_GPIO) == LOW) {
    USBSerial.println("[WiFi] BOOT键按下 (BOOT pressed), 强制进入配网模式 (forcing provisioning mode)...");
    WiFiManager wm;
    wm.resetSettings();
    Preferences pref;
    pref.begin("ftdx10", false);
    pref.remove("wifi_ssid");
    pref.remove("wifi_pass");
    pref.end();
  }

  // ---- 尝试从 Preferences 读取保存的凭据 ----
  Preferences pref;
  pref.begin("ftdx10", true);
  String saved_ssid = pref.getString("wifi_ssid", "");
  String saved_pass = pref.getString("wifi_pass", "");
  pref.end();

  // ---- 启动 WiFiManager ----
  WiFiManager wm;
  wm.setConfigPortalTimeout(180);     // 配网页 3 分钟超时
  wm.setConnectTimeout(15);           // 连接超时 15 秒
  wm.setCaptivePortalEnable(true);    // 强制门户 (自动弹出配网页)
  wm.setAPCallback([](WiFiManager *wm) {
    USBSerial.printf("[WiFi] 热点已开启 (Hotspot on): %s\n", wm->getConfigPortalSSID().c_str());
    USBSerial.println("[WiFi] 用手机连接该热点设置WiFi (Connect with mobile to set WIFI)");
  });

  // 先尝试已保存的凭据
  if (saved_ssid.length() > 0) {
    USBSerial.printf("[WiFi] 尝试已保存配置 (Trying saved WiFi): %s\n", saved_ssid.c_str());
    WiFi.begin(saved_ssid.c_str(), saved_pass.c_str());

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
      delay(500);
      USBSerial.print(".");
      attempts++;
    }
  }

  // 如果没连上, 启动 WiFiManager 配网页
  if (WiFi.status() != WL_CONNECTED) {
    USBSerial.println("\n[WiFi] 尚未连接, 启动配网 (Not connected yet, starting WiFi setup)");
    USBSerial.println("[WiFi] AP: FTDX10_AP");

    if (!wm.autoConnect("FTDX10_AP")) {
      USBSerial.println("[WiFi] 配网超时/取消 (Timeout/Cancel)");
    }
  }

  // 检查最终状态
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.setSleep(false);   // 关闭 modem sleep: 消除省电唤醒延迟 (实时性关键) / disable modem sleep: kill wake-up latency
    USBSerial.printf("\n[WiFi] 就绪 (Ready) | IP: %s | RSSI: %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());

    tcp_queue = xQueueCreate(5, sizeof(wifi_queue_item_t));   // TCP: 5 深 (无线主通道缓冲) / TCP: 5-deep
    xTaskCreatePinnedToCore(wifi_task, "wifi", 8192, NULL, 3, NULL, 1);   // TCP 发送: 中等优先级, 栈 8KB 防溢出 / TCP TX: mid prio, 8KB stack
  } else {
    USBSerial.println("\n[WiFi] 未连接 (Not connected)! 仅 USB CDC 可用 (USB CDC only)");
    USBSerial.println("[WiFi] 通过 USB 串口发送 WIFI_SET <SSID> <PWD> 配置 (Send via USB serial)");
    USBSerial.println("[WiFi] 或按住 BOOT 键重启进入配网模式 (Or hold BOOT on reboot for AP mode)");
  }
}

// ============================================================
//  WiFi 初始化任务 (Core 1) — 异步执行, 不阻塞心跳/SPI
//  WiFi init task — runs async so heartbeat & SPI start instantly
// ============================================================
static void wifi_init_task(void *pvParameters) {
  wifi_init();
  vTaskDelete(NULL);
}

// ============================================================
//  SPI 接收任务 (Core 1, 优先级 6 = 全任务最高) — 与 USB 发送同核
//  数据链路 SPI→队列→USB 不走跨核传递; SPI 处理永不被 USB/WiFi/心跳抢占
//  SPI RX task (Core 1, prio 6 = highest) — same core as USB TX:
//  data path SPI→queue→USB stays on one core; SPI never preempted
// ============================================================
void spi_rx_task(void *pvParameters) {
  spi_bus_config_t buscfg = {
    .mosi_io_num = SPI_SLAVE_MOSI,
    .miso_io_num = -1,
    .sclk_io_num = SPI_SLAVE_SCLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = PACKET_SIZE + 64,   // 默认4092不够4096帧! / default 4092 < 4096!
  };
  spi_slave_interface_config_t slvcfg = {
    .spics_io_num = SPI_SLAVE_CS,
    .flags = 0,
    .queue_size = 4,       // ≥ 挂载事务数+1: 3 事务 + 余量 / >= queued txns + 1: 3 txns + spare
    .mode = 1,           // 电台 Mode 1 (CPOL=0, CPHA=1): 从机须在下降沿采样, 否则载荷全 0!
                         // radio is Mode 1: slave MUST sample on falling edge, else all-zero payload!
    .post_setup_cb = NULL,   // IDF 5.x: setup 在前 / setup first
    .post_trans_cb = NULL,   // trans 在后 / trans second
  };

  esp_err_t ret = spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg,
                                        SPI_DMA_CH_AUTO);
  if (ret != ESP_OK) {
    USBSerial.printf("[FATAL] SPI 初始化失败 (SPI init failed): %d\n", ret);
    vTaskDelete(NULL);
  }

  // CS 上拉: 防止空闲悬空时噪声毛刺截断帧 (大帧被拆成碎片的根因)
  // CS pull-up: reject noise glitches that split 4096B frames into fragments
  gpio_pullup_en((gpio_num_t)SPI_SLAVE_CS);

  // MOSI 上拉: 接线断开时读 0xFF (诊断), 开漏主机时保证高电平 / pull-up for diagnostics
  gpio_pullup_en((gpio_num_t)SPI_SLAVE_MOSI);

  // CS 毛刺滤波: 忽略 <200ns 的尖峰 (射频干扰/长线毛刺)
  // glitch filter: ignore spikes <200ns (RF interference / long-wire glitches)
  gpio_glitch_filter_handle_t cs_filter = NULL;
  gpio_pin_glitch_filter_config_t filter_cfg = {
    .clk_src = GLITCH_FILTER_CLK_SRC_DEFAULT,
    .gpio_num = (gpio_num_t)SPI_SLAVE_CS,
  };
  if (gpio_new_pin_glitch_filter(&filter_cfg, &cs_filter) == ESP_OK) {
    gpio_glitch_filter_enable(cs_filter);
  }
  USBSerial.printf("[SPI] 从机就绪 (Slave ready) | 模式1 (Mode 1) | 2MHz\n");

  // ===== 流水线三事务: CS 高 <4ms, 响应窗口极短 =====
  // 电台时序: 40ms 帧周期, 2MHz 稀疏时钟 (脉冲簇+间隙), CS 低≈36ms (32768 时钟),
  // CS 高<4ms → 处理+重新挂载必须 <4ms (实测 ~50μs, 余量充足);
  // 3 事务保证任意时刻队列 ≥2 备用, 处理耗时再大也不空窗
  // ===== 3-txn pipeline: CS-high <4ms, tight response window =====
  // radio timing: 40ms frame, 2MHz sparse clocks (bursts+gaps), CS-low≈36ms
  // (32768 clocks), CS-high<4ms → process+requeue must fit (<4ms; ~50μs real);
  // 3 txns keep >=2 spares queued, no blank window regardless of latency
  spi_slave_transaction_t trans[3];
  uint8_t *txn_bufs[3] = {spi_buf0, spi_buf1, spi_buf2};
  for (int i = 0; i < 2; i++) {
    memset(&trans[i], 0, sizeof(trans[i]));
    trans[i].length    = PACKET_SIZE * 8;  // 预设最大接收长度(位)! length=0 → DMA RX 描述符 0 字节
                                            // → 硬件照常数时钟(trans_len 正常)但数据从不写入缓冲 → 载荷全 0!
                                            // preset max RX bits! length=0 → zero-size DMA RX descriptor
                                            // → clocks counted but payload never written → all-zero!
    trans[i].rx_buffer = txn_bufs[i];
    trans[i].flags     = SPI_SLAVE_TRANS_DMA_BUFFER_ALIGN_AUTO;
    if (spi_slave_queue_trans(SPI2_HOST, &trans[i], portMAX_DELAY) != ESP_OK) {
      USBSerial.printf("[FATAL] SPI 预挂载失败 (SPI pre-queue failed)\n");
      vTaskDelete(NULL);
    }
  }

  while (true) {
    spi_slave_transaction_t *done = NULL;
    if (spi_slave_get_trans_result(SPI2_HOST, &done, portMAX_DELAY) != ESP_OK) continue;
    uint8_t *active_buf = (uint8_t *)done->rx_buffer;

    size_t rx_len = done->trans_len / 8;
    if (rx_len == 0) {
      // 无时钟事务 (CS 毛刺无 SCK): 不转发, 仅重新挂载 / no-clock xfer: re-queue only
      done->length = PACKET_SIZE * 8;
      spi_slave_queue_trans(SPI2_HOST, done, portMAX_DELAY);
      continue;
    }

    // ESP32-S3 从机已知怪癖: CS↑ 与末位时钟竞争 → trans_len 多计 8 位 (=4097B)
    // known quirk: CS↑ vs last-clock race → trans_len over-counts 8 bits (=4097B)
    // DMA 提交长 4096B → 缓冲数据完整 (逻辑分析仪确认主机时钟=32768), 仅截断计数
    // DMA was sized 4096B → buffer holds full frame (LA confirms 32768 clocks); clamp count only
    if (rx_len > PACKET_SIZE && rx_len <= PACKET_SIZE + 64) {
      trunc_count++;
      rx_len = PACKET_SIZE;
    }

    if (rx_len == PACKET_SIZE) {
      // 标准帧: 校验帧头位置, 诊断 SPI 数据是否从中间开始接收
      // verify magic at byte 0 (diagnose mid-stream alignment)
      if (!(active_buf[0] == 0x66 && active_buf[1] == 0xCC && active_buf[2] == 0xFF)) {
        misalign_count++;
        if (hb_watch_frames > 0) hb_bad_frames++;
      }
      usb_enqueue(active_buf, rx_len);
      wifi_enqueue_data(active_buf, rx_len);

    } else {
      // ===== STM32 策略: 数据包不完整也照样转发, 绝不丢弃 =====
      // 任何长度事务都封装发送 (usb_tx_task/write_frame: 66CCFF + len=实际 + XOR + 数据),
      // 下游按帧头+len 切帧 → 短帧被正常消费 → 数据流永远连续 → 时间轴无空洞无抖动
      // (STM32 send_frame 对 rx_len>0 无条件转发, 这就是它频谱不抖的根源)
      // ===== STM32 strategy: forward incomplete frames, never drop =====
      // (send_frame forwards ANY rx_len>0; downstream splits on magic+len, so the
      // stream stays continuous and the spectrum timeline never jumps)
      usb_enqueue(active_buf, rx_len);
      wifi_enqueue_data(active_buf, rx_len);

      if (rx_len < 32) {
        glitch_count++;   // CS 毛刺短事务 (<32B ≈ 16μs @2MHz) 也转发, 帧率保持连续
        if (hb_watch_frames > 0) hb_bad_frames++;
      } else {
        non_std_count++;   // 计数保留, 运行期静音 / counter kept, silent runtime
        if (hb_watch_frames > 0) hb_bad_frames++;
      }
    }

    // 观察窗口递减: 每处理一帧减 1 (先记异常后递减, 同帧一致)
    // watch window decrement: one frame per iteration (bad counted before decrement)
    if (hb_watch_frames > 0) hb_watch_frames--;

    // 处理完成 → 重新挂载 (队列保持满, 处理期 CS↓ 由另一个事务顶住)
    // re-queue after processing (other txn covers CS↓ during processing)
    done->length = PACKET_SIZE * 8;
    spi_slave_queue_trans(SPI2_HOST, done, portMAX_DELAY);
  }
}

// ============================================================
//  串口心跳 (电台)
// ============================================================
static void handle_radio_uart() {
  // 滑动窗口匹配: 每收 1 字节比较"最近 6 字节", 免疫任意字节错位
  // 旧固定缓冲: 电台未上电时 RX 垃圾把缓冲相位锁死 → 心跳永远错位不匹配
  // (先开 ESP32 再开电台时必现) → 电台 2-3s 后停发心跳 → 死锁
  // sliding-window: compare last 6 bytes on every byte → immune to any
  // misalignment; old fixed buffer locks wrong phase from pre-power-on
  // RX noise → heartbeat never matches when radio powers later
  static uint8_t rx_buf[HEARTBEAT_LEN];

  while (RadioSerial.available() > 0) {
    uint8_t c = RadioSerial.read();
    memmove(rx_buf, rx_buf + 1, HEARTBEAT_LEN - 1);
    rx_buf[HEARTBEAT_LEN - 1] = c;

    if (memcmp(rx_buf, HEARTBEAT, HEARTBEAT_LEN) == 0) {
      heartbeat_count++;
#if REPLY_HEARTBEAT
      // 回复心跳: 必须 10ms 内回复, 否则电台 2-3s 后停止 SPI 数据
      // reply: required within 10ms or radio halts SPI after 2-3s
      RadioSerial.write(HEARTBEAT, HEARTBEAT_LEN);
      RadioSerial.flush();
      hb_watch_frames = 2;   // 开观察窗: 回复后 2 帧内异常 → 坐实心跳耦合
                             // open watch: bad frame within next 2 → heartbeat coupling proven
#endif
#if HEARTBEAT_LED
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
#endif
    }
  }
}

// ============================================================
//  串口任务 (Core 0) — 心跳接收 + USB 命令处理
//  与 Core 1 数据链路 (SPI/USB/WiFi) 完全隔离, 串口活动零干扰
//  UART task (Core 0) — heartbeat + USB commands, fully isolated
//  from the Core 1 data path (SPI/USB/WiFi)
// ============================================================
static void uart_task(void *pvParameters) {
  while (true) {
    handle_radio_uart();
    handle_usb_commands();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ============================================================
//  初始化
// ============================================================
void setup() {
  // 日志口最先初始化, 确保诊断日志始终可见
  USBSerial.begin(115200);
  // 大 TX 缓冲: 日志写入即返回(内存拷贝), 后台 ISR 排空, 打印永不阻塞 SPI 任务
  // large TX buffer: log writes return instantly, ISR drains in background
  // → printf never stalls SPI RX task (256B default blocks ~10ms/line @115200)
  USBSerial.setTxBufferSize(2048);

  // ---- USB 诊断: 编译期配置 + begin() 结果 (排查 USB 数据口消失) ----
#if defined(ARDUINO_USB_CDC_ON_BOOT)
  USBSerial.printf("[USB] CDCOnBoot=%d", ARDUINO_USB_CDC_ON_BOOT);
#else
  USBSerial.printf("[USB] CDCOnBoot=<未定义/undefined>");
#endif
#if defined(ARDUINO_USB_MODE)
  USBSerial.printf(" | USBMode=%d\n", ARDUINO_USB_MODE);
#else
  USBSerial.println(" | USBMode=<未定义/undefined>");
#endif

  // 强制启动 TinyUSB: core 3.x 仅当 CDCOnBoot 启用时才自动调用,
  // 板级默认 Disabled → 不调 USB.begin() → TinyUSB 从未初始化 → USB 无数据!
  // force TinyUSB start: core 3.x auto-calls USB.begin() only when CDCOnBoot
  // is enabled; board default is Disabled → USB stays dead without this line
  usb_ready = USB.begin();
  USBSerial.printf("[USB] USB.begin() → %s\n",
                   usb_ready ? "OK (TinyUSB 已启动/started)" : "FAILED (TinyUSB 未启动/not started!)");
  USBSerial.println("[USB] 若 OK: 检查设备管理器是否出现新 COM 口 (需刷新/拔插) / check Device Manager for a new COM port");
  USBSerial.println("[USB] 若 FAILED: USB PHY 可能被 USB-Serial-JTAG 占用 / USB PHY may be occupied by USB-Serial-JTAG");

  Serial.begin(115200);

  USBSerial.println();
  USBSerial.println("╔═══════════════════════════════════╗");
  USBSerial.println("║  FTDX10 Adapter  —  ESP32-S3      ║");
  USBSerial.println("╚═══════════════════════════════════╝");
  USBSerial.printf("UART: GPIO%d(RX)←Pin12 | GPIO%d(TX)→Pin11 | 115200 8N1 (心跳/heartbeat)\n",
                RADIO_UART_RX, RADIO_UART_TX);
  USBSerial.printf("SPI:  GPIO%d(SCK)←Pin5 | GPIO%d(MOSI)←Pin4 | GPIO%d(CS)←Pin6 (频谱/spectrum)\n",
                SPI_SLAVE_SCLK, SPI_SLAVE_MOSI, SPI_SLAVE_CS);
  USBSerial.printf("WiFi: TCP Port:%d | UDP Port:%d | 配网 (WiFi Set): 长按BOOT键 (hold BOOT)\n", TCP_PORT, UDP_PORT);
  //USBSerial.println("输入 HELP 查看 USB 命令");
  USBSerial.println();

  RadioSerial.begin(115200, SERIAL_8N1, RADIO_UART_RX, RADIO_UART_TX);

  // UART1 TX 降驱动强度 (20mA→5mA): 减小 GPIO39(原 GPIO2) 翻转瞬态电流 → 削弱对相邻 SPI 引脚
  // (GPIO10/11/13) 的内部电源/封装耦合 (实测: 板载 UART1 回复心跳 → SPI 载荷采样错;
  // 外置串口无干扰 → 干扰路径在芯片内部/板内)。115200 低速, 5mA 驱动足够。
  // reduce TX drive (20mA→5mA): cut toggle transient, weaken coupling to adjacent
  // SPI pins (proven: on-board UART1 reply corrupts SPI payload; external UART clean)
  gpio_set_drive_capability((gpio_num_t)RADIO_UART_TX, GPIO_DRIVE_CAP_0);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // USB 发送队列 + 任务 (Core 1) — 参考 STM32 策略: 收发解耦
  // USB TX queue + task — decoupled from SPI RX like STM32
  // 优先级 5: WiFi(3)/心跳(1) 让位; SPI 接收(6) 优先于本任务 → SPI 实时性最高
  // prio 5: WiFi(3)/heartbeat(1) yield; SPI RX(6) preempts → SPI realtime first
  usb_queue = xQueueCreate(6, sizeof(usb_queue_item_t));
  xTaskCreatePinnedToCore(usb_tx_task, "usb_tx", 4096, NULL, 5, NULL, 1);

  // SPI 任务 → Core 1 (与 USB 发送同核, 数据链路零跨核; 优先级 6 = 最高)
  // SPI task → Core 1 (same core as USB TX, no cross-core data path; prio 6)
  xTaskCreatePinnedToCore(spi_rx_task, "spi_rx", 4096, NULL, 6, NULL, 1);

  // 串口任务 → Core 0: 心跳/命令与数据链路完全隔离
  // UART task → Core 0: heartbeat/commands isolated from data path
  xTaskCreatePinnedToCore(uart_task, "uart", 4096, NULL, 1, NULL, 0);

  // WiFi 初始化 → 异步任务 (Core 1), 不阻塞心跳回复与 SPI 接收
  xTaskCreatePinnedToCore(wifi_init_task, "wifi_init", 8192, NULL, 2, NULL, 1);

  USBSerial.println("[OK] 初始化完成 (Init complete), 等待电台心跳 (waiting for radio heartbeat)...\n");
}

void loop() {
  // 心跳与命令处理已移至 Core 0 (uart_task), 本任务仅空转保持结构
  // heartbeat & commands moved to Core 0 uart_task; this task just idles
  vTaskDelay(1000);
}
