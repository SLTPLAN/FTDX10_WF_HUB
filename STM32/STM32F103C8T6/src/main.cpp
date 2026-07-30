// ============================================================
//  文件: main.cpp
//  File: main.cpp
//  描述: STM32F103C8T6 转接板固件 — FTDX10 电台数据采集
//  Desc: adapter firmware — FTDX10 radio data acquisition
//  电台 ACC Pinout / Radio ACC connector:
//    Pin4  (MOSI)  → 数据 Data   Pin5  (SCLK) → 时钟 Clock
//    Pin6  (CS)    → 片选 Chip Select
//  协议: SPI 模式1 2MHz 从机 → USB CDC
//  Protocol: SPI mode1 slave 2MHz → USB CDC
//  输出: USB CDC (虚拟串口) via SerialUSB
//  Output: USB CDC (virtual serial) via SerialUSB
//  平台: PlatformIO + STM32duino (Arduino_Core_STM32)
//  Platform: PlatformIO + STM32duino
// ============================================================

#include <Arduino.h>
#include <stdint.h>
#include <USBSerial.h>

// ============================================================
//  引脚定义
//  Pin Definitions
// ============================================================
#define PIN_SPI_CS          PA4
#define PIN_SPI_SCK         PA5
#define PIN_SPI_MOSI        PA7
#define PIN_LED             PC13
#define LED_ON              LOW
#define LED_OFF             HIGH

// ============================================================
//  协议常量
//  Protocol Constants
// ============================================================
#define PACKET_SIZE          4096

// ============================================================
//  全局变量
//  Global Variables
// ============================================================
static uint8_t spi_buffer[PACKET_SIZE];        // SPI 单缓冲 / single SPI buffer
static volatile uint16_t spi_rx_index = 0;     // 当前接收位置 / current RX index
static volatile bool     packet_ready = false;  // 包就绪标志 / frame complete flag
static volatile bool     spi_active   = false;  // SPI 正在接收 / SPI actively receiving

static uint32_t packet_count = 0;
static uint32_t total_bytes  = 0;

// ============================================================
//  SPI1 中断 — 安全网 (DMA 为主, 仅在溢出时触发)
//  SPI1 ISR — safety net (DMA primary, fires on overrun only)
// ============================================================
extern "C" void SPI1_IRQHandler(void) {
  if (SPI1->SR & SPI_SR_OVR) {
    (void)SPI1->DR;
    (void)SPI1->SR;
  }
}

// ============================================================
//  CS 引脚中断 — 硬件 NSS + DMA 帧控制 (完整复位序列)
//  CS Pin ISR — HW NSS + DMA frame control (full reset sequence)
// ============================================================
static void cs_interrupt_handler(void) {
  if (!(GPIOA->IDR & (1 << 4))) {        // CS ↓ — 帧开始
    // ============ 极简路径: 缩小竞争窗口 ============
    // Fast path: minimize window between CS↓ and DMA enable
    DMA1->IFCR = DMA_IFCR_CTCIF2        // 清除所有标志 / clear all flags for CH2
               | DMA_IFCR_CTEIF2
               | DMA_IFCR_CGIF2;
    DMA1_Channel2->CCR = 0;              // 关断 / disable
    DMA1_Channel2->CNDTR = PACKET_SIZE;  // 设计数 / set transfer count
    DMA1_Channel2->CCR = DMA_CCR_MINC    // 内存递增 / mem inc
                       | DMA_CCR_PSIZE_0 // 16位匹配 SPI DR / 16-bit width
                       | DMA_CCR_EN;     // 使能 / enable
    spi_active = true;

  } else if (spi_active) {               // CS ↑ — 帧结束
    // ============ 精准停止, 读取计数 ============
    DMA1_Channel2->CCR = 0;              // 发起停止 / request stop
    while (DMA1_Channel2->CCR & DMA_CCR_EN); // 等待真正停稳 / wait for confirmed stop

    uint16_t remaining = DMA1_Channel2->CNDTR;
    uint16_t received = PACKET_SIZE - remaining;
    if (received > 0) {
      spi_rx_index = received;
      packet_ready = true;
    }
    spi_active = false;
  }
}

// ============================================================
//  SPI 从机初始化
//  SPI Slave Initialization
// ============================================================
static void spi_slave_init(void) {
  // ---- 使能时钟 / Enable clocks ----
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;

  // ---- PA4(CS) - 浮空输入 (硬件 NSS) / floating input (HW NSS) ----
  GPIOA->CRL &= ~(0xF << (4 * 4));
  GPIOA->CRL |=  (0x4 << (4 * 4));

  // ---- PA5(SCK) - 浮空输入 / floating input ----
  GPIOA->CRL &= ~(0xF << (5 * 4));
  GPIOA->CRL |=  (0x4 << (5 * 4));

  // ---- PA6(MISO) - 复用推挽输出 50MHz / AF PP 50MHz ----
  GPIOA->CRL &= ~(0xF << (6 * 4));
  GPIOA->CRL |=  (0xB << (6 * 4));

  // ---- PA7(MOSI) - 浮空输入 / floating input ----
  GPIOA->CRL &= ~(0xF << (7 * 4));
  GPIOA->CRL |=  (0x4 << (7 * 4));

  // ---- DMA1 通道2: SPI1_RX (只设常量, CNDTR/EN 在 CS↓ 中设) ----
  // DMA1 Channel 2: SPI1_RX (constants only; CNDTR/EN set in CS↓)
  DMA1_Channel2->CCR = 0;
  DMA1_Channel2->CPAR = (uint32_t)&SPI1->DR;
  DMA1_Channel2->CMAR = (uint32_t)spi_buffer;
  DMA1_Channel2->CNDTR = 0;
  (void)DMA1_Channel2->CCR;                // 回读确认 / read-back confirmation

  // ---- SPI1: 从机, 模式1, 硬件 NSS + DMA RX 请求 ----
  //  Slave, mode1, HW NSS + DMA RX request
  SPI1->CR1 = 0;
  SPI1->CR1 = SPI_CR1_CPHA;                // 模式1, MSTR=0(从机), SSM=0(硬件NSS)
  SPI1->CR1 &= ~SPI_CR1_LSBFIRST;          // MSB 先行 / MSB first
  SPI1->CR2 = SPI_CR2_RXDMAEN;             // DMA RX 请求使能 / DMA RX request enable
  SPI1->CR1 |= SPI_CR1_SPE;                // 使能 SPI / enable SPI

  // ---- NVIC ----
  NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 3);
  NVIC_SetPriority(EXTI4_IRQn, 1);

  // ---- EXTI ----
  pinMode(PIN_SPI_CS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_SPI_CS), cs_interrupt_handler, CHANGE);

  SerialUSB.printf("[SPI] DMA 就绪 | 模式1 | HW NSS | DMA1_CH2");
  SerialUSB.println();
}

// ============================================================
//  帧封装 (USB CDC)
//  Frame Encapsulation (USB CDC)
// ============================================================
static void send_frame(const uint8_t *data, size_t len) {
  uint8_t xor_sum = 0;
  for (size_t i = 0; i < len; i++) xor_sum ^= data[i];

  uint8_t header[] = {0x66, 0xCC, 0xFF};
  SerialUSB.write(header, 3);
  SerialUSB.write((len >> 8) & 0xFF);
  SerialUSB.write(len & 0xFF);
  SerialUSB.write(xor_sum);

  // 分块发送, 避免长时间阻塞; 用实际写入量重试 / chunked send, retry on partial write
  size_t offset = 0;
  while (offset < len) {
    size_t chunk = len - offset;
    if (chunk > 64) chunk = 64;
    size_t written = SerialUSB.write(data + offset, chunk);
    if (written == 0) break;  // USB 断开则放弃 / give up if USB disconnected
    offset += written;
  }
}

// ============================================================
//  USB 串口命令
//  USB Serial Commands
// ============================================================
#define CMD_BUF_SIZE  160
static char cmd_buf[CMD_BUF_SIZE];
static uint8_t cmd_idx = 0;

static void handle_usb_commands(void) {
  while (SerialUSB.available() > 0) {
    char c = SerialUSB.read();
    if (c == '\n' || c == '\r') {
      if (cmd_idx > 0) {
        cmd_buf[cmd_idx] = '\0';
        if (strcmp(cmd_buf, "HELP") == 0) {
          SerialUSB.println("[CMD] 可用命令:");
          SerialUSB.println("  HELP  — 显示帮助 / show help");
        } else if (cmd_buf[0] != '\0') {
          SerialUSB.printf("[CMD] 未知: %s\n", cmd_buf);
        }
        cmd_idx = 0;
      }
    } else if (cmd_idx < CMD_BUF_SIZE - 1) {
      cmd_buf[cmd_idx++] = c;
    }
  }
}

// ============================================================
//  初始化
//  Setup / Initialization
// ============================================================
void setup(void) {
  SerialUSB.begin();
  delay(500);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LED_OFF);

  SerialUSB.println();
  SerialUSB.println("╔═══════════════════════════════════╗");
  SerialUSB.println("║  FTDX10 转接板  —  STM32F103C8   ║");
  SerialUSB.println("╚═══════════════════════════════════╝");
  SerialUSB.println("SPI:  PA4(CS)←Pin6 | PA5(SCK)←Pin5 | PA7(MOSI)←Pin4 | 模式1");
  SerialUSB.println("输出: USB CDC (虚拟串口) / Output: USB CDC");
  SerialUSB.println();

  spi_slave_init();

  SerialUSB.println("[OK] 初始化完成, 等待电台 SPI 数据...\n");
  SerialUSB.println("[OK] Init done, waiting for radio SPI data...\n");
}

// ============================================================
//  主循环
//  Main Loop
// ============================================================
void loop(void) {
  handle_usb_commands();

  if (packet_ready) {
    packet_ready = false;

    size_t rx_len = spi_rx_index;
    if (rx_len > 0) {
      packet_count++;
      total_bytes += rx_len;

      send_frame(spi_buffer, rx_len);

      if (rx_len == PACKET_SIZE) {
        static unsigned long last_summary = 0;
        unsigned long now = millis();
        if (now - last_summary > 5000) {
          SerialUSB.printf("[SPI] 包 #%u | %u B | 总: %u KB\n",
                        packet_count, rx_len, total_bytes / 1024);
          last_summary = now;
        }
      } else {
        SerialUSB.printf("[SPI] !!! #%u | %u B (非标准)\n",
                        packet_count, rx_len);
      }
    }
  }
}
