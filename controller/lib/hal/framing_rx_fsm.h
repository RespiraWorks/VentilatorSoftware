#include "network_protocol.pb.h"
#include "uart_dma.h"

extern UART_DMA dmaUART;

class FramingRxFSM : public UART_DMA_RxListener {
  enum State_t { STATE_LOST, STATE_WAIT_START, STATE_RX_FRAME };
  static constexpr uint32_t RX_TIMEOUT = 115200 * 10;

  State_t state = STATE_LOST;
  uint32_t errorCounter = 0;
  // Size of the buffer is set asuming a corner case where EVERY GuiStatus
  // byte and CRC32 will be escaped + two marker chars; this is too big, but
  // safe.
  static constexpr uint32_t RX_BUF_LEN = (GuiStatus_size + 4) * 2 + 2;
  static constexpr uint32_t RX_BYTES_MAX = RX_BUF_LEN;
  uint8_t rx_buf[RX_BUF_LEN];

public:
  void begin();
  void onRxComplete();
  void onCharacterMatch();
  void onRxError(RxError_t e);

private:
  uint32_t receivedBytesCount();
  void restartRX();
  void processReceivedData();
};
