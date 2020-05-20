#include "comms.h"

#include <pb_common.h>
#include <pb_decode.h>
#include <pb_encode.h>

#include "framing_rx_fsm.h"
#include "hal.h"
#include "network_protocol.pb.h"
#include "gtest/gtest.h"

bool UART_DMA::isTxInProgress() { return true; }
bool UART_DMA::isRxInProgress() { return true; }
bool UART_DMA::startTX(const uint8_t *buf, uint32_t length,
                       UART_DMA_TxListener *txl) {
  return false;
};

uint32_t UART_DMA::getRxBytesLeft() { return 0; };

void UART_DMA::stopTX(){};

bool UART_DMA::startRX(const uint8_t *buf, const uint32_t length,
                       const uint32_t timeout, UART_DMA_RxListener *rxl) {
  return 0;
};
void UART_DMA::stopRX(){};
void UART_DMA::charMatchEnable(){};
void UART_DMA::UART_ISR() {}
void UART_DMA::DMA_RX_ISR(){};
void UART_DMA::DMA_TX_ISR(){};
void UART_DMA::init(int baud){};

UART_DMA dmaUART = UART_DMA();
Comms comms(dmaUART);
// TestFsm rxFSM(dmaUART);

TEST(CommTests, SendControllerStatus) {
  // Initialize a large ControllerStatus so as to force multiple calls to
  // comms_handler to send it.
  ControllerStatus s = ControllerStatus_init_zero;
  s.uptime_ms = 42;
  s.active_params.mode = VentMode_PRESSURE_CONTROL;
  s.active_params.peep_cm_h2o = 10;
  s.active_params.breaths_per_min = 15;
  s.active_params.pip_cm_h2o = 1;
  s.active_params.inspiratory_expiratory_ratio = 2;
  s.active_params.rise_time_ms = 100;
  s.active_params.inspiratory_trigger_cm_h2o = 5;
  s.active_params.expiratory_trigger_ml_per_min = 9;
  // Set very large values here because they take up more space in the encoded
  // proto, and our goal is to make it big.
  s.active_params.alarm_lo_tidal_volume_ml =
      std::numeric_limits<uint32_t>::max();
  s.active_params.alarm_hi_tidal_volume_ml =
      std::numeric_limits<uint32_t>::max();
  s.active_params.alarm_lo_breaths_per_min =
      std::numeric_limits<uint32_t>::max();
  s.active_params.alarm_hi_breaths_per_min =
      std::numeric_limits<uint32_t>::max();
  s.sensor_readings.patient_pressure_cm_h2o = 11;
  s.sensor_readings.volume_ml = 800;
  s.sensor_readings.flow_ml_per_min = 1000;

  // Run comms_handler until it stops sending data.  10 iterations should be
  // more than enough.
  for (int i = 0; i < 10; i++) {
    GuiStatus gui_status_ignored = GuiStatus_init_zero;
    comms.handler(s, &gui_status_ignored);
  }
  char tx_buffer[ControllerStatus_size];
  uint16_t len = 15;
  // Hal.test_serialGetOutgoingData(tx_buffer, sizeof(tx_buffer));
  ASSERT_GT(len, 0);
  pb_istream_t stream =
      pb_istream_from_buffer(reinterpret_cast<unsigned char *>(tx_buffer), len);

  ControllerStatus sent = ControllerStatus_init_zero;
  ASSERT_TRUE(pb_decode(&stream, ControllerStatus_fields, &sent));
  EXPECT_EQ(s.uptime_ms, sent.uptime_ms);
  EXPECT_EQ(s.active_params.mode, sent.active_params.mode);
  EXPECT_EQ(s.active_params.peep_cm_h2o, sent.active_params.peep_cm_h2o);
  EXPECT_EQ(s.sensor_readings.patient_pressure_cm_h2o,
            sent.sensor_readings.patient_pressure_cm_h2o);
}

TEST(CommTests, DISABLED_CommandRx) {
  GuiStatus s = GuiStatus_init_zero;
  s.uptime_ms = std::numeric_limits<uint32_t>::max() / 2;
  s.desired_params.mode = VentMode_PRESSURE_CONTROL;
  s.desired_params.peep_cm_h2o = 10;
  s.desired_params.breaths_per_min = 15;
  s.desired_params.pip_cm_h2o = 1;
  s.desired_params.inspiratory_expiratory_ratio = 2;
  s.desired_params.rise_time_ms = 100;
  s.desired_params.inspiratory_trigger_cm_h2o = 5;
  s.desired_params.expiratory_trigger_ml_per_min = 9;
  // Set very large values here because they take up more space in the encoded
  // proto, and our goal is to make it big.
  s.desired_params.alarm_lo_tidal_volume_ml =
      std::numeric_limits<uint32_t>::max();
  s.desired_params.alarm_hi_tidal_volume_ml =
      std::numeric_limits<uint32_t>::max();
  s.desired_params.alarm_lo_breaths_per_min =
      std::numeric_limits<uint32_t>::max();
  s.desired_params.alarm_hi_breaths_per_min =
      std::numeric_limits<uint32_t>::max();

  char rx_buffer[GuiStatus_size];
  pb_ostream_t stream = pb_ostream_from_buffer(
      reinterpret_cast<unsigned char *>(rx_buffer), sizeof(rx_buffer));
  pb_encode(&stream, GuiStatus_fields, &s);
  EXPECT_GT(stream.bytes_written, 0u);
  // Hal.test_serialPutIncomingData(rx_buffer,
  //                                static_cast<uint16_t>(stream.bytes_written));
  // EXPECT_GT(Hal.serialBytesAvailableForRead(), 0);

  ControllerStatus controller_status_ignored = ControllerStatus_init_zero;
  GuiStatus received = GuiStatus_init_zero;

  // Run comms_handler until it updates GuiStatus.  10 iterations should be
  // more than enough to read the whole thing.
  for (int i = 0; i < 10; i++) {
    comms.handler(controller_status_ignored, &received);
    // We use a timeout for framing packets, so we have to advance the time,
    // otherwise we'll never think the packet is complete!
    Hal.delay(milliseconds(1));
  }
  EXPECT_EQ(s.uptime_ms, received.uptime_ms);
  EXPECT_EQ(s.desired_params.mode, received.desired_params.mode);
}
