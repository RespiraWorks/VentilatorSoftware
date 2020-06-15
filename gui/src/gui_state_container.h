#ifndef GUI_STATE_CONTAINER_H
#define GUI_STATE_CONTAINER_H

#include <QObject>
#include <stdint.h>

#include "chrono.h"
#include "controller_history.h"
#include "simple_clock.h"

#include <iostream>
#include <tuple>
#include <vector>

#include <QPointF>
#include <QVector>
#include <QtCore/QObject>

// A thread-safe container for readable and writable state
// of the GUI.
//
// The rest of the GUI must bind itself to accessors and mutators
// of this class - e.g. render graphs from GetControllerStatusHistory(),
// and when a parameter is changed in the UI, call a mutator on this
// object.
//
// In other words, this is essentially an MVC "Model".
//
// TODO(jkff, paulovap): This class embodies the "God object" antipattern. We
// should split it into several parts, with GuiStateContainer being only the
// entry point for them.
class GuiStateContainer : public QObject {
  Q_OBJECT

public:
  enum VentilationMode {
    COMMAND_PRESSURE,
    PRESSURE_ASSIST,
    HIGH_FLOW_NASAL_CANNULA,
  };
  Q_ENUM(VentilationMode)

  // Initializes the state container to keep the history of controller
  // statuses in a given time window.
  GuiStateContainer(DurationMs history_window)
      : startup_time_(SteadyClock::now()), history_(history_window) {}

  // Returns when the GUI started up.
  SteadyInstant GetStartupTime() { return startup_time_; }

  // Returns the current GuiStatus to be sent to the controller.
  GuiStatus GetGuiStatus() {
    GuiStatus status = GuiStatus_init_zero;

    status.uptime_ms = TimeAMinusB(SteadyClock::now(), startup_time_).count();
    status.desired_params.mode = [&] {
      switch (commanded_mode_) {
      case VentilationMode::COMMAND_PRESSURE:
        return VentMode::VentMode_PRESSURE_CONTROL;
      case VentilationMode::PRESSURE_ASSIST:
        return VentMode::VentMode_PRESSURE_ASSIST;
      case VentilationMode::HIGH_FLOW_NASAL_CANNULA:
        return VentMode::VentMode_HIGH_FLOW_NASAL_CANNULA;
      default:
        // Should never happen.
        std::cerr << "Unexpected commanded_mode: " << commanded_mode_
                  << std::endl;
        return VentMode::VentMode_PRESSURE_CONTROL;
      }
    }();
    status.desired_params.peep_cm_h2o = commanded_peep_;
    status.desired_params.breaths_per_min = commanded_rr_;
    status.desired_params.pip_cm_h2o = commanded_pip_;
    float breath_duration_sec = 60.0 / commanded_rr_;
    float commanded_e_time = breath_duration_sec - commanded_i_time_;
    status.desired_params.inspiratory_expiratory_ratio =
        commanded_i_time_ / commanded_e_time;

    return status;
  }

  // Returns the recent history of ControllerStatus.
  std::vector<std::tuple<SteadyInstant, ControllerStatus>>
  GetControllerStatusHistory() {
    return history_.GetHistory();
  }

  // Measured parameters
  Q_PROPERTY(qreal measured_pressure READ get_measured_pressure NOTIFY
                 measurements_changed)
  Q_PROPERTY(
      qreal measured_flow READ get_measured_flow NOTIFY measurements_changed)
  Q_PROPERTY(qreal measured_tv READ get_measured_tv NOTIFY measurements_changed)
  Q_PROPERTY(
      quint32 measured_rr READ get_measured_rr NOTIFY measurements_changed)
  Q_PROPERTY(
      quint32 measured_peep READ get_measured_peep NOTIFY measurements_changed)
  Q_PROPERTY(
      quint32 measured_pip READ get_measured_pip NOTIFY measurements_changed)
  Q_PROPERTY(
      qreal measured_ier READ get_measured_ier NOTIFY measurements_changed)

  // Graphs
  Q_PROPERTY(QVector<QPointF> pressureSeries READ GetPressureSeries NOTIFY
                 PressureSeriesChanged)
  Q_PROPERTY(
      QVector<QPointF> flowSeries READ GetFlowSeries NOTIFY FlowSeriesChanged)
  Q_PROPERTY(QVector<QPointF> tidalSeries READ GetTidalSeries NOTIFY
                 TidalSeriesChanged)

  // Commanded parameters
  Q_PROPERTY(VentilationMode commanded_mode MEMBER commanded_mode_ NOTIFY
                 params_changed)
  Q_PROPERTY(quint32 commanded_rr MEMBER commanded_rr_ NOTIFY params_changed)
  Q_PROPERTY(
      quint32 commanded_peep MEMBER commanded_peep_ NOTIFY params_changed)
  Q_PROPERTY(quint32 commanded_pip MEMBER commanded_pip_ NOTIFY params_changed)
  Q_PROPERTY(
      qreal commanded_i_time MEMBER commanded_i_time_ NOTIFY params_changed)

  // Other properties
  Q_PROPERTY(int batteryPercentage READ get_battery_percentage NOTIFY
                 battery_percentage_changed)
  Q_PROPERTY(SimpleClock *clock READ get_clock NOTIFY clock_changed)

  QVector<QPointF> GetPressureSeries() const { return pressure_series_; }

  void SetPressureSeries(QVector<QPointF> &&series) {
    pressure_series_ = series;
    emit PressureSeriesChanged();
  }

  QVector<QPointF> GetFlowSeries() const { return flow_series_; }

  void SetFlowSeries(QVector<QPointF> &&series) {
    flow_series_ = series;
    emit FlowSeriesChanged();
  }

  QVector<QPointF> GetTidalSeries() const { return tidal_series_; }

  void SetTidalSeries(QVector<QPointF> &&series) {
    tidal_series_ = series;
    emit TidalSeriesChanged();
  }

signals:
  void measurements_changed();
  void params_changed();
  void battery_percentage_changed();
  void clock_changed();
  void PressureSeriesChanged();
  void FlowSeriesChanged();
  void TidalSeriesChanged();

public slots:
  // Adds a data point of controller status to the history.
  void controller_status_changed(SteadyInstant now,
                                 const ControllerStatus &status) {
    history_.Append(now, status);
    measurements_changed();
  }

  void update();

private:
  int get_battery_percentage() const {
    return battery_percentage_;
    // TODO: Figure our how battery will be implemented
    // and how to get estimation.
  }
  SimpleClock *get_clock() const { return const_cast<SimpleClock *>(&clock_); }

  // ====================== Measured parameters ========================
  qreal get_measured_pressure() const {
    return history_.GetLastStatus().sensor_readings.patient_pressure_cm_h2o;
  }
  qreal get_measured_flow() const {
    return 0.001 * history_.GetLastStatus().sensor_readings.flow_ml_per_min;
  }
  qreal get_measured_tv() const {
    return history_.GetLastStatus().sensor_readings.volume_ml;
  }
  qreal get_measured_rr() const {
    // TODO: Compute based on last breath. For now, return commanded value.
    return commanded_rr_;
  }
  qreal get_measured_peep() const {
    // TODO: Compute based on last breath. For now, return commanded value.
    return commanded_peep_;
  }
  qreal get_measured_pip() const {
    // TODO: Compute based on last breath. For now, return commanded value.
    return commanded_pip_;
  }
  qreal get_measured_ier() const {
    // TODO: Compute based on last breath. For now, return commanded value.
    float breath_duration_sec = 60.0 / commanded_rr_;
    float commanded_e_time = breath_duration_sec - commanded_i_time_;
    return commanded_i_time_ / commanded_e_time;
  }

  // ====================== Commanded parameters ========================
  const SteadyInstant startup_time_;
  ControllerHistory history_;
  int battery_percentage_ = 70;
  SimpleClock clock_;

  QVector<QPointF> pressure_series_;
  QVector<QPointF> flow_series_;
  QVector<QPointF> tidal_series_;

  // Commanded parameters
  // Initialize to default parameters like in
  // https://github.com/RespiraWorks/VentilatorSoftware/blob/89b817af/controller/src/main.cpp#L84
  VentilationMode commanded_mode_ = VentilationMode::COMMAND_PRESSURE;
  quint32 commanded_rr_ = 12;
  quint32 commanded_pip_ = 15;
  quint32 commanded_peep_ = 5;
  qreal commanded_i_time_ = 1.0;
};

#endif // GUI_STATE_CONTAINER_H
