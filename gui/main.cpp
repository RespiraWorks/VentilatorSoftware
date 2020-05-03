#include "chrono.h"
#include "connected_device.h"
#include "controller_history.h"
#include "gui_state_container.h"
#include "periodic_closure.h"
#include "respira_connected_device.h"

#include <QCommandLineParser>
#include <QtCore/QDir>
#include <QtQml/QQmlContext>
#include <QtQml/QQmlEngine>
#include <QtQuick/QQuickView>
#include <QtWidgets/QApplication>
#include <cmath>
#include <iostream>
#include <memory>

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  app.setWindowIcon(QIcon(":/Logo.png"));

  QCommandLineParser parser;
  parser.setApplicationDescription("Ventilator GUI application");
  parser.addHelpOption();
  // Support this option so we can verify that the GUI can start up at all,
  // e.g. load all resources and so on.
  QCommandLineOption startupOnlyOption(
      QStringList() << "startup-only",
      QApplication::translate("main",
                              "Start up and exit successfully (for testing)"));
  QCommandLineOption serialPortOption(
      QStringList() << "serial-port",
      QApplication::translate(
          "main", "Serial port filename. Uses fake data if not set."));
  parser.addOption(startupOnlyOption);
  parser.addOption(serialPortOption);
  parser.process(app);

  GuiStateContainer state_container(
      /*history_window=*/DurationMs(30000));
  auto startup_time = state_container.GetStartupTime();

  // Check out utils/mock-cycle-controller.py - it is
  // a script that bangs ControllerState at the set rate into serial
  // port.

  std::unique_ptr<ConnectedDevice> device;
  if (parser.isSet(serialPortOption)) {
    device = std::make_unique<RespiraConnectedDevice>(
        parser.value(serialPortOption));
  } else {
    device = std::make_unique<FakeConnectedDevice>(
        [&](const GuiStatus &gui_status) {
          // For now, completely ignore gui_status.
          (void)gui_status;
        },
        [&](ControllerStatus *controller_status) {
          // Fill the status with fake data.
          controller_status->uptime_ms =
              TimeAMinusB(SteadyClock::now(), startup_time).count();
          auto *sensors = &controller_status->sensor_readings;
          sensors->pressure_cm_h2o = sin(controller_status->uptime_ms * 0.001);
          sensors->volume_ml = sin(controller_status->uptime_ms * 0.002);
          sensors->flow_ml_per_min = sin(controller_status->uptime_ms * 0.003);
        });
  }

  // TODO: Bind the readable aspects of state_container to UI elements
  // TODO: Bind the writable aspects of state_container to parameters set in the
  // UI

  // TODO: Figure out whether asynchronously and periodically sending GUI status
  // and receiving controller status is the right thing to do here.

  PeriodicClosure communicate(DurationMs(10), [&] {
    device->SendGuiStatus(state_container.GetGuiStatus());
    ControllerStatus controller_status;
    if (device->ReceiveControllerStatus(&controller_status)) {
      state_container.AppendHistory(controller_status);
    }
  });
  communicate.Start();

  QQuickView mainView;
  mainView.setTitle(QStringLiteral("Ventilator"));

  mainView.rootContext()->setContextProperty("stateContainer",
                                             &state_container);

  mainView.setSource(QUrl("qrc:/main.qml"));
  mainView.setResizeMode(QQuickView::SizeRootObjectToView);
  mainView.setColor(QColor("#000000"));

  mainView.show();

  if (parser.isSet(startupOnlyOption)) {
    return EXIT_SUCCESS;
  }

  return app.exec();
}
