import QtQuick 2.0
import "../basic" as Basic

Item {
  id: itemAutoPilotRun
  property double space: 10
  signal stop()

  Basic.Header {
    id: headerAutoPilotRun
    title: "AI智能扫地机"
    width: parent.width
    height: parent.height * 0.08
  }

  Basic.CarLocationState {
    id: locationStateRun
    x: space
    y: headerAutoPilotRun.height + space
    width: parent.width * 0.6
    height: parent.height - headerAutoPilotRun.height - space * 2;
  }

  Basic.AutoPilotRunPanel {
    id: autoPilotRunPanel
    x: locationStateRun.width + 2 * space
    y: headerAutoPilotRun.height + space
    width: parent.width - locationStateRun.width - 3 * space
    height: parent.height - headerAutoPilotRun.height - 2 * space;

    onStop: {
      itemAutoPilotRun.stop()
    }
  }
}
