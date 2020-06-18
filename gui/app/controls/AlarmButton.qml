import QtQuick 2.11
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.4
import QtGraphicalEffects 1.0
import Respira 1.0
import ".."

HeaderButton {
    id: root

    property int alarmsCount: 0

    property alias elapsedTimeText: elapsedTime.text
    property color alarm_button_fg: "white"
    property int priority: AlarmPriority.NONE

    width: elapsedTime.visible ? 168: 104;
    height: 40

    state: switch(priority) {
        case AlarmPriority.NONE: "";   break;
        case AlarmPriority.LOW: "low"; break;
        case AlarmPriority.MEDIUM: "medium"; break;
        case AlarmPriority.HIGH: "high"; break;
    }


    states: [
        State {
            name: "high"
            PropertyChanges {
                target: root;
                color: root.checked || root.down ? "black" : Style.theme.color.alarmHighBright
                alarm_button_fg: "white"
                restoreEntryValues: false
            }
        },
        State {
            name: "medium"
            PropertyChanges {
                target: root;
                color: root.checked || root.down ? "white" : Style.theme.color.alarmMediumBright
                alarm_button_fg: "black"
                restoreEntryValues: false
            }
        },
        State {
            name: "low"
            PropertyChanges {
                target: root;
                color: root.checked || root.down ? "black" : Style.theme.color.alarmLowBright
                alarm_button_fg: "white"
                restoreEntryValues: false
            }
        }
    ]

    contentItem: Item {

        Row {
            id: rect
            anchors.centerIn: parent
            spacing: 8
            Text {
                id: alarmCountText
                anchors.verticalCenter: parent.verticalCenter
                text: root.alarmsCount.toString()
                font: Style.theme.font.headerButton
                color: alarm_button_fg
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                elide: Text.ElideRight
            }

            Image {
                id: icon
                anchors.verticalCenter: parent.verticalCenter
                width: 20; height: 20
                sourceSize: Qt.size(width, height)
                fillMode: Image.PreserveAspectFit
                source: root.priority != AlarmPriority.NONE ?
                            'qrc:/images/RW_alarm-off_24.svg' :
                            'qrc:/images/RW_alarm_24.svg'
                layer.enabled: true
                layer.effect: ColorOverlay {
                    anchors.fill: icon
                    source: icon
                    color: root.alarm_button_fg
                }
            }

            Text {
                id: elapsedTime
                anchors.verticalCenter: parent.verticalCenter
                width: visible ? implicitWidth : 0
                visible: text.length > 0
                font: Style.theme.font.headerButton
                color: alarm_button_fg
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
        }
    }
}
