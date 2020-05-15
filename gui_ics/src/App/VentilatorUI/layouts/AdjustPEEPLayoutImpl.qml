//NOTE: Please remove this line to prevent this file being overwritten by the flow generation process

import GreenHouse 1.0
import QtQuick 2.6
import VentilatorUI 1.0 as VentilatorUISet
import GreenHouse.Theming 1.0

import "qrc:/VentilatorUI/private/"
import "qrc:/VentilatorUI/animations/"
import "qrc:/VentilatorUI/interfaces/"

AdjustPEEPLayout {
    id: root

    property bool fillParent: false
    x: fillParent ? 0 : 0
    y: fillParent ? 0 : 0
    width: fillParent ? (parent ? parent.width : 0) : 1024
    height: fillParent ? (parent ? parent.height : 0) : 600

    inputFallthroughBlocked: true

    VentilatorUISet.ScreenBackground {
        id: fi_ScreenBackground
        x: 0
        y: 60
        z: 0
        width: 1024
        height: 540
        opacity: 1
        enabled: true
        visible: true
        clip: false
        rotation: 0
        scale: 1
    }
    Item {
        id: fi_PopupFrame
        x: 199
        y: 141
        z: 1
        width: 626
        height: 377
        opacity: 1
        enabled: true
        visible: true
        clip: false
        rotation: 0
        scale: 1

        VentilatorUISet.BoundingBox {
            id: fi_BoundingBox
            x: 0
            y: 0
            z: 0
            width: 626
            height: 377
            opacity: 1
            enabled: true
            visible: true
            clip: false
            fi_Title_color: GreenHouseThemeManager.theme.color("buttonAltTextColor")
            fi_Title_horizontalAlignment: Text.AlignHCenter
            fi_Title_opacity: 0.85
            fi_Title_text: qsTr("PEEP (cmH20)")
            fi_Title_verticalAlignment: Text.AlignVCenter
            rotation: 0
            scale: 1
        }
        Text {
            id: fi_Value
            x: 130
            y: 49
            z: 3
            width: 366
            height: 196
            text: VentilatorParametersInterface.peepPopup
            font: GreenHouseThemeManager.theme.font("Figma::Noto Serif_NotoSerif-Bold_144_75_0_0")
            color: GreenHouseThemeManager.theme.color("buttonAltTextColor")
            opacity: 1
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            elide: Text.ElideNone
            wrapMode: Text.WordWrap
            maximumLineCount: 2147483647
            enabled: true
            visible: true
            clip: false
            rotation: 0
            scale: 1
        }
        VentilatorUISet.Button_Minus_Default_State {
            id: fi_DecrementButton
            x: 40
            y: 102
            z: 4
            width: 90
            height: 90
            opacity: 1
            enabled: true
            visible: true
            clip: false
            icon_opacity: 1
            icon_source: GreenHouseThemeManager.theme.asset("/Icons/Minus__DefaultState.png")
            rotation: 0
            scale: 1

            onClicked: {
                root.itemInterface.set_VentilatorParametersInterface_peepPopup(VentilatorParametersInterface.peepPopup - VentilatorParametersInterface.peepStep)
            }
        }
        VentilatorUISet.Button_Plus_Default_State {
            id: fi_IncrementButton
            x: 496
            y: 102
            z: 5
            width: 90
            height: 90
            opacity: 1
            enabled: true
            visible: true
            clip: false
            icon_opacity: 1
            icon_source: GreenHouseThemeManager.theme.asset("/Icons/Plus__DefaultState.png")
            rotation: 0
            scale: 1

            onClicked: {
                root.itemInterface.set_VentilatorParametersInterface_peepPopup(VentilatorParametersInterface.peepPopup + VentilatorParametersInterface.peepStep)
            }
        }
        VentilatorUISet.Button_TextPrimary_Default_State {
            id: fi_Confirm
            x: 336
            y: 247
            z: 1
            width: 250
            height: 90
            opacity: 1
            enabled: true
            visible: true
            clip: false
            fi_Label_color: GreenHouseThemeManager.theme.color("buttonAltTextColor")
            fi_Label_horizontalAlignment: Text.AlignHCenter
            fi_Label_opacity: 1
            fi_Label_text: qsTr("Confirm")
            fi_Label_verticalAlignment: Text.AlignVCenter
            rotation: 0
            scale: 1

            onClicked: {
                root.itemInterface.call_VentilatorParametersInterface_adjustPeep(VentilatorParametersInterface.peepPopup)
                root.itemInterface.moveToNoPopup()
            }
        }
        VentilatorUISet.Button_TextSecondary_Default_State {
            id: fi_Cancel
            x: 40
            y: 247
            z: 2
            width: 250
            height: 90
            opacity: 1
            enabled: true
            visible: true
            clip: false
            fi_Label_color: GreenHouseThemeManager.theme.color("buttonAltTextColor")
            fi_Label_horizontalAlignment: Text.AlignHCenter
            fi_Label_opacity: 1
            fi_Label_text: qsTr("Cancel")
            fi_Label_verticalAlignment: Text.AlignVCenter
            rotation: 0
            scale: 1

            onClicked: {
                root.itemInterface.moveToNoPopup()
            }
        }
    }

    showAnimation: AdjustPEEPLayoutShowAnimation { target: root }
    hideAnimation: AdjustPEEPLayoutHideAnimation { target: root }
    initAction: AdjustPEEPLayoutInitAnimation { target: root }

    itemInterface: AdjustPEEPIFaceImpl {}
}
