import QtQuick 2.0
import QtGraphicalEffects 1.0
import com.ics.waveformchart 1.0

//PREVIEW IMAGE qrc:/MedicalPlottingComponentSet/PreviewAssets/icon.png
Item {
    id: root

    width: 256
    height: 128

    signal nodeSelected(int index)

    //MODEL
    property alias dataModel: plot.dataModel
    //STRING value
    property alias valueRoleName: plot.valueRoleName

    //STRING timeStamp
    property alias timeStampRoleName: plot.timeStampRoleName

    //INT 256
    property alias valueCeiling: plot.valueCeiling
    //INT 0
    property alias plotStartIndex: plot.plotStartIndex
    //INT 24
    property alias plotPointCount: plot.plotPointCount
    //INT 0
    property alias plotValueOffset: plot.plotValueOffset
    property int markedNodeIndex: -1
    //REAL 50.0
    property alias markerAreaSize: plot.markerAreaSize

    //COLOR plotColor
    property alias plotColor: plot.plotColor
    //REAL 2.0
    property alias plotLineWidth: plot.plotLineWidth
    //ASSET PLACEHOLDER
    property alias nodeMarkerSource: plot.nodeMarkerSource

    //REAL 50.0
    property alias headerHeight: plot.headerHeight

    //COLOR plotColor
    property alias headerTextColor: plot.headerTextColor

    //FONT textFont
    property alias headerTextFont: plot.headerTextFont

    //INT 15
    property alias headerBottomOffset: plot.headerBottomOffset

    //BOOL FALSE
    property alias useTimer: plot.useTimer

    //INT 5000
    property alias timerInterval: plot.timerInterval

    //BOOL TRUE
    property alias drawLineToEnd: plot.drawLineToEnd

    //INT 0
    property alias plotLeftMargin: plot.plotLeftMargin

    //INT 0
    property alias plotRightMargin: plot.plotRightMargin

    property bool autoSelect: true

    property int glowRadius: 12
    property real glowOpacity: 0.5

    Item {
        id: plotSrc

        anchors.fill: parent

        DropShadow {
            anchors.fill: plot

            samples: 8
            radius: root.glowRadius
            opacity: root.glowOpacity
            color: plot.plotColor
            source: plot
        }

        TrendChartWidget {
            id: plot

            anchors.fill: parent
        }
    }

    ShaderEffectSource {
        id: shaderSrc

        sourceItem: plotSrc
        hideSource: true
    }

    property real leftFadeStart: 0.0477
    Rectangle {
        id: mask

        width: parent.height
        height: parent.width
        anchors.centerIn: parent

        gradient: Gradient {
            GradientStop {
                position: 0.0
                color: "transparent"
            }
            GradientStop {
                position: root.leftFadeStart
                color: "#ffffff"
            }
        }
    }

    ShaderEffectSource {
        id: maskSrc

        sourceItem: mask
        hideSource: true
    }

    ShaderEffect {
        property var src: shaderSrc
        property var mask: maskSrc

        anchors.fill: parent
        vertexShader: "
        uniform highp mat4 qt_Matrix;
        attribute highp vec4 qt_Vertex;
        attribute highp vec2 qt_MultiTexCoord0;

        varying highp vec2 coord;

        void main() {
            coord = qt_MultiTexCoord0;
            gl_Position = qt_Matrix * qt_Vertex;
        }"
        fragmentShader: "
            varying highp vec2 coord;
            uniform sampler2D src;
            uniform sampler2D mask;
            void main() {
                lowp vec4 tex = texture2D(src, coord);
                lowp vec4 maskSrc = texture2D(mask, coord.yx);
                gl_FragColor = tex * maskSrc.a;
            }"
    }

    MouseArea {
        anchors.fill: parent

        onClicked: {
            var index = plot.nodeIndexAt(mouse.x, mouse.y)
            if (root.autoSelect)
                plot.markedNodeIndex = index
            root.nodeSelected(index)
        }
    }

    onMarkedNodeIndexChanged: plot.markedNodeIndex = markedNodeIndex
}
