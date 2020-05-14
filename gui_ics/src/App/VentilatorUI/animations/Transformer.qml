import QtQuick 2.0

Item {
    id: root

    property double xTranslation: 0.0
    property double yTranslation: 0.0

    property double rotationXOrigin: 0.0
    property double rotationYOrigin: 0.0
    property double rotationZOrigin: 0.0

    property double rotationXAxis: 0.0
    property double rotationYAxis: 0.0
    property double rotationZAxis: 1.0
    property double rotationAngle: 0.0

    property double scaleXOrigin: 0.0
    property double scaleYOrigin: 0.0
    property double xScale: 1.0
    property double yScale: 1.0

    transform: [
    Translate {
        x: root.xTranslation
        y: root.yTranslation
    },
    Rotation {
        origin {
            x: root.rotationXOrigin
            y: root.rotationYOrigin
            z: root.rotationZOrigin
        }
        axis {
            x: root.rotationXAxis
            y: root.rotationYAxis
            z: root.rotationZAxis
        }
        angle: root.rotationAngle
    },
    Scale {
        origin {
            x: root.scaleXOrigin
            y: root.scaleYOrigin
        }
        xScale: root.xScale
        yScale: root.yScale
    }
    ]
}
