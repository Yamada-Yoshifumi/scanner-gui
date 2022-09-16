

/*
This is a UI file (.ui.qml) that is intended to be edited in Qt Design Studio only.
It is supposed to be strictly declarative and only uses a subset of QML. If you edit
this file manually, you might introduce QML code that is not supported by Qt Design Studio.
Check out https://doc.qt.io/qtcreator/creator-quick-ui-forms.html for details on .ui.qml files.
*/
import QtQuick 2.1
import QtQuick.Controls 2.5


Canvas {
    id: mycanvas

    antialiasing: true

    onPaint: squircle();

    property string colour: "green"

    onColourChanged: {requestPaint();
                    }

    function squircle(){
        var ctx = getContext("2d");
        ctx.reset();

        var x = mycanvas.width/2 - 10,
            y = mycanvas.height/2 - 10,
            // Radii of the white glow.
            innerRadius = 0,
            outerRadius = mycanvas.width/40,
            // Radius of the entire circle.
            radius = mycanvas.width/40,
            width = radius * 2,
            height = width * 3;
        // This is available in all editors.
        /*
        var gradient = ctx.createRadialGradient(x - 3, y - 3, innerRadius, x - 3, y - 3, outerRadius);
        gradient.addColorStop(0, 'white');
        gradient.addColorStop(1, colour);

        ctx.arc(x, y, radius, 0, 2 * Math.PI);

        ctx.fillStyle = gradient;
        ctx.fill();
        */
        ctx.beginPath();
        ctx.moveTo(x + radius, y);
        ctx.lineTo(x + width - radius, y);
        ctx.quadraticCurveTo(x + width, y, x + width, y + radius);
        ctx.lineTo(x + width, y + height - radius);
        ctx.quadraticCurveTo(x + width, y + height, x + width - radius, y + height);
        ctx.lineTo(x + radius.bl, y + height);
        ctx.quadraticCurveTo(x, y + height, x, y + height - radius);
        ctx.lineTo(x, y + radius);
        ctx.quadraticCurveTo(x, y, x + radius, y);
        ctx.closePath();
        var gradient = ctx.createRadialGradient(x+radius, y+radius, innerRadius, x+radius, y+radius, outerRadius);
        gradient.addColorStop(0, 'white');
        gradient.addColorStop(1, colour);
        ctx.fillStyle = gradient;
        ctx.fill();

    }
}

/*##^##
Designer {
    D{i:0;autoSize:true;formeditorZoom:0.66;height:480;width:640}
}
##^##*/
