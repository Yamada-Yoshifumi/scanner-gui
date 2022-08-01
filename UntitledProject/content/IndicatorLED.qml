

/*
This is a UI file (.ui.qml) that is intended to be edited in Qt Design Studio only.
It is supposed to be strictly declarative and only uses a subset of QML. If you edit
this file manually, you might introduce QML code that is not supported by Qt Design Studio.
Check out https://doc.qt.io/qtcreator/creator-quick-ui-forms.html for details on .ui.qml files.
*/
import QtQuick 2.15
import QtQuick.Controls 2.15
import UntitledProject


Canvas {
    id: mycanvas

    antialiasing: true

    onPaint: squircle();

    function squircle(led_color = "green"){
        var ctx = getContext("2d");
        ctx.reset();

        ctx.fillStyle = '#343434';
        ctx.fillRect(0, 0, mycanvas.width, mycanvas.height);

        var x = mycanvas.width/2,
            y = mycanvas.height/2,
            // Radii of the white glow.
            innerRadius = mycanvas.width/50,
            outerRadius = mycanvas.width/20,
            // Radius of the entire circle.
            radius = mycanvas.width/20;

        var gradient = ctx.createRadialGradient(x, y, innerRadius, x, y, outerRadius);
        gradient.addColorStop(0, 'white');
        gradient.addColorStop(1, led_color);

        ctx.arc(x, y, radius, 0, 2 * Math.PI);

        ctx.fillStyle = gradient;
        ctx.fill();
    }
}

/*##^##
Designer {
    D{i:0;autoSize:true;formeditorZoom:0.66;height:480;width:640}
}
##^##*/
