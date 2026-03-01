package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Color Sensor Test (Green vs Purple)")
public class ColorSensorTest extends OpMode {

    private ColorSensor colorSensor;

    private static final int GREEN_THRESHOLD = 120;
    private static final int PURPLE_THRESHOLD = 120;

    @Override
    public void init() {
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        telemetry.addLine("Color Sensor Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        String detected = classifyColor(r, g, b);

        telemetry.addData("Red", r);
        telemetry.addData("Green", g);
        telemetry.addData("Blue", b);
        telemetry.addData("Detected", detected);
        telemetry.update();
    }

    private String classifyColor(int r, int g, int b) {

        if (g > GREEN_THRESHOLD && g > r && g > b) {
            return "GREEN";
        }

        if ((r + b) > PURPLE_THRESHOLD && g < (r + b) / 2) {
            return "PURPLE";
        }

        return "UNKNOWN";
    }
}
