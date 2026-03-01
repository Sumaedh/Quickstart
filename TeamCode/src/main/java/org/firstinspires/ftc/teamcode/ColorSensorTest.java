package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Color Sensor Test")
public class ColorSensorTest extends OpMode {

    private ColorSensor colorSensor1;
    private ColorSensor colorSensor2;
    private ColorSensor colorSensor3;

    private static final int GREEN_THRESHOLD = 120;
    private static final int PURPLE_THRESHOLD = 120;

    @Override
    public void init() {
        colorSensor1 = hardwareMap.get(ColorSensor.class, "colorSensor1");
        colorSensor2 = hardwareMap.get(ColorSensor.class, "colorSensor2");
        colorSensor3 = hardwareMap.get(ColorSensor.class, "colorSensor3");

        telemetry.addLine("All 3 Color Sensors Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        readAndDisplaySensor("Sensor 1", colorSensor1);
        readAndDisplaySensor("Sensor 2", colorSensor2);
        readAndDisplaySensor("Sensor 3", colorSensor3);

        telemetry.update();
    }

    private void readAndDisplaySensor(String name, ColorSensor sensor) {
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        String detected = classifyColor(r, g, b);

        telemetry.addLine(name);
        telemetry.addData("  Red", r);
        telemetry.addData("  Green", g);
        telemetry.addData("  Blue", b);
        telemetry.addData("  Detected", detected);
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
