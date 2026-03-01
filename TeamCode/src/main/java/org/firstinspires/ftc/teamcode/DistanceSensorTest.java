package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(name = "Distance Sensor Test")
public class DistanceSensorTest extends OpMode {

    private DistanceSensor distanceSensor1;
    private DistanceSensor distanceSensor2;
    private DistanceSensor distanceSensor3;

    public static double BALL_DETECT_THRESHOLD_INCHES = 1.0;

    private FtcDashboard dashboard;

    @Override
    public void init() {
        distanceSensor1 = hardwareMap.get(DistanceSensor.class, "distanceSensor1");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "distanceSensor2");
        distanceSensor3 = hardwareMap.get(DistanceSensor.class, "distanceSensor3");

        dashboard = FtcDashboard.getInstance();

        telemetry.addLine("All 3 Distance Sensors Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        sendSensorData("Sensor 1", distanceSensor1);
        sendSensorData("Sensor 2", distanceSensor2);
        sendSensorData("Sensor 3", distanceSensor3);

        telemetry.update();
        dashboard.getTelemetry().update();
    }

    private void sendSensorData(String name, DistanceSensor sensor) {
        double distInches = sensor.getDistance(DistanceUnit.INCH);
        boolean ballDetected = distInches > 0 && distInches < BALL_DETECT_THRESHOLD_INCHES;

        telemetry.addLine(name);
        telemetry.addData("  Distance (in)", distInches);
        telemetry.addData("  Ball Detected", ballDetected);

        dashboard.getTelemetry().addLine(name);
        dashboard.getTelemetry().addData("  Distance (in)", distInches);
        dashboard.getTelemetry().addData("  Ball Detected", ballDetected);
    }
}
