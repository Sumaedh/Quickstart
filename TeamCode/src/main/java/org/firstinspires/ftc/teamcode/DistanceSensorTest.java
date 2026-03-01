package org.firstinspires.ftc.teamcode.marrow;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Distance Sensor Test (Ball Detect)")
public class DistanceSensorTest extends OpMode {

    private DistanceSensor distanceSensor;

    private static final double BALL_DETECT_THRESHOLD_INCHES = 6.0;

    @Override
    public void init() {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
    }

    @Override
    public void loop() {

        double distInches = distanceSensor.getDistance(DistanceUnit.INCH);

        boolean ballDetected = distInches > 0 && distInches < BALL_DETECT_THRESHOLD_INCHES;

        telemetry.addData("Distance (in)", distInches);
        telemetry.addData("Ball Detected", ballDetected);
        telemetry.update();
    }
}
