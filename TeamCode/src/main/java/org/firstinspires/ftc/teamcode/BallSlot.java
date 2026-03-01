package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class BallSlot {

    private DistanceSensor distanceSensor;
    private ColorSensor colorSensor;

    private boolean hasBall = false;
    private BallColor detectedColor = BallColor.UNKNOWN;

    private static final double BALL_DETECTION_DISTANCE_CM = 4.0;
    private static final int PURPLE_THRESHOLD = 200;
    private static final int GREEN_THRESHOLD = 200;

    private boolean acceptGreen = true;
    private boolean acceptPurple = true;
    private boolean acceptUnknown = false;

    private Boolean forcedDecision = null;

    public enum BallColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }

    public BallSlot(DistanceSensor distanceSensor, ColorSensor colorSensor) {
        this.distanceSensor = distanceSensor;
        this.colorSensor = colorSensor;
    }

    public void initSlot(HardwareMap hwMap, String distanceName, String colorName) {
        distanceSensor = hwMap.get(DistanceSensor.class, distanceName);
        colorSensor = hwMap.get(ColorSensor.class, colorName);
    }

    public void detectBall() {
        double dist = distanceSensor.getDistance(DistanceUnit.CM);
        hasBall = dist < BALL_DETECTION_DISTANCE_CM;
    }


    public void detectColor() {
        if (!hasBall) {
            detectedColor = BallColor.UNKNOWN;
            return;
        }

        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        if (g > GREEN_THRESHOLD && g > r && g > b) {
            detectedColor = BallColor.GREEN;
        }
        else if (((r + b) / 2) > PURPLE_THRESHOLD && (r + b) > g) {
            detectedColor = BallColor.PURPLE;
        }
        else {
            detectedColor = BallColor.UNKNOWN;
        }

    }


    public boolean shouldIntake() {

        if (forcedDecision != null) {
            return forcedDecision;
        }

        detectBall();
        detectColor();

        if (!hasBall) return true;

        switch (detectedColor) {
            case GREEN:
                return acceptGreen;

            case PURPLE:
                return acceptPurple;

            case UNKNOWN:
            default:
                return acceptUnknown;
        }
    }


    public void setAcceptGreen(boolean accept) {
        this.acceptGreen = accept;
    }

    public void setAcceptPurple(boolean accept) {
        this.acceptPurple = accept;
    }

    public void setAcceptUnknown(boolean accept) {
        this.acceptUnknown = accept;
    }


    public void forceIntake(boolean intake) {
        this.forcedDecision = intake;
    }

    public void clearOverride() {
        this.forcedDecision = null;
    }


    public boolean HasBall() {
        return hasBall;
    }

    public BallColor getDetectedColor() {
        return detectedColor;
    }
}
