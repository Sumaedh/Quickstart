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

    public static double BALL_DETECTION_DISTANCE_INCH = 1.0;
    public static int PURPLE_THRESHOLD = 120;
    public static int GREEN_THRESHOLD = 120;

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

    public BallSlot(HardwareMap hwMap, String distanceName, String colorName) {
        initSlot(hwMap, distanceName, colorName);
    }

    public void initSlot(HardwareMap hwMap, String distanceName, String colorName) {
        this.distanceSensor = hwMap.get(DistanceSensor.class, distanceName);
        this.colorSensor = hwMap.get(ColorSensor.class, colorName);
    }

    public void detectBall() {
        double dist = distanceSensor.getDistance(DistanceUnit.INCH);
        hasBall = dist > 0 && dist < BALL_DETECTION_DISTANCE_INCH;
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
        } else if (((r + b) / 2) > PURPLE_THRESHOLD && (r + b) > g) {
            detectedColor = BallColor.PURPLE;
        } else {
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

    public void setAcceptGreen(boolean accept) { this.acceptGreen = accept; }
    public void setAcceptPurple(boolean accept) { this.acceptPurple = accept; }
    public void setAcceptUnknown(boolean accept) { this.acceptUnknown = accept; }

    public void forceIntake(boolean intake) { this.forcedDecision = intake; }
    public void clearOverride() { this.forcedDecision = null; }

    public boolean hasBall() { return hasBall; }
    public BallColor getDetectedColor() { return detectedColor; }
}
