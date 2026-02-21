package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lever {

    private Servo servo1;
    private Servo servo2;

    // Preset positions
    public static double REST_POS = 0.00;
    public static double SCORE_POS = 0.20;
    public static double PRELOAD_POS = 0.15;

    // Smooth motion
    private double targetPos = REST_POS;
    private double currentPos = REST_POS;
    private double step = 0.01;      // smoothness
    private boolean reversed = false;

    public void initLever(HardwareMap hwMap) {
        servo1 = hwMap.get(Servo.class, "leverServo1");
        servo2 = hwMap.get(Servo.class, "leverServo2");


        servo1.setPosition(REST_POS);
        servo2.setPosition(REST_POS);
        currentPos = REST_POS;
        targetPos = REST_POS;
    }


    public void leverUp() {
        setTarget(SCORE_POS);
    }

    public void leverDown() {
        setTarget(REST_POS);
    }

    public void preload() {
        setTarget(PRELOAD_POS);
    }

    public void custom(double pos) {
        setTarget(pos);
    }



    private void setTarget(double pos) {
        targetPos = clamp(pos);
    }

    public void update() {
        if (Math.abs(targetPos - currentPos) < step) {
            currentPos = targetPos;
        } else if (currentPos < targetPos) {
            currentPos += step;
        } else {
            currentPos -= step;
        }

        // Sync protection: always set both servos together
        servo1.setPosition(currentPos);
        servo2.setPosition(currentPos);
    }


    private double clamp(double p) {
        return Math.max(0.0, Math.min(1.0, p));
    }
}
