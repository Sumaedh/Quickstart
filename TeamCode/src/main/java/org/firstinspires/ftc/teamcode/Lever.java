package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lever {

    private Servo servo1;
    private Servo servo2;

    // No preset positions
    private double targetPos = 0.0;
    private double currentPos = 0.0;

    // Smooth motion
    private double step = 0.01;

    public void initLever(HardwareMap hwMap) {
        servo1 = hwMap.get(Servo.class, "leverServo1");
        servo2 = hwMap.get(Servo.class, "leverServo2");

        servo1.setPosition(currentPos);
        servo2.setPosition(currentPos);
    }

    /** Set any target position externally */
    public void setPosition(double pos) {
        targetPos = clamp(pos);
    }

    /** Smoothly update servo positions */
    public void update() {
        if (Math.abs(targetPos - currentPos) < step) {
            currentPos = targetPos;
        } else if (currentPos < targetPos) {
            currentPos += step;
        } else {
            currentPos -= step;
        }

        servo1.setPosition(currentPos);
        servo2.setPosition(currentPos);
    }

    private double clamp(double p) {
        return Math.max(0.0, Math.min(1.0, p));
    }

    public void leverDown() {
    }

    public void leverUp() {
    }
}
