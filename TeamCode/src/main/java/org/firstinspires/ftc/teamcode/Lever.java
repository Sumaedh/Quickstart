package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lever {

    Servo leverServo;

    public void initLever(HardwareMap hwMap) {
        leverServo = hwMap.get(Servo.class, "leverServo");
    }

    public void leverUp() {
        leverServo.setPosition(0.2);
    }

    public void leverDown() {
        leverServo.setPosition(0);
    }
}
