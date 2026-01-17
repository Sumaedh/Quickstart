package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Pitch {

    Servo pitchServo;

    public void initPitch(HardwareMap hwMap) {
        pitchServo = hwMap.get(Servo.class, "pitchServo");
        pitchServo.setDirection(Servo.Direction.REVERSE);
    }

    public void pitchUp() {
        pitchServo.setPosition(0);
    }

    public void pitchDown() {
        pitchServo.setPosition(0.42);
    }
}
