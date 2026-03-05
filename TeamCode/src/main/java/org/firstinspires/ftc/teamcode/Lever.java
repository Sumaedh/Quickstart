package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lever {

    Servo leftLeverServo;
    Servo rightLeverServo;

    double LEVER_UP_POS = 0.17;
    double LEVER_DOWN_POS = 0.03;

    public void initLever(HardwareMap hwMap) {
        leftLeverServo = hwMap.get(Servo.class, "leftLeverServo");

        leftLeverServo.setDirection(Servo.Direction.REVERSE);

        rightLeverServo = hwMap.get(Servo.class, "rightLeverServo");
    }

    public void leverUp() {
        leftLeverServo.setPosition(LEVER_UP_POS);
        rightLeverServo.setPosition(LEVER_UP_POS);
    }

    public void leverDown() {
        leftLeverServo.setPosition(LEVER_DOWN_POS);
        rightLeverServo.setPosition(LEVER_DOWN_POS);
    }

    public double getLeverPos() {
        return leftLeverServo.getPosition();
    }
}
