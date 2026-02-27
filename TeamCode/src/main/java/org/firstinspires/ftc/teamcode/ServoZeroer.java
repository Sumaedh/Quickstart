package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class ServoZeroer extends OpMode {

    public static double leverPos = 0;

    public static double pitchPos = 0;

    Servo leftLeverServo;

    Servo rightLeverServo;

    Servo pitchServo;

    @Override
    public void init() {
        leftLeverServo = hardwareMap.get(Servo.class, "leftLeverServo");

        leftLeverServo.setDirection(Servo.Direction.REVERSE);

        rightLeverServo = hardwareMap.get(Servo.class, "rightLeverServo");

        pitchServo = hardwareMap.get(Servo.class, "pitchServo");

        pitchServo.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {

        leftLeverServo.setPosition(leverPos);
        rightLeverServo.setPosition(leverPos);
        pitchServo.setPosition(pitchPos);
    }
}
