package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Servo Zero")
public class ServoZero extends LinearOpMode {

    double pos1 = 0.5;
    double pos2 = 0.5;
    double pos3 = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        Servo leverServo1 = hardwareMap.get(Servo.class, "lever servo 1");
        Servo leverServo2 = hardwareMap.get(Servo.class, "lever servo 2");
        Servo pitchServo = hardwareMap.get(Servo.class, "pitch servo");

        waitForStart();

        // Start centered
        leverServo1.setPosition(pos1);
        leverServo2.setPosition(pos2);
        pitchServo.setPosition(pos3);

        while (opModeIsActive()) {

            // Move leverServo with dpad up and down
            if (gamepad1.dpad_up) pos1 += 0.01;
            if (gamepad1.dpad_down) pos1 -= 0.01;

            // Move leverServo2 with X and B
            if (gamepad1.x) pos2 += 0.01;
            if (gamepad1.b) pos2 -= 0.01;

            // Move pitchServo with Y and A
            if (gamepad1.y) pos3 += 0.01;
            if (gamepad1.a) pos3 -= 0.01;

            pos1 = Range.clip(pos1, 0, 1);
            pos2 = Range.clip(pos2, 0, 1);
            pos3 = Range.clip(pos3, 0, 1);

            // Put new positions
            leverServo1.setPosition(pos1);
            leverServo2.setPosition(pos2);
            pitchServo.setPosition(pos3);

            telemetry.addData("leverServo", pos1);
            telemetry.addData("leverServo2", pos2);
            telemetry.addData("pitchServo", pos3);
            telemetry.update();
        }
    }
}
