package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TankDrive")
public class TankDrive extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotor frontRightWheel = hardwareMap.get(DcMotor.class, "FrontRightWheel");
        DcMotor backRightWheel = hardwareMap.get(DcMotor.class, "BackRightWheel");
        DcMotor frontLeftWheel = hardwareMap.get(DcMotor.class, "FrontLeftWheel");
        DcMotor backLeftWheel = hardwareMap.get(DcMotor.class, "BackLeftWheel");

        //TODO Reverse left side so robot drives forward correctly
        frontLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            //TODO left stick controls left wheels right stick controls right wheels
            double leftPower  = -gamepad1.left_stick_y;
            double rightPower = -gamepad1.right_stick_y;

            frontLeftWheel.setPower(leftPower);
            backLeftWheel.setPower(leftPower);

            frontRightWheel.setPower(rightPower);
            backRightWheel.setPower(rightPower);

            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update();
        }
    }
}
