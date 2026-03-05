package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class ShooterPID extends OpMode {

    // 2000 long, f= 15
    // 1680 short, f = 15
    public DcMotorEx shootingMotor;

    Servo leftLeverServo;
    Servo rightLeverServo;
    Servo pitchServo;

    public static double curTargetVelocity = 0;

    public static double pitchPos = 0.75;
    public static double f = 0;
    public static double p = 0;
    public double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    public int stepIndex = 1;

    @Override
    public void init() {
        shootingMotor = hardwareMap.get(DcMotorEx.class, "shootingMotor");
        leftLeverServo = hardwareMap.get(Servo.class, "leftLeverServo");
        rightLeverServo = hardwareMap.get(Servo.class, "rightLeverServo");
        shootingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootingMotor.setDirection(DcMotor.Direction.FORWARD);
        leftLeverServo.setDirection(Servo.Direction.REVERSE);

        pitchServo = hardwareMap.get(Servo.class, "pitchServo");

        pitchServo.setDirection(Servo.Direction.REVERSE);
        pitchServo.setPosition(0);

        PIDFCoefficients pidf = new PIDFCoefficients(p, 0, 0, f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shootingMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        telemetry.addLine("Init complete");
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_up) {
            leftLeverServo.setPosition(0.2);
            rightLeverServo.setPosition(0.2);
        } else {
            leftLeverServo.setPosition(0.0);
            rightLeverServo.setPosition(0.0);
        }

        pitchServo.setPosition(pitchPos);
        PIDFCoefficients newPidf = new PIDFCoefficients(p, 0, 0, f);
        shootingMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPidf);

        shootingMotor.setVelocity(curTargetVelocity);

        double curVelocity = shootingMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;


        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", curVelocity);
        telemetry.addData("Error", error);
        telemetry.addData("P", p);
        telemetry.addData("F", f);
        telemetry.addData("Step Size", stepSizes[stepIndex]);
        telemetry.update();
    }
}
