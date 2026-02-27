package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TeleopStates extends OpMode {

    //TODO: DRIVETRAIN
    DrivetrainClass dt = new DrivetrainClass();

    // TODO: INTAKE
    Intake in = new Intake();

    // TODO: SORTER
    DcMotor sorterMotor;

    private static final double TICKS_PER_REV = 537.6 * (double)( 10 / 14);
    public static double INCREMENT = TICKS_PER_REV / 3;
    public double target = 0;

    // TODO: LEVER
    Lever lv = new Lever();

    // TODO: PITCH
    Servo pitchServo;

    // TODO: SHOOTER
    Shooter sh = new Shooter();

    // TODO: TURRET



    @Override
    public void init() {
        dt.initDrivetrain(hardwareMap);
        in.initIntake(hardwareMap);
        //sr.initSorter(hardwareMap);
        sh.initShooter(hardwareMap);

        sorterMotor = hardwareMap.get(DcMotor.class, "sorterMotor");
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sorterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lv.initLever(hardwareMap);

        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        pitchServo.setDirection(Servo.Direction.REVERSE);

        lv.leftLeverServo.setPosition(0);
        lv.rightLeverServo.setPosition(0);

        telemetry.addLine("Initialized");
    }

    @Override
    public void loop() {

        //TODO: DRIVETRAIN
        if (gamepad1.left_trigger > 0.75) {
            dt.driveMecanumSlower(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            dt.driveMecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        // TODO: INTAKE
        if (gamepad1.a) in.intakeOn();
        if (gamepad1.b) in.intakeOff();
        if (gamepad1.x) in.intakeReverse();

        // TODO: SORTER
        //sr.PIDFSorterLoop();

        /*
        if (gamepad1.aWasPressed()) {
            sr.setSorterTarget(1);
        }
         */

        if (gamepad2.aWasPressed()) {
            target += INCREMENT;
        }

        sorterMotor.setTargetPosition((int) target);
        sorterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorterMotor.setPower(0.8);
        if (sorterMotor.getCurrentPosition() == sorterMotor.getTargetPosition()) {
            sorterMotor.setPower(0);
        }

        //TODO: LEVER
        if (gamepad2.dpad_up) {
            lv.leverUp();
        } else {
            lv.leverDown();
        }

        // TODO: SHOOTER
        sh.PIDFShootingLoop();

        boolean shooterEnabled1 = gamepad2.left_trigger > 0.75;
        boolean shooterEnabled2 = gamepad2.right_trigger > 0.75;

        if (!shooterEnabled2 && !shooterEnabled1) {
            pitchServo.setPosition(0.75);
            sh.setCurTargetVelocity("0", 0);
        }
        else if (shooterEnabled2 && !shooterEnabled1) {
            pitchServo.setPosition(0.45);
            sh.setCurTargetVelocity("long", 0);
            sh.fShooting = 15;
        }
        else if (!shooterEnabled2 && shooterEnabled1) {
            pitchServo.setPosition(0.75);
            sh.setCurTargetVelocity("short", 0);
            sh.fShooting = 15.25;
        }
        else if (shooterEnabled2 && shooterEnabled1) {
            pitchServo.setPosition(0.75);
            sh.setCurTargetVelocity("0", 0);
        }

        // TODO: TURRET

    }
}
