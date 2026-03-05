package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDController;

@TeleOp
public class TeleopStates extends OpMode {
    //TODO: DRIVETRAIN
    DrivetrainClass dt = new DrivetrainClass();

    // TODO: INTAKE
    Intake in = new Intake();

    // TODO: SORTER
    Sorter sr = new Sorter();

    // TODO: LEVER
    Lever lv = new Lever();

    // TODO: PITCH
    Servo pitchServo;

    // TODO: SHOOTER
    Shooter sh = new Shooter();

// TODO: TURRET




    @Override
    public void init() {
        sr.initSorter(hardwareMap);

        dt.initDrivetrain(hardwareMap);
        in.initIntake(hardwareMap);

        lv.initLever(hardwareMap);

        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        pitchServo.setDirection(Servo.Direction.REVERSE);

        lv.leftLeverServo.setPosition(0.03);
        lv.rightLeverServo.setPosition(0.03);

        // SHOOTER INIT
        sh.initShooter(hardwareMap);

        pitchServo.setPosition(0);

        telemetry.addLine("Initialized");
    }

    @Override
    public void loop() {

        // TODO: SORTER

        // SORTER
        if (gamepad2.aWasPressed()) {
            sr.turnSorter(1);
        }

        sr.PIDFSorterLoop();

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


        //TODO: LEVER
        if (gamepad2.dpad_up) {
            lv.leverUp();
        } else {
            lv.leverDown();
        }

        // TODO: SHOOTER
        boolean shooterEnabled1 = gamepad2.left_trigger > 0.75;
        boolean shooterEnabled2 = gamepad2.right_trigger > 0.75;

        if (!shooterEnabled2 && !shooterEnabled1) {
            sh.setCurTargetVelocity("0", 0);
        }
        else if (shooterEnabled2 && !shooterEnabled1) {
            sh.setCurTargetVelocity("long", 0);
        }
        else if (!shooterEnabled2 && shooterEnabled1) {
            sh.setCurTargetVelocity("short", 0);
        }
        else if (shooterEnabled2 && shooterEnabled1) {
            sh.setCurTargetVelocity("0", 0);
        }

        sh.PIDFShootingLoop();

        // TODO: TURRET

    }
}
