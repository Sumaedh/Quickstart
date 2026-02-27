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
    DcMotorEx shootingMotor;
    public double fShooting = 15;
    public double fShootingshort = 15.25;
    public double pShooting = 250;
    public double curTargetVelocity = 0;

// TODO: TURRET


    public void sorterMove() {
        sorterMotor.setTargetPosition((int) target);
        sorterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorterMotor.setPower(1);
    }

    @Override
    public void init() {
        dt.initDrivetrain(hardwareMap);
        in.initIntake(hardwareMap);
        //sr.initSorter(hardwareMap);

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

        // SHOOTER INIT
        shootingMotor = hardwareMap.get(DcMotorEx.class, "shootingMotor");

        PIDFCoefficients pidfShooting =
                new PIDFCoefficients(pShooting, 0, 0, fShooting);
        shootingMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, pidfShooting
        );

        shootingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pitchServo.setPosition(0.42);

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

        sorterMove();

        if (gamepad2.aWasPressed()) {
            target += INCREMENT;
        }

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
            pitchServo.setPosition(0.75);
            curTargetVelocity = 0;
        }
        else if (shooterEnabled2 && !shooterEnabled1) {
            pitchServo.setPosition(0.45);
            curTargetVelocity = 1680;
            fShooting = 15;
        }
        else if (!shooterEnabled2 && shooterEnabled1) {
            pitchServo.setPosition(0.75);
            curTargetVelocity = 1420;
            fShooting = 15.25;
        }
        else if (shooterEnabled2 && shooterEnabled1) {
            pitchServo.setPosition(0.75);
            curTargetVelocity = 0;
        }

        PIDFCoefficients newPidf = new PIDFCoefficients(pShooting, 0, 0, fShooting);
        shootingMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPidf);
        shootingMotor.setVelocity(curTargetVelocity);

        // TODO: TURRET

    }
}
