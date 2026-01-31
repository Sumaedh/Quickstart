package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "REAL TELEOP 3 blue ds")
public class TeleopNewest3DS extends OpMode {

    public double fShooting = 15;
    public double fShootingshort = 15.25;
    public double pShooting = 250;
    public double curTargetVelocity = 0;

    DistanceSensor distanceSensor2;

    private DcMotor sorterMotor;
    private PIDController sorterController;

    private double pSorting = 0.0029;
    private double iSorting = 0.0;
    private double dSorting = 0.00017;

    public static double kSSorting = 0.02;

    private static final double TICKS_PER_REV = 537.6;

    public static double INCREMENT = TICKS_PER_REV / 6;

    double target = 0;

    final int READ_PERIOD = 1;


    DcMotor intakeMotor;
    Servo pitchServo;
    DcMotorEx shootingMotor;
    Servo leverServo;
    ColorSensor colorSensor;
    HuskyLens huskyLens;
    HuskyLens huskyLens2;
    Deadline rateLimit;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    IMU turretImu;

    Turret turret = new Turret();
    ElapsedTime timer = new ElapsedTime();

    private enum SorterState {
        IDLE,           // Normal PID operation
        SEEKING,        // Moving toward sensor target
        CONFIRMING      // Ball detected, waiting 0.1s to confirm settled
    }
    private SorterState sorterState = SorterState.IDLE;

    private ElapsedTime confirmTimer = new ElapsedTime();
    private final double CONFIRM_TIME_SEC = 0.07;
    private final double TARGET_RANGE_MIN = 1.4;
    private final double TARGET_RANGE_MAX = 2.0;
    private final double SEEK_POWER = 0.1;
    public void driveMecanum(double left_y, double left_x, double right_x){
        double maxPower = Math.max(Math.abs(left_y) + Math.abs(left_x) + Math.abs(right_x), 1);
        frontLeft.setPower((-left_y + left_x + right_x) / maxPower);
        frontRight.setPower((-left_y - left_x - right_x) / maxPower);
        backLeft.setPower((-left_y - left_x + right_x) / maxPower);
        backRight.setPower((-left_y + left_x - right_x) / maxPower);
    }

    public void driveMecanumSlower(double left_y, double left_x, double right_x){
        double maxPower = Math.max(Math.abs(left_y) + Math.abs(left_x) + Math.abs(right_x), 1);
        frontLeft.setPower(((-left_y + left_x + right_x) / maxPower) / 3);
        frontRight.setPower(((-left_y - left_x - right_x) / maxPower) / 3);
        backLeft.setPower(((-left_y - left_x + right_x) / maxPower) / 3);
        backRight.setPower(((-left_y + left_x - right_x) / maxPower) / 3);
    }

    @Override
    public void init() {

        turret.initTurret(hardwareMap, telemetry);
        sorterController = new PIDController(pSorting,iSorting,dSorting);
        sorterMotor = hardwareMap.get(DcMotor.class, "sorterMotor");
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        pitchServo = hardwareMap.get(Servo.class,"pitchServo");
        shootingMotor = hardwareMap.get(DcMotorEx.class, "shootingMotor");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens2 = hardwareMap.get(HuskyLens.class, "huskylens2");
        sorterMotor = hardwareMap.get(DcMotor.class, "sorterMotor");
        leverServo = hardwareMap.get(Servo.class,"leverServo");
        turretImu = hardwareMap.get(IMU.class, "turretImu");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "distanceSensor2");

        PIDFCoefficients pidfShooting =
                new PIDFCoefficients(pShooting, 0, 0, fShooting);
        shootingMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, pidfShooting
        );

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        turretImu.initialize(new IMU.Parameters(orientationOnRobot));

        pitchServo.setDirection(Servo.Direction.REVERSE);
        pitchServo.setPosition(0);

        shootingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();
        leverServo.setPosition(0);

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        pitchServo.setDirection(Servo.Direction.REVERSE);
        pitchServo.setPosition(0);

        sorterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("status", "initialized");
    }

    public void start() {
        timer.reset();
        pitchServo.setPosition(0.42);
    }

    @Override
    public void loop() {

        // PITCH
        pitchServo.setPosition(0.42);

        turret.PIDFTurretLoop();

        // DRIVE
        if (gamepad1.left_trigger > 0.75) {
            driveMecanumSlower(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveMecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        // INTAKE
        if (gamepad1.a) intakeMotor.setPower(1.0);
        if (gamepad1.b) intakeMotor.setPower(0);
        if (gamepad1.x) intakeMotor.setPower(-1.0);


        // LEVER
        if (gamepad2.dpad_up) leverServo.setPosition(0.2);
        else leverServo.setPosition(0);


        // SORTER
        if (gamepad2.aWasPressed() && sorterState == SorterState.IDLE) {


            target += INCREMENT;
        }

        if (gamepad2.yWasPressed() && sorterState == SorterState.IDLE) {


            target += (INCREMENT*2);
        }

        // FSM Logic
        switch (sorterState) {
            case IDLE:
                // Check if we should start relocalizing
                if (gamepad2.xWasPressed()) {
                    sorterState = SorterState.SEEKING;
                }

                // Run normal PID
                sorterController.setPID(pSorting, iSorting, dSorting);
                double currentPos = sorterMotor.getCurrentPosition();
                double error = target - currentPos;
                double pidOutput = sorterController.calculate(currentPos, target);
                double staticFF = kSSorting * Math.signum(error);
                sorterMotor.setPower(pidOutput + staticFF);
                break;

            case SEEKING:
                double dist = distanceSensor2.getDistance(DistanceUnit.INCH);

                // Check if ball is in range
                if (dist >= TARGET_RANGE_MIN && dist <= TARGET_RANGE_MAX) {
                    // Ball detected! Start confirmation timer
                    sorterState = SorterState.CONFIRMING;
                    confirmTimer.reset();  // Start the 0.1s countdown
                    sorterMotor.setPower(SEEK_POWER * 0.5); // Optional: slow down while confirming
                } else {
                    // Keep seeking
                    sorterMotor.setPower(SEEK_POWER);
                }

                // Safety cancel: if driver hits X again, stop
                if (gamepad2.xWasPressed()) {
                    sorterState = SorterState.IDLE;
                    target = sorterMotor.getCurrentPosition(); // Lock current position
                }
                break;

            case CONFIRMING:
                double confirmDist = distanceSensor2.getDistance(DistanceUnit.INCH);

                // Verify ball is still there during confirmation window
                if (confirmDist < TARGET_RANGE_MIN || confirmDist > TARGET_RANGE_MAX) {
                    // Ball moved or was lost! Go back to seeking
                    sorterState = SorterState.SEEKING;
                } else if (confirmTimer.seconds() >= CONFIRM_TIME_SEC) {
                    // Success! 0.1s has passed with ball stable
                    sorterMotor.setPower(0);
                    target = sorterMotor.getCurrentPosition(); // Update target to stay here
                    sorterState = SorterState.IDLE;
                }
                // else: still confirming, keep motor at low power or hold position

                // Allow cancel during confirmation too
                if (gamepad2.xWasPressed()) {
                    sorterState = SorterState.IDLE;
                }
                break;
        }

        // SHOOTING
        boolean shooterEnabled1 = gamepad2.left_trigger > 0.75;
        boolean shooterEnabled2 = gamepad2.right_trigger > 0.75;

        if (!shooterEnabled2 && !shooterEnabled1) {
            curTargetVelocity = 0;
        }
        else if (shooterEnabled2 && !shooterEnabled1) {
            curTargetVelocity = 1680;
            fShooting = 15;
        }
        else if (!shooterEnabled2 && shooterEnabled1) {
            curTargetVelocity = 1420;
            fShooting = 15.25;
        }
        else if (shooterEnabled2 && shooterEnabled1) {
            curTargetVelocity = 0;
        }

        PIDFCoefficients newPidf = new PIDFCoefficients(pShooting, 0, 0, fShooting);
        shootingMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPidf);
        shootingMotor.setVelocity(curTargetVelocity);
    }
}
