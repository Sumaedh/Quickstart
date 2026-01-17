package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Config
public class TeleopOld extends OpMode {

    public double fShooting = 15;
    public double fShootingshort = 15.25;
    public double pShooting = 250;
    public double curTargetVelocity = 0;

    public static double targetPower = 0;

    public static double LONG_SHOT = 0.87;
    public static double SHORT_SHOT = 0.795;

    public static double LONG_SERVO_POS = 0.3;
    public static double SHORT_SERVO_POS = 0.4;

    private DcMotor sorterMotor;
    private PIDController pid;

    final int READ_PERIOD = 1;


    private static final double TICKS_PER_REV = 537.6;

    private double[] positionDegrees = { 0, 60, 120, 180, 240, 300 };

    private int currentIndex = 0;
    private int targetTicks = 0;

    private double kP = 0.0029;
    private double kI = 0.0;
    private double kD = 0.00017;

    public static double kS = 0.02;

    private long lastAdvanceTime = 0;
    private static final long ADVANCE_DEBOUNCE_MS = 200;

    DcMotor intakeMotor;
    Servo pitchServo;
    DcMotor rotationMotor;
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

    boolean prevX = false;
    boolean sorting = false;

    public static int timeAmount = 110;
    ElapsedTime timer = new ElapsedTime();
    public static double leverPos = 0;
    public static double pitchPos = 0.42;

    public static int encoderAmount = 89;

    boolean prevPressed = false;

    // ------------------- FIXED SHOOTER SEQUENCE (timing tweaks) -------------------
    private final int SORTER_ADVANCE_STEPS = 2;
    private final double SEQ_SHOOTER_VELOCITY = 1680.0;
    private final double SHOOTER_TOLERANCE = 40.0;

    // lever hold time at up position (now 150 ms)
    private final long LEVER_UP_STAY_MS = 150; // 150 ms

    // adjusted per your latest request:
    private final long WAIT_AFTER_FIRST_LEVER_DOWN_MS = 1100;  // reduced by 250 ms
    private final long WAIT_AFTER_FIRST_SORTER_MS = 500;      // reduced by 350 ms

    // keep these as before
    private final long WAIT_AFTER_SECOND_LEVER_DOWN_MS = 1500; // unchanged (from prior)
    private final long WAIT_AFTER_SECOND_SORTER_MS = 850;     // unchanged (from prior)
    private final long WAIT_AFTER_THIRD_LEVER_DOWN_MS = 2000; // unchanged

    // sequence states
    private static final int SEQ_IDLE = 0;
    private static final int SEQ_WAIT_SHOOTER_BEFORE_FIRST = 1;
    private static final int SEQ_FIRST_LEVER_UP = 2;
    private static final int SEQ_WAIT_AFTER_FIRST_LEVER_DOWN = 3;
    private static final int SEQ_WAIT_AFTER_FIRST_SORTER = 4;
    private static final int SEQ_WAIT_SHOOTER_BEFORE_SECOND = 5;
    private static final int SEQ_SECOND_LEVER_UP = 6;
    private static final int SEQ_WAIT_AFTER_SECOND_LEVER_DOWN = 7;
    private static final int SEQ_WAIT_AFTER_SECOND_SORTER = 8;
    private static final int SEQ_WAIT_SHOOTER_BEFORE_THIRD = 9;
    private static final int SEQ_THIRD_LEVER_UP = 10;
    private static final int SEQ_WAIT_AFTER_THIRD_LEVER_DOWN = 11;
    private static final int SEQ_FINISH = 12;

    private boolean seqRunning = false;
    private int seqStage = SEQ_IDLE;
    private ElapsedTime seqTimer = new ElapsedTime();
    private double prevCurTargetVelocity = 0;
    private boolean lastSeqButtonState = false;

    private boolean stageJustEntered = false;
    private boolean leverReachedFlag = false;



    // ---------------------------------------------------------------------------

    public void driveMecanum(double left_y, double left_x, double right_x){
        double maxPower = Math.max(Math.abs(left_y) + Math.abs(left_x) + Math.abs(right_x), 1);
        frontLeft.setPower((-left_y + left_x + right_x) / maxPower);
        frontRight.setPower((-left_y + left_x - right_x) / maxPower);
        backLeft.setPower((-left_y - left_x + right_x) / maxPower);
        backRight.setPower((-left_y - left_x - right_x) / maxPower);
    }

    public void driveMecanumSlower(double left_y, double left_x, double right_x){
        double maxPower = Math.max(Math.abs(left_y) + Math.abs(left_x) + Math.abs(right_x), 1);
        frontLeft.setPower(((-left_y + left_x + right_x) / maxPower) / 3);
        frontRight.setPower(((-left_y + left_x - right_x) / maxPower) / 3);
        backLeft.setPower(((-left_y - left_x + right_x) / maxPower) / 3);
        backRight.setPower(((-left_y - left_x - right_x) / maxPower) / 3);
    }

    @Override
    public void init() {
        sorterMotor = hardwareMap.get(DcMotor.class, "sorterMotor");
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pid = new PIDController(kP, kI, kD);

        targetTicks = degreesToTicks(positionDegrees[currentIndex]);
        pid.setSetPoint(targetTicks);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        pitchServo = hardwareMap.get(Servo.class,"pitchServo");
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        shootingMotor = hardwareMap.get(DcMotorEx.class, "shootingMotor");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens2 = hardwareMap.get(HuskyLens.class, "huskylens2");
        sorterMotor = hardwareMap.get(DcMotor.class, "sorterMotor");
        leverServo = hardwareMap.get(Servo.class,"leverServo");
        turretImu = hardwareMap.get(IMU.class, "turretImu");

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

        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    private void advanceSorterBySteps(int steps) {
        currentIndex += steps;
        currentIndex = ((currentIndex % positionDegrees.length) + positionDegrees.length) % positionDegrees.length;
        targetTicks = degreesToTicks(positionDegrees[currentIndex]);
        pid.setSetPoint(targetTicks);
    }

    @Override
    public void loop() {

        pitchServo.setPosition(0.42);

        if (gamepad1.left_trigger > 0.75) {
            driveMecanumSlower(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveMecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        if (gamepad1.a) intakeMotor.setPower(1.0);
        if (gamepad1.b) intakeMotor.setPower(0);
        if (gamepad1.x) intakeMotor.setPower(-1.0);

        // ONLY allow manual lever control when sequence is NOT running
        if (!seqRunning) {
            if (gamepad2.dpad_up) leverServo.setPosition(0.2);
            else leverServo.setPosition(0);
        }

        long now = System.currentTimeMillis();
        if (gamepad2.a && (now - lastAdvanceTime) > ADVANCE_DEBOUNCE_MS) {
            currentIndex++;
            if (currentIndex >= positionDegrees.length) currentIndex = 0;
            targetTicks = degreesToTicks(positionDegrees[currentIndex]);
            pid.setSetPoint(targetTicks);
            lastAdvanceTime = now;
        }

        // ------------------- SHOOTER SEQUENCE (updated per request) -------------------
        boolean seqButton = gamepad2.y;
        if (seqButton && !lastSeqButtonState && !seqRunning) {
            seqRunning = true;
            seqStage = SEQ_WAIT_SHOOTER_BEFORE_FIRST;
            seqTimer.reset();
            prevCurTargetVelocity = curTargetVelocity;
            stageJustEntered = true;
            leverReachedFlag = false;
        }
        lastSeqButtonState = seqButton;

        if (seqRunning) {
            // force shooter target while running sequence
            curTargetVelocity = SEQ_SHOOTER_VELOCITY;

            switch (seqStage) {

                case SEQ_WAIT_SHOOTER_BEFORE_FIRST:
                    if (stageJustEntered) { seqTimer.reset(); stageJustEntered = false; }
                    // when shooter at/above (target - tol) -> **immediately** command lever up and switch stage
                    if (shootingMotor.getVelocity() >= (SEQ_SHOOTER_VELOCITY - SHOOTER_TOLERANCE)) {
                        // command lever up immediately (no extra wait)
                        leverServo.setPosition(0.2);
                        seqStage = SEQ_FIRST_LEVER_UP;
                        seqTimer.reset();
                        leverReachedFlag = false;
                        // do not set stageJustEntered — we already applied the entry action
                    }
                    break;

                case SEQ_FIRST_LEVER_UP:
                    // wait for lever to actually reach commanded position, then hold LEVER_UP_STAY_MS
                    if (!leverReachedFlag && leverServo.getPosition() >= 0.199) {
                        leverReachedFlag = true;
                        seqTimer.reset();
                    }
                    if (leverReachedFlag && seqTimer.milliseconds() >= LEVER_UP_STAY_MS) {
                        leverServo.setPosition(0.0); // down
                        seqStage = SEQ_WAIT_AFTER_FIRST_LEVER_DOWN;
                        seqTimer.reset();
                        stageJustEntered = true;
                        leverReachedFlag = false;
                    }
                    break;

                case SEQ_WAIT_AFTER_FIRST_LEVER_DOWN:
                    if (stageJustEntered) { seqTimer.reset(); stageJustEntered = false; }
                    if (seqTimer.milliseconds() >= WAIT_AFTER_FIRST_LEVER_DOWN_MS) {
                        advanceSorterBySteps(SORTER_ADVANCE_STEPS);
                        seqStage = SEQ_WAIT_AFTER_FIRST_SORTER;
                        seqTimer.reset();
                        stageJustEntered = true;
                    }
                    break;

                case SEQ_WAIT_AFTER_FIRST_SORTER:
                    if (stageJustEntered) { seqTimer.reset(); stageJustEntered = false; }
                    if (seqTimer.milliseconds() >= WAIT_AFTER_FIRST_SORTER_MS) {
                        seqStage = SEQ_WAIT_SHOOTER_BEFORE_SECOND;
                        seqTimer.reset();
                        stageJustEntered = true;
                        leverReachedFlag = false;
                    }
                    break;

                case SEQ_WAIT_SHOOTER_BEFORE_SECOND:
                    if (stageJustEntered) { seqTimer.reset(); stageJustEntered = false; }
                    // immediate lever up when shooter ready
                    if (shootingMotor.getVelocity() >= (SEQ_SHOOTER_VELOCITY - SHOOTER_TOLERANCE)) {
                        leverServo.setPosition(0.2);
                        seqStage = SEQ_SECOND_LEVER_UP;
                        seqTimer.reset();
                        leverReachedFlag = false;
                    }
                    break;

                case SEQ_SECOND_LEVER_UP:
                    if (!leverReachedFlag && leverServo.getPosition() >= 0.199) {
                        leverReachedFlag = true;
                        seqTimer.reset();
                    }
                    if (leverReachedFlag && seqTimer.milliseconds() >= LEVER_UP_STAY_MS) {
                        leverServo.setPosition(0.0);
                        seqStage = SEQ_WAIT_AFTER_SECOND_LEVER_DOWN;
                        seqTimer.reset();
                        stageJustEntered = true;
                        leverReachedFlag = false;
                    }
                    break;

                case SEQ_WAIT_AFTER_SECOND_LEVER_DOWN:
                    if (stageJustEntered) { seqTimer.reset(); stageJustEntered = false; }
                    if (seqTimer.milliseconds() >= WAIT_AFTER_SECOND_LEVER_DOWN_MS) {
                        advanceSorterBySteps(SORTER_ADVANCE_STEPS);
                        seqStage = SEQ_WAIT_AFTER_SECOND_SORTER;
                        seqTimer.reset();
                        stageJustEntered = true;
                    }
                    break;

                case SEQ_WAIT_AFTER_SECOND_SORTER:
                    if (stageJustEntered) { seqTimer.reset(); stageJustEntered = false; }
                    if (seqTimer.milliseconds() >= WAIT_AFTER_SECOND_SORTER_MS) {
                        seqStage = SEQ_WAIT_SHOOTER_BEFORE_THIRD;
                        seqTimer.reset();
                        stageJustEntered = true;
                        leverReachedFlag = false;
                    }
                    break;

                case SEQ_WAIT_SHOOTER_BEFORE_THIRD:
                    if (stageJustEntered) { seqTimer.reset(); stageJustEntered = false; }
                    // immediate lever up when shooter ready
                    if (shootingMotor.getVelocity() >= (SEQ_SHOOTER_VELOCITY - SHOOTER_TOLERANCE)) {
                        leverServo.setPosition(0.2);
                        seqStage = SEQ_THIRD_LEVER_UP;
                        seqTimer.reset();
                        leverReachedFlag = false;
                    }
                    break;

                case SEQ_THIRD_LEVER_UP:
                    if (!leverReachedFlag && leverServo.getPosition() >= 0.199) {
                        leverReachedFlag = true;
                        seqTimer.reset();
                    }
                    if (leverReachedFlag && seqTimer.milliseconds() >= LEVER_UP_STAY_MS) {
                        leverServo.setPosition(0.0); // down
                        seqStage = SEQ_WAIT_AFTER_THIRD_LEVER_DOWN;
                        seqTimer.reset();
                        stageJustEntered = true;
                        leverReachedFlag = false;
                    }
                    break;

                case SEQ_WAIT_AFTER_THIRD_LEVER_DOWN:
                    if (stageJustEntered) { seqTimer.reset(); stageJustEntered = false; }
                    if (seqTimer.milliseconds() >= WAIT_AFTER_THIRD_LEVER_DOWN_MS) {
                        seqStage = SEQ_FINISH;
                        seqTimer.reset();
                        stageJustEntered = true;
                    }
                    break;

                case SEQ_FINISH:
                    seqRunning = false;
                    seqStage = SEQ_IDLE;
                    curTargetVelocity = prevCurTargetVelocity;
                    seqTimer.reset();
                    stageJustEntered = false;
                    leverReachedFlag = false;
                    break;
            }
        }
        // -------------------------------------------------------------------

        double currentPos = sorterMotor.getCurrentPosition();
        double power = pid.calculate(currentPos);
        double error = targetTicks - currentPos;
        double staticFF = kS * Math.signum(error);
        power += staticFF;
        if (Math.abs(targetTicks - currentPos) < 10) power = 0;
        power = Math.max(-0.6, Math.min(0.6, power));
        sorterMotor.setPower(power);

        telemetry.addData("Cur ticks", currentPos);
        telemetry.addData("Tgt ticks", targetTicks);
        telemetry.addData("Index", currentIndex);
        telemetry.addData("Error", targetTicks - currentPos);
        telemetry.addData("Power", power);
        telemetry.addData("seqRunning", seqRunning);
        telemetry.addData("seqStage", seqStage);
        telemetry.addData("ShooterVel", shootingMotor.getVelocity());
        telemetry.addData("LeverPosCmd", leverServo.getPosition());
        telemetry.update();

        if (!seqRunning) {
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
        }

        PIDFCoefficients newPidf = new PIDFCoefficients(pShooting, 0, 0, fShooting);
        shootingMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPidf);
        shootingMotor.setVelocity(curTargetVelocity);
    }

    @Override
    public void stop() {
        sorterMotor.setPower(0);
    }

    private int degreesToTicks(double deg) {
        return (int)((deg / 360.0) * TICKS_PER_REV);
    }
}
