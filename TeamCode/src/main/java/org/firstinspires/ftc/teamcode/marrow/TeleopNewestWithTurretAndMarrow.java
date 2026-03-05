package org.firstinspires.ftc.teamcode.marrow;
/*
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.seattlesolvers.solverslib.controller.PIDController;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

import com.skeletonarmy.marrow.zones.PolygonZone;
import com.skeletonarmy.marrow.zones.CircleZone;
import com.skeletonarmy.marrow.zones.CompositeZone;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.OpModeManager;

@Config
@TeleOp(name = "REAL TELEOP (with turret + Marrow zones + assists)")
public class TeleopNewestWithTurretAndMarrow extends OpMode {

    public double fShooting = 15;
    public double pShooting = 250;
    public double curTargetVelocity = 0;

    public static double A = 0.0;
    public static double B = 0.0;
    public static double C = 0.0;

    private static final double GOAL_X_INCHES = 144.0;
    private static final double GOAL_Y_INCHES = 72.0;

    private DcMotor sorterMotor;
    private PIDController sorterController;

    private double pSorting = 0.004;
    private double iSorting = 0.0;
    private double dSorting = 0.00027;

    public static double kSSorting = 0.034;

    private static final double TICKS_PER_REV = 537.6 * ((double) 10 / 14);
    public static double INCREMENT = TICKS_PER_REV / 6;
    private double target = 0;
    private int turns = 0;

    double lastTx = 0;
    Limelight3A limelight;

    final int READ_PERIOD = 1;

    DcMotor intakeMotor;
    Servo pitchServo;
    DcMotorEx shootingMotor;
    Servo leverServo1;
    Servo leverServo2;
    ColorSensor colorSensor;
    HuskyLens huskyLens;
    HuskyLens huskyLens2;
    Deadline rateLimit;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DistanceSensor distanceSensor2;

    ElapsedTime timer = new ElapsedTime();

    private PIDController controllerTurret;

    public static double pTurret = 0.03;
    public static double iTurret = 0.00;
    public static double dTurret = 0.0002;
    public static double kFFTurret = 0.02;

    IMU turretImuTurret;
    private double lastYawDegTurret = 0.0;
    private double unwrappedYawDegTurret = 0.0;
    private double yawOffsetDegTurret = 0.0;

    DcMotor rotationMotorTurret;

    public static int TURRET_ENCODER_HIGH_LIMIT_Turret = 1000;
    public static int TURRET_ENCODER_LOW_LIMIT_Turret  = -1000;

    private enum SorterState {
        IDLE,
        SEEKING,
        CONFIRMING
    }
    private SorterState sorterState = SorterState.IDLE;

    private ElapsedTime confirmTimer = new ElapsedTime();
    private final double CONFIRM_TIME_SEC = 0.1;
    private final double TARGET_RANGE_MIN = 1.4;
    private final double TARGET_RANGE_MAX = 2.0;
    private final double SEEK_POWER = 0.2;

    private PolygonZone launchZone;
    private CircleZone wallBufferZone;
    private CompositeZone dangerZone;

    private double robotXInches = 0.0;
    private double robotYInches = 0.0;

    private boolean enableAutoAim = true;
    private boolean endgameSlowMode = true;
    private boolean autoIntakeAssist = true;
    private boolean autoShootOnZoneEntry = true;
    private boolean autoParkEnabled = true;

    private boolean wasInLaunchZone = false;
    private ElapsedTime shootTimer = new ElapsedTime();
    private boolean shootingPulseActive = false;
    private static final double SHOOT_PULSE_TIME = 0.20;
    private static final double SHOOT_READY_TOLERANCE = 50.0;

    private static final double AUTO_INTAKE_RANGE_INCHES = 6.0;

    private static final double PARK_X_INCHES = 24.0;
    private static final double PARK_Y_INCHES = 24.0;
    private static final double AUTO_PARK_K = 0.02;
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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        sorterController = new PIDController(pSorting,iSorting,dSorting);
        sorterMotor = hardwareMap.get(DcMotor.class, "sorterMotor");
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sorterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        sorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        pitchServo = hardwareMap.get(Servo.class,"pitchServo");
        rotationMotorTurret = hardwareMap.get(DcMotor.class, "rotationMotor");
        shootingMotor = hardwareMap.get(DcMotorEx.class, "shootingMotor");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "distanceSensor2");
        huskyLens2 = hardwareMap.get(HuskyLens.class, "huskylens2");
        leverServo1 = hardwareMap.get(Servo.class,"leverServo1");
        leverServo2 = hardwareMap.get(Servo.class, "leverServo2");
        turretImuTurret = hardwareMap.get(IMU.class, "turretImu");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);
        telemetry.setMsTransmissionInterval(11);

        PIDFCoefficients pidfShooting =
                new PIDFCoefficients(pShooting, 0, 0, fShooting);
        shootingMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, pidfShooting
        );

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        turretImuTurret.initialize(new IMU.Parameters(orientationOnRobot));
        turretImuTurret.resetYaw();

        YawPitchRollAngles initAngles = turretImuTurret.getRobotYawPitchRollAngles();
        lastYawDegTurret = initAngles.getYaw(AngleUnit.DEGREES);
        unwrappedYawDegTurret = lastYawDegTurret;
        yawOffsetDegTurret = lastYawDegTurret;

        controllerTurret = new PIDController(pTurret, iTurret, dTurret);

        rotationMotorTurret.setDirection(DcMotorSimple.Direction.REVERSE);
        rotationMotorTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotorTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationMotorTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();
        leverServo1.setPosition(0);
        leverServo2.setPosition(0);

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

        sorterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shootingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        launchZone = new PolygonZone(
                new Point(0, 0),
                new Point(48, 0),
                new Point(48, 48),
                new Point(0, 48)
        );

        wallBufferZone = new CircleZone(new Point(0, 72), 12);
        dangerZone = new CompositeZone(launchZone, wallBufferZone);

        OpModeManager.getTelemetry().addData("OpMode", OpModeManager.getActiveOpModeName());
        OpModeManager.getTelemetry().update();

        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        timer.reset();
        pitchServo.setPosition(0.42);
        shootTimer.reset();
        shootingPulseActive = false;
        wasInLaunchZone = false;
    }

    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();

            double xMeters = botpose.getPosition().x;
            double yMeters = botpose.getPosition().y;

            robotXInches = xMeters * 39.3701;
            robotYInches = yMeters * 39.3701;

            lastTx = result.getTx();
        }

        double dxGoal = GOAL_X_INCHES - robotXInches;
        double dyGoal = GOAL_Y_INCHES - robotYInches;
        double distanceToGoal = Math.hypot(dxGoal, dyGoal);

        Point robotPoint = new Point(robotXInches, robotYInches);
        boolean inLaunchZone = launchZone.contains(robotPoint);
        boolean nearWall = wallBufferZone.contains(robotPoint);
        boolean inDanger = dangerZone.contains(robotPoint);

        pitchServo.setPosition(0.42);

        boolean autoParkCommand = gamepad1.y;
        boolean useSlowDrive = gamepad1.left_trigger > 0.75;

        if (autoParkEnabled && autoParkCommand) {
            double dx = PARK_X_INCHES - robotXInches;
            double dy = PARK_Y_INCHES - robotYInches;

            double forward = -dy * AUTO_PARK_K;
            double strafe  = dx * AUTO_PARK_K;
            double rotate  = 0.0;

            driveMecanumSlower(forward, strafe, rotate);
        } else {
            if (endgameSlowMode && timer.seconds() > 110.0) {
                useSlowDrive = true;
            }
            if (nearWall) {
                useSlowDrive = true;
            }

            if (useSlowDrive) {
                driveMecanumSlower(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            } else {
                driveMecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }
        }
        if (gamepad1.a) intakeMotor.setPower(1.0);
        if (gamepad1.b) intakeMotor.setPower(0);
        if (gamepad1.x) intakeMotor.setPower(-1.0);

        if (autoIntakeAssist) {
            double distInches = distanceSensor2.getDistance(DistanceUnit.INCH);
            if (distInches > 0 && distInches < AUTO_INTAKE_RANGE_INCHES) {
                intakeMotor.setPower(1.0);
            }
        }

        if (gamepad2.dpad_up) leverServo1.setPosition(0.2);
        else if (!shootingPulseActive) leverServo1.setPosition(0);
        if (gamepad2.dpad_up) leverServo2.setPosition(0.2);
        else if (!shootingPulseActive) leverServo2.setPosition(0);

        if (gamepad2.aWasReleased()) {
            turns += 1;
            target = (turns % 6) * INCREMENT;
        }
        if (gamepad2.yWasPressed()) {
            turns += 2;
            target = (turns % 6) * INCREMENT;
        }

        sorterController.setPID(pSorting, iSorting, dSorting);
        double currentPos = sorterMotor.getCurrentPosition();
        double error = target - currentPos;

        double pidOutput = sorterController.calculate(currentPos, target);
        double staticFF = kSSorting * Math.signum(error);
        double motorPowerSorter = pidOutput + staticFF;

        sorterMotor.setPower(motorPowerSorter);

        boolean shooterEnabled1 = gamepad2.left_trigger > 0.75;
        boolean shooterEnabled2 = gamepad2.right_trigger > 0.75;

        if (!shooterEnabled1 && !shooterEnabled2) {
            curTargetVelocity = 0;
        } else {
            curTargetVelocity = A * distanceToGoal * distanceToGoal +
                    B * distanceToGoal +
                    C;
        }

        PIDFCoefficients newPidf = new PIDFCoefficients(pShooting, 0, 0, fShooting);
        shootingMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPidf);
        shootingMotor.setVelocity(curTargetVelocity);

        if (autoShootOnZoneEntry && inLaunchZone && !wasInLaunchZone) {
            if (curTargetVelocity > 0 &&
                    Math.abs(shootingMotor.getVelocity() - curTargetVelocity) < SHOOT_READY_TOLERANCE) {
                leverServo1.setPosition(0.2);
                leverServo2.setPosition(0.2);
                shootTimer.reset();
                shootingPulseActive = true;
            }
        }

        if (shootingPulseActive && shootTimer.seconds() > SHOOT_PULSE_TIME) {
            leverServo1.setPosition(0.0);
            leverServo2.setPosition(0.0);
            shootingPulseActive = false;
        }

        wasInLaunchZone = inLaunchZone;

        controllerTurret.setPID(pTurret, iTurret, dTurret);
        double errorTurret = -lastTx;

        double pidOutputTurret = controllerTurret.calculate(lastTx, 0);
        double feedforward = Math.copySign(kFFTurret, errorTurret);
        double motorPowerTurret = pidOutputTurret + feedforward;

        motorPowerTurret = Math.max(-1.0, Math.min(1.0, motorPowerTurret));
        int turretEnc = rotationMotorTurret.getCurrentPosition();

        boolean atHighLimitTurret = turretEnc >= TURRET_ENCODER_HIGH_LIMIT_Turret;
        boolean atLowLimitTurret  = turretEnc <= TURRET_ENCODER_LOW_LIMIT_Turret;

        if ((atHighLimitTurret && motorPowerTurret > 0.0) ||
                (atLowLimitTurret  && motorPowerTurret < 0.0)) {
            motorPowerTurret = 0.0;
        }

        if (enableAutoAim) {
            rotationMotorTurret.setPower(motorPowerTurret);
        } else {
            rotationMotorTurret.setPower(0.0);
        }

        telemetry.addData("tx", lastTx);
        telemetry.addData("target", target);
        telemetry.addData("Sorter Error", error);
        telemetry.addData("ticks", rotationMotorTurret.getCurrentPosition());
        telemetry.addData("Match Time", "%.1f", timer.seconds());
        telemetry.addData("Endgame Slow", endgameSlowMode);
        telemetry.addData("Auto Aim", enableAutoAim);
        telemetry.addData("Auto Intake Assist", autoIntakeAssist);
        telemetry.addData("Auto Shoot on Zone Entry", autoShootOnZoneEntry);
        telemetry.addData("Auto Park Enabled", autoParkEnabled);
        telemetry.addData("Robot X", robotXInches);
        telemetry.addData("Robot Y", robotYInches);
        telemetry.addData("In Launch Zone", inLaunchZone);
        telemetry.addData("Near Wall", nearWall);
        telemetry.addData("In Danger Zone", inDanger);
        telemetry.addData("Targets Visible", result != null ? result.getFiducialResults().size() : 0);
        telemetry.update();
    }
}

 */
