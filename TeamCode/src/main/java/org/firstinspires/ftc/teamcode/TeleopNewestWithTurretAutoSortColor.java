package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "REAL TELEOP AutoSort 2Dist1Col")
public class TeleopNewestWithTurretAutoSortColor extends OpMode {

    public double fShooting = 15;
    public double fShootingshort = 15.25;
    public double pShooting = 250;
    public double curTargetVelocity = 0;

    private DcMotor sorterMotor;
    private PIDController sorterController;
    private double pSorting = 0.007;
    private double iSorting = 0.0;
    private double dSorting = 0.00033;
    public static double kSSorting = 0.02;
    private static final double TICKS_PER_REV = 537.6 * ((double) 10 / 14);
    public static double INCREMENT = TICKS_PER_REV / 6;
    double target = 0;
    final int READ_PERIOD = 1;
    private ElapsedTime boostTimer = new ElapsedTime();
    private boolean boostActive = false;

    DcMotor intakeMotor;
    Servo pitchServo;
    DcMotorEx shootingMotor;
    Servo leverServo;
    ColorSensor colorSensor;
    HuskyLens huskyLens;
    HuskyLens huskyLens2;
    Deadline rateLimit;
    DcMotor frontLeft, frontRight, backLeft, backRight;
    ElapsedTime timer = new ElapsedTime();

    // Turret
    private PIDController controllerTurret;
    public static double pTurret = 0.03, iTurret = 0.001, dTurret = 0.0004, kFFTurret = 0.045, targetAngleTurret = 0;
    IMU turretImuTurret;
    private double lastYawDegTurret = 0.0, unwrappedYawDegTurret = 0.0, yawOffsetDegTurret = 0.0;
    DcMotor rotationMotorTurret;
    public static int TURRET_ENCODER_HIGH_LIMIT_Turret = 1000, TURRET_ENCODER_LOW_LIMIT_Turret = -1000;
    private boolean wrapOverrideActiveTurret = false, stuckAtHighLimitTurret = false;

    // AUTO-SORT: 2 Distance + 1 Color
    DistanceSensor distanceSensor1; // Intake
    DistanceSensor distanceSensor2; // Left compartment
    ColorSensor colorSensor3;       // Right compartment (replaces distanceSensor3)

    boolean prevBallAtIntake = false;
    final double BALL_DIST_THRESHOLD_IN = 1.05;
    final int COLOR_THRESHOLD = 300; // RGB sum threshold for ball detection

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
        huskyLens2 = hardwareMap.get(HuskyLens.class, "huskylens2");
        leverServo = hardwareMap.get(Servo.class,"leverServo");
        turretImuTurret = hardwareMap.get(IMU.class, "turretImu");

        // AUTO-SORT: Hardware map sensors
        distanceSensor1 = hardwareMap.get(DistanceSensor.class, "distanceSensor1");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "distanceSensor2");
        colorSensor3 = hardwareMap.get(ColorSensor.class, "colorSensor3"); // Right compartment

        PIDFCoefficients pidfShooting = new PIDFCoefficients(pShooting, 0, 0, fShooting);
        shootingMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfShooting);

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
        rotationMotorTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationMotorTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        frontLeft.setPower(0); frontRight.setPower(0); backLeft.setPower(0); backRight.setPower(0);

        pitchServo.setDirection(Servo.Direction.REVERSE);
        pitchServo.setPosition(0);
        shootingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    public void start() {
        timer.reset();
        pitchServo.setPosition(0.42);
    }

    // Helper for color sensor ball detection
    private boolean isBallDetected(ColorSensor sensor) {
        return (sensor.red() + sensor.green() + sensor.blue()) > COLOR_THRESHOLD;
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

        if (gamepad2.dpad_up) leverServo.setPosition(0.2);
        else leverServo.setPosition(0);

        // Driver control
        if (gamepad2.aWasPressed()) {
            target += INCREMENT;
            boostActive = true;
            boostTimer.reset();
        }
        if (gamepad2.yWasPressed()) {
            target += (INCREMENT * 2);
            boostActive = true;
            boostTimer.reset();
        }

        // AUTO-SORT LOGIC (2 Dist + 1 Color)
        int currentSlot = (int)Math.round(target / INCREMENT);
        currentSlot = ((currentSlot % 6) + 6) % 6;

        if ((int)Math.round(target / INCREMENT) % 2 == 1 && !boostActive) {
            double dist1 = distanceSensor1.getDistance(DistanceUnit.INCH);
            double dist2 = distanceSensor2.getDistance(DistanceUnit.INCH);
            boolean ballAtIntake = dist1 < BALL_DIST_THRESHOLD_IN;
            boolean ballAtLeft = dist2 < BALL_DIST_THRESHOLD_IN;
            boolean ballAtRight = isBallDetected(colorSensor3); // Color sensor check

            if (ballAtIntake && !prevBallAtIntake && currentSlot == 0) {
                if (!ballAtLeft && ballAtRight) {
                    target += 2 * INCREMENT; // To Left (Pos 2)
                    boostActive = true;
                    boostTimer.reset();
                } else if (ballAtLeft && !ballAtRight) {
                    target -= 4 * INCREMENT; // To Right (Pos 4)
                    boostActive = true;
                    boostTimer.reset();
                } else if (!ballAtLeft && !ballAtRight) {
                    target += 2 * INCREMENT; // Default Left
                    boostActive = true;
                    boostTimer.reset();
                } else if (ballAtLeft && ballAtRight) {
                    target += INCREMENT; // To odd (Pos 1), lock
                    boostActive = true;
                    boostTimer.reset();
                }
            }
        }

        if (currentSlot == 0) {
            prevBallAtIntake = (distanceSensor1.getDistance(DistanceUnit.INCH) < BALL_DIST_THRESHOLD_IN);
        } else {
            prevBallAtIntake = false;
        }

        // Sorter PID
        sorterController.setPID(pSorting, iSorting, dSorting);
        double currentPos = sorterMotor.getCurrentPosition();
        double error = target - currentPos;
        double motorPowerSorter;

        if (boostActive && boostTimer.seconds() < 0.1) {
            motorPowerSorter = Math.signum(error);
        } else {
            boostActive = false;
            double pidOutput = sorterController.calculate(currentPos, target);
            double staticFF = kSSorting * Math.signum(error);
            motorPowerSorter = pidOutput + staticFF;
        }
        sorterMotor.setPower(motorPowerSorter);

        // Shooting
        boolean shooterEnabled1 = gamepad2.left_trigger > 0.75;
        boolean shooterEnabled2 = gamepad2.right_trigger > 0.75;
        if (!shooterEnabled2 && !shooterEnabled1) curTargetVelocity = 0;
        else if (shooterEnabled2 && !shooterEnabled1) { curTargetVelocity = 1680; fShooting = 15; }
        else if (!shooterEnabled2 && shooterEnabled1) { curTargetVelocity = 1420; fShooting = 15.25; }
        else if (shooterEnabled2 && shooterEnabled1) curTargetVelocity = 0;

        PIDFCoefficients newPidf = new PIDFCoefficients(pShooting, 0, 0, fShooting);
        shootingMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPidf);
        shootingMotor.setVelocity(curTargetVelocity);

        // Turret
        controllerTurret.setPID(pTurret, iTurret, dTurret);
        YawPitchRollAngles orientation = turretImuTurret.getRobotYawPitchRollAngles();
        double currentYawDeg = orientation.getYaw(AngleUnit.DEGREES);
        double delta = currentYawDeg - lastYawDegTurret;
        if (delta > 180.0) delta -= 360.0;
        else if (delta < -180.0) delta += 360.0;
        unwrappedYawDegTurret += delta;
        lastYawDegTurret = currentYawDeg;
        double currentAngleDegTurret = unwrappedYawDegTurret - yawOffsetDegTurret;
        double currentTargetDegTurret = (targetAngleTurret > 180.0) ? targetAngleTurret - 360.0 : targetAngleTurret;
        double errorTurret = currentTargetDegTurret - currentAngleDegTurret;
        double pidOutputTurret = controllerTurret.calculate(currentAngleDegTurret, currentTargetDegTurret);
        double feedforwardTurret = Math.copySign(kFFTurret, errorTurret);
        double motorPowerTurret = pidOutputTurret + feedforwardTurret;
        motorPowerTurret = Math.max(-1.0, Math.min(1.0, motorPowerTurret));
        int turretEnc = rotationMotorTurret.getCurrentPosition();
        boolean atHighLimitTurret = turretEnc >= TURRET_ENCODER_HIGH_LIMIT_Turret;
        boolean atLowLimitTurret  = turretEnc <= TURRET_ENCODER_LOW_LIMIT_Turret;
        if ((atHighLimitTurret && motorPowerTurret > 0.0) || (atLowLimitTurret  && motorPowerTurret < 0.0)) motorPowerTurret = 0.0;
        rotationMotorTurret.setPower(motorPowerTurret);

        telemetry.addData("Current Slot", currentSlot);
        telemetry.addData("BallAtIntake", prevBallAtIntake);
        telemetry.addData("BallAtRight(Col)", isBallDetected(colorSensor3));
        telemetry.addData("Turret Angle", currentAngleDegTurret);
        telemetry.update();
    }
}