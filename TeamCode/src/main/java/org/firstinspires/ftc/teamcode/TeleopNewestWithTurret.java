package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "REAL TELEOP (with turret)")
public class TeleopNewestWithTurret extends OpMode {

    // Shooting / sorter / drive (existing)
    public double fShooting = 15;
    public double fShootingshort = 15.25;
    public double pShooting = 250;
    public double curTargetVelocity = 0;

    private DcMotor sorterMotor;
    private PIDController sorterController;

    private double pSorting = 0.004;
    private double iSorting = 0.0;
    private double dSorting = 0.00027;

    public static double kSSorting = 0.034;

    double lastTx = 0;

    private static final double TICKS_PER_REV = 537.6 * ((double) 10 / 14);

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

    DistanceSensor distanceSensor2;
    DcMotor backRight;
    ElapsedTime timer = new ElapsedTime();

    // ---------- Turret PID/IMU variables (appended 'Turret') ----------
    private PIDController controllerTurret;

    public static double pTurret = 0.03;
    public static double iTurret = 0.00;
    public static double dTurret = 0.0002;
    public static double kFFTurret = 0.02;
    public static double targetAngleTurret = 0;

    // IMU and yaw unwrap state (turret)
    IMU turretImuTurret;
    private double lastYawDegTurret = 0.0;
    private double unwrappedYawDegTurret = 0.0;
    private double yawOffsetDegTurret = 0.0;

    // turret motor (renamed)
    DcMotor rotationMotorTurret;

    Limelight3A limelight;



    // encoder clip limits (turret) — names end with Turret
    public static int TURRET_ENCODER_HIGH_LIMIT_Turret = 1000;
    public static int TURRET_ENCODER_LOW_LIMIT_Turret  = -1000;

    // wrap override state (turret)
    private boolean wrapOverrideActiveTurret = false;
    private boolean stuckAtHighLimitTurret = false;

    private boolean relocalize = false;

    // FSM States for relocalization
    private enum SorterState {
        IDLE,           // Normal PID operation
        SEEKING,        // Moving toward sensor target
        CONFIRMING      // Ball detected, waiting 0.1s to confirm settled
    }
    private SorterState sorterState = SorterState.IDLE;

    private ElapsedTime confirmTimer = new ElapsedTime();
    private final double CONFIRM_TIME_SEC = 0.1;
    private final double TARGET_RANGE_MIN = 1.4;
    private final double TARGET_RANGE_MAX = 2.0;
    private final double SEEK_POWER = 0.2;

    int turns = 0;

    // Utility drive functions (unchanged)
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
        // telemetry to dashboard + driver station
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // sorter
        sorterController = new PIDController(pSorting,iSorting,dSorting);
        sorterMotor = hardwareMap.get(DcMotor.class, "sorterMotor");
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sorterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        sorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // basic hardware
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        pitchServo = hardwareMap.get(Servo.class,"pitchServo");
        rotationMotorTurret = hardwareMap.get(DcMotor.class, "rotationMotor"); // turret motor hw name unchanged
        shootingMotor = hardwareMap.get(DcMotorEx.class, "shootingMotor");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "distanceSensor2");
        huskyLens2 = hardwareMap.get(HuskyLens.class, "huskylens2");
        leverServo = hardwareMap.get(Servo.class,"leverServo");
        turretImuTurret = hardwareMap.get(IMU.class, "turretImu");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();
        limelight.pipelineSwitch(0);
        telemetry.setMsTransmissionInterval(11);

        // shooting PIDF
        PIDFCoefficients pidfShooting =
                new PIDFCoefficients(pShooting, 0, 0, fShooting);
        shootingMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, pidfShooting
        );

        // Use the same RevHub orientation as in the original turret opmode
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

        // turret motor settings mirror the original turret opmode
        rotationMotorTurret.setDirection(DcMotorSimple.Direction.REVERSE);
        rotationMotorTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotorTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationMotorTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // rest of teleop hardware/setup
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

        sorterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shootingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    public void start() {
        timer.reset();
        pitchServo.setPosition(0.42);
    }

    @Override
    public void loop() {

        // PITCH (unchanged)
        pitchServo.setPosition(0.42);

        // DRIVE (unchanged)
        if (gamepad1.left_trigger > 0.75) {
            driveMecanumSlower(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveMecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        // INTAKE (unchanged)
        if (gamepad1.a) intakeMotor.setPower(1.0);
        if (gamepad1.b) intakeMotor.setPower(0);
        if (gamepad1.x) intakeMotor.setPower(-1.0);

        // LEVER (unchanged)
        if (gamepad2.dpad_up) leverServo.setPosition(0.2);
        else leverServo.setPosition(0);


        /*
        if (gamepad2.aWasPressed()) {
            turns += 1;
            target = (turns % 6) * INCREMENT;
        }

        if (gamepad2.yWasPressed()) {
            turns += 2;
            target = (turns % 6) * INCREMENT;
        }
         */

        // SORTER (unchanged)
        if (gamepad2.aWasPressed()) {
            target += INCREMENT;
        }

        if (gamepad2.yWasPressed()) {
            target += (INCREMENT * 2);
        }

        sorterController.setPID(pSorting, iSorting, dSorting);
        double currentPos = sorterMotor.getCurrentPosition();
        double error = target - currentPos;

        double motorPowerSorter;

        double pidOutput = sorterController.calculate(currentPos, target);
        double staticFF = kSSorting * Math.signum(error);
        motorPowerSorter = pidOutput + staticFF;


        sorterMotor.setPower(motorPowerSorter);

        telemetry.addData("target pos", target);
        telemetry.addData("current pos", currentPos);
        telemetry.update();

        // SHOOTING (unchanged)
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

        // ---------- Turret PID + IMU integration (exactly as original turret opmode) ----------
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();

            lastTx = result.getTx();
        }

        controllerTurret.setPID(pTurret, iTurret, dTurret);
        double errorTurret = 0 - lastTx;

        double pidOutputTurret = controllerTurret.calculate(lastTx, 0);
        double feedforward = Math.copySign(kFFTurret, errorTurret);
        double motorPowerTurret = pidOutputTurret + feedforward;

        motorPowerTurret = Math.max(-1.0, Math.min(1.0, motorPowerTurret));
        int turretEnc = rotationMotorTurret.getCurrentPosition();


        telemetry.addData("tx", lastTx);
        telemetry.addData("target", target);
        telemetry.addData("Error (deg)", error);
        telemetry.addData("ticks", rotationMotorTurret.getCurrentPosition());
        telemetry.addData("Targets Visible", result != null ? result.getFiducialResults().size() : 0);
        telemetry.update();

        boolean atHighLimitTurret = turretEnc >= TURRET_ENCODER_HIGH_LIMIT_Turret;
        boolean atLowLimitTurret  = turretEnc <= TURRET_ENCODER_LOW_LIMIT_Turret;

        // HARD CLAMP
        if ((atHighLimitTurret && motorPowerTurret > 0.0) ||
                (atLowLimitTurret  && motorPowerTurret < 0.0)) {
            motorPowerTurret = 0.0;
        }

        rotationMotorTurret.setPower(motorPowerTurret);

        /*
        // ENTER WRAP OVERRIDE
        if (!wrapOverrideActiveTurret &&
                motorPowerTurret == 0.0 &&
                Math.abs(errorTurret) > 30 &&
                (atHighLimitTurret || atLowLimitTurret)) {

            wrapOverrideActiveTurret = true;
            stuckAtHighLimitTurret = atHighLimitTurret;
        }

        // WRAP OVERRIDE BEHAVIOR
        if (wrapOverrideActiveTurret) {
            timer.reset();
            if (Math.abs(errorTurret) < 15) {
                wrapOverrideActiveTurret = false;
            } else {
                motorPowerTurret = stuckAtHighLimitTurret ? -0.8 : 0.8;
            }
        }

         */

        telemetry.update();
    }
}
