package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "Sorter Test")
public class SorterTest extends OpMode {
    private PIDController controller;

    final int READ_PERIOD = 1;

    // TODO: find lever pos
    double leverPos = 0.5;

    DcMotor intakeMotor;
    Servo pitchServo;
    DcMotor rotationMotor;
    DcMotor shootingMotor;
    DcMotor sorterMotor;
    Servo leverServo;
    //ColorSensor colorSensor;
    HuskyLens huskyLens;
    HuskyLens huskyLens2;
    Deadline rateLimit;

    public static double integralSum = 0;
    private double p = 0.0029;
    private double i = 0.0;
    private double d = 0.00017;

    public static double kS = 0.02;

    private static final double TICKS_PER_REV = 537.6;

    public static double INCREMENT = TICKS_PER_REV / 6;

    public static double targetAngle = 90;
    public static double MIN_ANGLE = 20;
    public static double MAX_ANGLE = 330;

    double target = 0;


    ElapsedTime timer = new ElapsedTime();
    public double lastError = 0;

    @Override
    public void init() {
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        /* BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        turretImu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); */

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        pitchServo = hardwareMap.get(Servo.class,"pitchServo");
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        shootingMotor = hardwareMap.get(DcMotor.class, "shootingMotor");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens2 = hardwareMap.get(HuskyLens.class, "huskylens2");
        sorterMotor = hardwareMap.get(DcMotor.class, "sorterMotor");
        leverServo = hardwareMap.get(Servo.class,"leverServo");

        rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sorterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        sorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Added: For direct power control in PID
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Added: To hold position

        // Rate limiter for telemetry
        rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        leverServo.setPosition(0);

        // Choose the algorithm
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        // huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        // Set up parameters for turret orientation (adjust based on mounting)
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        // Initialize


        // Set up our telemetry dashboard
        integralSum = 0;
        lastError = 0;
    }

    @Override
    public void loop() {


        if (gamepad1.aWasPressed()) {
            target += INCREMENT;
        }


        controller.setPID(p,i,d);
        double currentPos = sorterMotor.getCurrentPosition();
        double pidOutput = controller.calculate(currentPos, target);
        double error = target - currentPos;
        double staticFF = kS * Math.signum(error);

        sorterMotor.setPower(pidOutput + staticFF);

        telemetry.addData("Pos: ", sorterMotor.getCurrentPosition());
        telemetry.addData("target: ", target);
        telemetry.update();
    }
}
