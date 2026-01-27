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
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class NewPIDExternalIMU extends OpMode {

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
    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double targetAngle = 90;

    public static double MIN_ANGLE = 20;
    public static double MAX_ANGLE = 330;
    IMU turretImu;
    ElapsedTime timer = new ElapsedTime();
    public double lastError = 0;

    // TODO: FIND OUT THESE VALUES
    static final double TICKS_PER_REV = 1440; // adjust to your motor
    static final double GEAR_RATIO = 1.0;     //adjust if geared
    static final double TICKS_TO_RADIANS = 2 * Math.PI / (TICKS_PER_REV * GEAR_RATIO);

    @Override
    public void init() {
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        /*

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;

        turretImu.initialize(parameters);
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
         */

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        turretImu = hardwareMap.get(IMU.class, "turretImu");
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
        turretImu.initialize(new IMU.Parameters(orientationOnRobot));

        turretImu.resetYaw();
        // Set up our telemetry dashboard
        integralSum = 0;
        lastError = 0;

        /*
        parameters = new BHI260IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(parameters);
        imu.resetYaw();
         */
        /*BHI260IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);
        imu.resetYaw();

         */

    }

    @Override
    public void loop() {
        /* encoder values
        double power = PIDControl(target, motor.getCurrentPosition());
        motor.setPower(power);
        */
        /*
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentAngle = orientation.getYaw(AngleUnit.RADIANS);
        double turretRelativeAngle = motor.getCurrentPosition() * TICKS_TO_RADIANS;

        //  compute field-relative turret angle
        double turretAbsoluteAngle = turretRelativeAngle - currentAngle;

        // use turretAbsoluteAngle as state
        double power = PIDControl(targetAngleRadians, turretAbsoluteAngle);
         */

        /*
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentAngle = orientation.getYaw(AngleUnit.RADIANS);
        double power = PIDControl(targetAngleRadians, currentAngle);


         */
        controller.setPID(p,i,d);
        /*
        angles   = turretImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double currentYaw = angles.firstAngle;

         */
        YawPitchRollAngles orientation = turretImu.getRobotYawPitchRollAngles();
        double yawDeg = orientation.getYaw(AngleUnit.DEGREES);

        double currentAngleDeg = yawDeg;
        if (currentAngleDeg < 0) {
            currentAngleDeg += 360.0;
        }

        double unclampedTargetDeg = targetAngle;
        double currentTargetDeg = unclampedTargetDeg;
        if (currentTargetDeg < MIN_ANGLE) {
            currentTargetDeg = MIN_ANGLE;
        }
        if (currentTargetDeg > MAX_ANGLE) {
            currentTargetDeg = MAX_ANGLE;
        }



        double pidOutput = controller.calculate(currentAngleDeg,currentTargetDeg);

        if (currentAngleDeg <= MIN_ANGLE && pidOutput < 0) {
            pidOutput = 0;
        }
        if (currentAngleDeg >= MAX_ANGLE && pidOutput > 0) {
            pidOutput = 0;
        }

        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));

        rotationMotor.setPower(pidOutput);

        telemetry.addData("Pos: ", currentAngleDeg);
        telemetry.addData("target: ", currentTargetDeg);
        telemetry.update();
        //double power = PIDControl(targetAngleRadians, currentYaw);

        //rotationMotor.setPower(Math.max(-1.0, Math.min(1.0, power)));

        /*
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("target", target);
        telemetry.addData("current", currentAngle);
        telemetry.update();

         */

    }

    /*
    public double PIDControl(double reference, double state) {
        // IF CONTINUOUS
        // double error = angleWrap(reference - state);
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;



        timer.reset();

        double output = (error * kp) + (derivative * kd) + (integralSum * ki);
        return output;
    }
     */

    // IF CONTINUOUS

    /*
    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }
     */
}