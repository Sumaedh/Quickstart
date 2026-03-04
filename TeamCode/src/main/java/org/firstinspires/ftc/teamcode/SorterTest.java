package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "Sorter Test")
public class SorterTest extends OpMode {
    private PIDController controller;

    final int READ_PERIOD = 1;

    // TODO: find lever pos
    DcMotor sorterMotor;
    Deadline rateLimit;

    public static double integralSum = 0;
    public static double p = 0.0083;
    public static double i = 0.0;
    public static double d = 0.0002;
    public static double kS = 0.0;

    private static final double TICKS_PER_REV = 537.6 * ((double) 10 / 14);

    public static double INCREMENT = TICKS_PER_REV / 3;

    double target = 0;


    private ElapsedTime boostTimer = new ElapsedTime();
    private boolean boostActive = false;

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
        sorterMotor = hardwareMap.get(DcMotor.class, "sorterMotor");

        sorterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        sorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Added: For direct power control in PID
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Added: To hold position

        // Rate limiter for telemetry
        rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

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
        // Check for target changes and trigger boost
        if (gamepad1.aWasPressed()) {
            target += INCREMENT;
            boostActive = true;
            boostTimer.reset();
        }

        controller.setPID(p, i, d);
        double currentPos = sorterMotor.getCurrentPosition();
        double error = target - currentPos;

        double motorPower;

        /*
        // BOOST PHASE: Full power for 0.25s when target changes
        if (boostActive && boostTimer.seconds() < 0.1) {
            // Full speed in the direction we need to go
            // Use Math.signum(error) to get -1 or 1 based on direction
            motorPower = Math.signum(error); // or 0.8 if full power is too violent
        }

         */
        // PID PHASE: Normal control after 0.25s or if boost finished

        boostActive = false; // Reset flag
        double pidOutput = controller.calculate(currentPos, target);
        //double staticFF = kS * Math.signum(error);
        //motorPower = pidOutput + staticFF;


        sorterMotor.setPower(pidOutput);

        telemetry.addData("Boost Active: ", boostActive);
        telemetry.addData("Boost Time: ", boostActive ? boostTimer.seconds() : 0);
        telemetry.addData("Pos: ", currentPos);
        telemetry.addData("Target: ", target);
        telemetry.addData("Error: ", error);
        telemetry.update();
    }
}
