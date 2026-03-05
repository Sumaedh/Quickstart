package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@Config
@TeleOp
public class OdometryTester extends OpMode {

    private GoBildaPinpointDriver pinpoint;
    private DcMotor rotationMotor;

    private PIDController odoController;


    private static final double STARTING_X = 8.75;
    private static final double STARTING_Y = 56.625;
    private static final double STARTING_HEADING = 0;


    private double targetX = 14;
    private double targetY = 139;


    public static double p = 0.03;
    public static double iS = 0;
    public static double d = 0.0002;
    public static double kFF = 0.02;


    private static final int HIGH_LIMIT = 1200;
    private static final int LOW_LIMIT = -1200;


    private static final double TICK_DEADBAND = 10;


    private static final double MAX_POWER_DELTA = 0.05;


    private static final double GEAR_RATIO = ((double) 33 / 15);
    private static final double COUNTS_PER_REV = 383.6;
    private static final double TICKS_PER_DEGREE = (COUNTS_PER_REV * GEAR_RATIO) / 360.0;
    private static final double DEGREES_PER_TICK = 360.0 / (COUNTS_PER_REV * GEAR_RATIO);

    private double lastMotorPower = 0.0;

    @Override
    public void init() {
        odoController = new PIDController(p, iS, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");

        pinpoint.resetPosAndIMU();
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-5.156, -2.75, DistanceUnit.INCH);
        pinpoint.resetPosAndIMU();

        Pose2D startPosition = new Pose2D(
                DistanceUnit.INCH, 0, 0,
                AngleUnit.DEGREES, 0
        );

















        pinpoint.setPosition(startPosition);

        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        pinpoint.update();

        double xDistance = (targetX) - (pinpoint.getPosX(DistanceUnit.INCH) + STARTING_X);
        double yDistance = (targetY) - (pinpoint.getPosY(DistanceUnit.INCH) + STARTING_Y);


        double angleToTarget = (Math.toDegrees(Math.atan2(yDistance, xDistance))) - 90;
        double robotHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        double turretAngle = angleToTarget - robotHeading;

        double normalizedTurretAngle = AngleUnit.normalizeDegrees(turretAngle);

        double targetTicks = normalizedTurretAngle * TICKS_PER_DEGREE;
        double targetTicksNoNormalized = turretAngle * TICKS_PER_DEGREE;

        double currentTicks = rotationMotor.getCurrentPosition();
        double currentDegs = rotationMotor.getCurrentPosition() * DEGREES_PER_TICK;

        //double errorTicks = targetTicks - currentTicks;
        /*
        if (Math.abs(errorTicks) <= TICK_DEADBAND) {
            rotationMotor.setPower(0);
            lastMotorPower = 0;
            telemetry.update();
            return;
        }
         */
        odoController.setPID(p, iS, d);
        double pidOutput = odoController.calculate(currentTicks, targetTicks);

        //double feedforward = Math.copySign(kFF, errorTicks);


        double motorPower = Math.max(-1.0, Math.min(1.0, pidOutput));

        /*
        double delta = motorPower - lastMotorPower;
        if (delta > MAX_POWER_DELTA) {
            motorPower = lastMotorPower + MAX_POWER_DELTA;
        } else if (delta < -MAX_POWER_DELTA) {
            motorPower = lastMotorPower - MAX_POWER_DELTA;
        }
         */

        //if (currentTicks >= HIGH_LIMIT && motorPower > 0) motorPower = 0;
        //if (currentTicks <= LOW_LIMIT && motorPower < 0) motorPower = 0;

        rotationMotor.setPower(motorPower);
        lastMotorPower = motorPower;

        telemetry.addData("Target X", targetX);
        telemetry.addData("Target Y", targetY);
        telemetry.addData("Robot X", (pinpoint.getPosX(DistanceUnit.INCH) + STARTING_X));
        telemetry.addData("Robot Y", (pinpoint.getPosY(DistanceUnit.INCH) + STARTING_Y));
        telemetry.addData("Angle To Target", angleToTarget);
        telemetry.addData("Robot Heading", robotHeading);
        telemetry.addData("Turret Angle (deg)", turretAngle);
        telemetry.addData("Normalized Turret Angle (deg)", normalizedTurretAngle);
        telemetry.addData("Target Ticks", targetTicks);
        telemetry.addData("Target Ticks (NOT NORMALIZED)", targetTicksNoNormalized);
        telemetry.addData("Current Ticks", currentTicks);
        telemetry.addData("Current Degs", currentDegs);
        //telemetry.addData("Error Ticks", errorTicks);
        telemetry.addData("Motor Power", motorPower);
        telemetry.update();
    }
}
