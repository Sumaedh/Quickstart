package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.controller.PIDController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class OdometryTracking2 {

    private GoBildaPinpointDriver pinpoint;
    private DcMotor rotationMotor;

    private PIDController odoController;


    private static final double STARTING_X = 8.75;
    private static final double STARTING_Y = 56.625;
    private static final double STARTING_HEADING = 90;


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

    private double lastMotorPower = 0.0;

    public void initOdometry(HardwareMap hwMap) {
        odoController = new PIDController(p, iS, d);

        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        rotationMotor = hwMap.get(DcMotor.class, "rotationMotor");

        pinpoint.resetPosAndIMU();
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-5.156, -2.75, DistanceUnit.INCH);
        pinpoint.resetPosAndIMU();

        Pose2D startPosition = new Pose2D(
                DistanceUnit.INCH, STARTING_X, STARTING_Y,
                AngleUnit.DEGREES, STARTING_HEADING
        );
        pinpoint.setPosition(startPosition);

        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void setTargetPoint(double xInches, double yInches) {
        this.targetX = xInches;
        this.targetY = yInches;
    }


    public void aimAtHighBasket(boolean isBlueAlliance) {
        if (isBlueAlliance) {
            setTargetPoint(14, 139);
        } else {
            setTargetPoint(130, 139);
        }
    }

    public void odoLoop(Telemetry telemetry) {
        pinpoint.update();


        double xDistance = targetX - pinpoint.getPosX(DistanceUnit.INCH);
        double yDistance = targetY - pinpoint.getPosY(DistanceUnit.INCH);


        double angleToTarget = Math.toDegrees(Math.atan2(yDistance, xDistance));


        double robotHeading = pinpoint.getHeading(AngleUnit.DEGREES);

        double turretAngle = angleToTarget - robotHeading;

        turretAngle = AngleUnit.normalizeDegrees(turretAngle);

        double targetTicks = turretAngle * TICKS_PER_DEGREE;

        targetTicks = Math.max(LOW_LIMIT, Math.min(HIGH_LIMIT, targetTicks));

        double currentTicks = rotationMotor.getCurrentPosition();
        double errorTicks = targetTicks - currentTicks;

        if (Math.abs(errorTicks) <= TICK_DEADBAND) {
            rotationMotor.setPower(0);
            lastMotorPower = 0;
            telemetry.update();
            return;
        }

        odoController.setPID(p, iS, d);
        double pidOutput = odoController.calculate(currentTicks, targetTicks);

        double feedforward = Math.copySign(kFF, errorTicks);

        double motorPower = pidOutput + feedforward;

        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

        double delta = motorPower - lastMotorPower;
        if (delta > MAX_POWER_DELTA) {
            motorPower = lastMotorPower + MAX_POWER_DELTA;
        } else if (delta < -MAX_POWER_DELTA) {
            motorPower = lastMotorPower - MAX_POWER_DELTA;
        }

        if (currentTicks >= HIGH_LIMIT && motorPower > 0) motorPower = 0;
        if (currentTicks <= LOW_LIMIT && motorPower < 0) motorPower = 0;

        rotationMotor.setPower(motorPower);
        lastMotorPower = motorPower;

        telemetry.addData("Target X", targetX);
        telemetry.addData("Target Y", targetY);
        telemetry.addData("Robot X", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("Robot Y", pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("Angle To Target", angleToTarget);
        telemetry.addData("Robot Heading", robotHeading);
        telemetry.addData("Turret Angle (deg)", turretAngle);
        telemetry.addData("Target Ticks", targetTicks);
        telemetry.addData("Current Ticks", currentTicks);
        telemetry.addData("Error Ticks", errorTicks);
        telemetry.addData("Motor Power", motorPower);
        telemetry.update();
    }
}
