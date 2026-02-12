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

public class OdometryTracking {

    // X IS FORWARD, Y IS BACKWARD
    // COUNTER CLOCKWISE IS POSITIVE FOR IMU PINPOINT
    GoBildaPinpointDriver pinpoint;
    DcMotor rotationMotor;

    private PIDController odoController;

    // POSES
    double STARTING_X = 8.75;
    double STARTING_Y = 56.625;
    double STARTING_HEADING = 90;

    double TARGET_X = 135;
    double TARGET_Y = 10;

    // CONSTANTS
    public static double p = 0.03;
    public static double iS = 0;
    public static double d = 0.0002;
    public static double kFF = 0.02;

    // LIMITS
    private static final int HIGH_LIMIT = 1200;
    private static final int LOW_LIMIT = -1200;

    double xDistance;
    double yDistance;
    double angle;

    // TICKS PER DEG CALC
    // TICKS PER DEGREE = (MOTOR COUNTS PER REVOLUTION) * (GEAR RATIO) / 360 (TICKS/ DEGREES)
    // DEGREE PER TICKS = 360 / (MOTOR COUNTS PER REVOLUTION) * (GEAR RATIO) (DEGREES/ TICKS)
    double GEAR_RATIO =  ((double) 33 / 15);
    double COUNTS_PER_REV = 383.6;


    public void initOdometry(HardwareMap hwMap) {
        odoController = new PIDController(p, iS, d);

        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        rotationMotor = hwMap.get(DcMotor.class, "rotationMotor");
        pinpoint.resetPosAndIMU();
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-5.156, -2.75, DistanceUnit.INCH);
        pinpoint.resetPosAndIMU();
        Pose2D startPosition = new Pose2D(DistanceUnit.INCH, STARTING_X, STARTING_Y, AngleUnit.DEGREES, STARTING_HEADING);
        pinpoint.setPosition(startPosition);

        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void odoLoop(Telemetry telemetry) {
        pinpoint.update();

        xDistance = TARGET_X - pinpoint.getPosX(DistanceUnit.INCH);
        yDistance = TARGET_Y - pinpoint.getPosY(DistanceUnit.INCH);

        angle = Math.toDegrees(Math.atan2(yDistance, xDistance));
        double targetAngleToTicks = (((COUNTS_PER_REV * GEAR_RATIO) / 360) * angle) - (((COUNTS_PER_REV * GEAR_RATIO) / 360) * pinpoint.getHeading(AngleUnit.DEGREES));
        double currentTicks = rotationMotor.getCurrentPosition();

        odoController.setPID(p, iS, d);
        double error = targetAngleToTicks - currentTicks;

        double pidOutput = odoController.calculate(currentTicks, targetAngleToTicks);
        double feedforward = Math.copySign(kFF, error);
        double motorPower = pidOutput + feedforward;

        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

        rotationMotor.setPower(motorPower);

        telemetry.update();
    }
}
