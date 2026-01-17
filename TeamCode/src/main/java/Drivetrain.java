package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Locale;

public class Drivetrain {
    public DcMotor frontLeft  = null;
    public DcMotor backLeft   = null;
    public DcMotor frontRight = null;
    public DcMotor backRight  = null;

    static private final double     COUNTS_PER_MOTOR_REV    = 537.6 ;
    static private final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static private final double     WHEEL_DIAMETER_INCHES   = 104/25.4 ;     // For figuring circumference
    static private final double     WHEEL_COUNTS_PER_INCH   = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    static private final int Reverse = 1;
    static private final int Forward = -1;

    private ElapsedTime runtime = new ElapsedTime();

    IMU imu;
    IMU.Parameters parameters;
    YawPitchRollAngles angles;

    LinearOpMode opMode;

    public Drivetrain(LinearOpMode op) {
        opMode = op;
    }

    public void drive(double speed, double distance) {
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeft.getCurrentPosition() + (int)(distance * WHEEL_COUNTS_PER_INCH);
            newBackLeftTarget = backLeft.getCurrentPosition() + (int)(distance * WHEEL_COUNTS_PER_INCH);
            newFrontRightTarget = frontRight.getCurrentPosition() + (int)(distance * WHEEL_COUNTS_PER_INCH);
            newBackRightTarget = backRight.getCurrentPosition() + (int)(distance * WHEEL_COUNTS_PER_INCH);

            frontLeft.setTargetPosition(newFrontLeftTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backRight.setTargetPosition(newBackRightTarget);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeft.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opMode.opModeIsActive() && (frontLeft.isBusy() && frontRight.isBusy())) {

                // Display it for the driver.
                opMode.telemetry.addData("Path1",  "Running to FL %7d :FR %7d :BL %7d :BR %7d", newFrontLeftTarget,  newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                opMode.telemetry.addData("Path2",  "Running at FL %7f :FR %7f :BL %7f :BR %7f",
                        frontLeft.getPower(),
                        frontRight.getPower(),
                        backLeft.getPower(),
                        backRight.getPower());
                opMode.telemetry.update();

                if(opMode.isStopRequested()) {
                    break;
                }
            }

            // Stop all motion;
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
        }
    }

    public void strafe(String direction, double speed, double distance) {
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        direction = direction.toLowerCase();

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
//            newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (distance * WHEEL_COUNTS_PER_INCH);
//            newBackLeftTarget = backLeft.getCurrentPosition() + (int) (distance * WHEEL_COUNTS_PER_INCH);
//            newFrontRightTarget = frontRight.getCurrentPosition() + (int) (distance * WHEEL_COUNTS_PER_INCH);
//            newBackRightTarget = backRight.getCurrentPosition() + (int) (distance * WHEEL_COUNTS_PER_INCH);
//
//            if (direction.equals("right")) {
//                newFrontLeftTarget *= Reverse;
//                newBackLeftTarget *= Forward;
//                newFrontRightTarget *= Forward;
//                newBackRightTarget *= Reverse;
//            } else if (direction.equals("left")) {
//                newFrontLeftTarget *= Forward;
//                newBackLeftTarget *= Reverse;
//                newFrontRightTarget *= Reverse;
//                newBackRightTarget *= Forward;
//            }

            if(direction.equals("right")) {
                newFrontLeftTarget = frontLeft.getCurrentPosition() - (int) (distance * WHEEL_COUNTS_PER_INCH);
                newBackLeftTarget = backLeft.getCurrentPosition() + (int) (distance * WHEEL_COUNTS_PER_INCH);
                newFrontRightTarget = frontRight.getCurrentPosition() + (int) (distance * WHEEL_COUNTS_PER_INCH);
                newBackRightTarget = backRight.getCurrentPosition() - (int) (distance * WHEEL_COUNTS_PER_INCH);
            }
            else if(direction.equals("left")) {
                newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (distance * WHEEL_COUNTS_PER_INCH);
                newBackLeftTarget = backLeft.getCurrentPosition() - (int) (distance * WHEEL_COUNTS_PER_INCH);
                newFrontRightTarget = frontRight.getCurrentPosition() - (int) (distance * WHEEL_COUNTS_PER_INCH);
                newBackRightTarget = backRight.getCurrentPosition() + (int) (distance * WHEEL_COUNTS_PER_INCH);
            }
            else {
                newFrontLeftTarget = frontLeft.getCurrentPosition();
                newBackLeftTarget = backLeft.getCurrentPosition();
                newFrontRightTarget = frontRight.getCurrentPosition();
                newBackRightTarget = backRight.getCurrentPosition();
            }

            frontLeft.setTargetPosition(newFrontLeftTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backRight.setTargetPosition(newBackRightTarget);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeft.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            while (opMode.opModeIsActive() && (frontLeft.isBusy() && frontRight.isBusy())) {

                // Display it for the driver.
                opMode.telemetry.addData("Path1",  "Running to FL %7d :FR %7d :BL %7d :BR %7d", newFrontLeftTarget,  newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                opMode.telemetry.addData("Path2",  "Running at FL %7f :FR %7f :BL %7f :BR %7f",
                        frontLeft.getPower(),
                        frontRight.getPower(),
                        backLeft.getPower(),
                        backRight.getPower());
                opMode.telemetry.update();

                if(opMode.isStopRequested()) {
                    break;
                }
            }

            // Stop all motion;
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
        }
    }

    public double getAngleDifference(double angle1, double angle2) {

        return (angle1 - angle2 + 540) % 360 - 180;

    }

    public void turn(double power, double angle, String direction) {
        angles = imu.getRobotYawPitchRollAngles();

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu.resetYaw();

        if (direction.equals("left")) {
            power *= -1;
        }

        frontLeft.setPower(-power);
        backLeft.setPower(-power);
        frontRight.setPower(power);
        backRight.setPower(power);

        while (opMode.opModeIsActive() && Math.abs(angles.getYaw(AngleUnit.DEGREES)) < angle) {
            opMode.telemetry.addData("heading", angles.getYaw(AngleUnit.DEGREES));
            opMode.telemetry.update();
            angles = imu.getRobotYawPitchRollAngles();

            if(opMode.isStopRequested()) {
                break;
            }
        }

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        initEncoders();
        imu.resetYaw();
    }

    public void turnButBetterThanTheOldTurn(double power, double angle, String direction) {
        angles = imu.getRobotYawPitchRollAngles();

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu.resetYaw();

        int directionMultiplier = -1;

        if (direction.equals("left")) {
            power *= -1;
            directionMultiplier *= -1;
        }

        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);

        while (opMode.opModeIsActive() && directionMultiplier * getAngleDifference(angles.getYaw(AngleUnit.DEGREES), angle) < 0) {
            opMode.telemetry.addData("heading", angles.getYaw(AngleUnit.DEGREES));
            opMode.telemetry.update();
            angles = imu.getRobotYawPitchRollAngles();

            if(opMode.isStopRequested()) {
                break;
            }
        }

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        initEncoders();
    }

    public void turnButDoesntResetAngleUnlikeTurnButBetterThanTheOldTurn(double power, double angle, String direction) {
        angles = imu.getRobotYawPitchRollAngles();

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int directionMultiplier = -1;

        if (direction.equals("closest")) {

            if (getAngleDifference(angles.getYaw(AngleUnit.DEGREES), angle) < 0) {
                power *= -1;
                directionMultiplier *= -1;
            }

        } else if (direction.equals("left")) {
            power *= -1;
            directionMultiplier *= -1;
        }

        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);

        while (opMode.opModeIsActive() && directionMultiplier * getAngleDifference(angles.getYaw(AngleUnit.DEGREES), angle) < 0) {
            opMode.telemetry.addData("heading", angles.getYaw(AngleUnit.DEGREES));
            opMode.telemetry.update();
            angles = imu.getRobotYawPitchRollAngles();

            if(opMode.isStopRequested()) {
                break;
            }
        }

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        initEncoders();
    }

    void turnToZero(double power, String direction) {
        angles = imu.getRobotYawPitchRollAngles();

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu.resetYaw();

        if (direction.equals("left")) {
            power *= -1;
        }

        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);

        while (opMode.opModeIsActive() && Math.abs(angles.getYaw(AngleUnit.DEGREES)) < 2) {
            opMode.telemetry.addData("heading", angles.getYaw(AngleUnit.DEGREES));
            opMode.telemetry.update();
            angles = imu.getRobotYawPitchRollAngles();
        }

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        initEncoders();
        imu.resetYaw();
    }

    void turnToAngle(double power, String direction, double angle) {
        angles = imu.getRobotYawPitchRollAngles();

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int directionMultiplier = -1;

        if (direction.equals("closest")) {

            if (getAngleDifference(angles.getYaw(AngleUnit.DEGREES), angle) < 0) {
                power *= -1;
                directionMultiplier *= -1;
            }

        } else if (direction.equals("left")) {
            power *= -1;
            directionMultiplier *= -1;
        }

        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);

        while (opMode.opModeIsActive() && directionMultiplier * getAngleDifference(angles.getYaw(AngleUnit.DEGREES), angle) < 0) {
            opMode.telemetry.addData("heading", angles.getYaw(AngleUnit.DEGREES));
            opMode.telemetry.update();
            angles = imu.getRobotYawPitchRollAngles();
        }

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        initEncoders();
        imu.resetYaw();
    }

    public void initDrivetrain(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backRight = hwMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    public void initGyro(HardwareMap hwMap) {
        parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );


        imu = hwMap.get(IMU.class, "imu");
        angles = imu.getRobotYawPitchRollAngles();
        imu.initialize(parameters);
    }

    void initEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
