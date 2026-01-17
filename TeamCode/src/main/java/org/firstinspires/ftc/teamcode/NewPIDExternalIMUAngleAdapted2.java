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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "New PID external imu (ANGLE ADAPTED) 2")
public class NewPIDExternalIMUAngleAdapted2 extends OpMode {

    private PIDController controller;

    final int READ_PERIOD = 1;

    DcMotor intakeMotor;
    Servo pitchServo;
    DcMotor rotationMotor;
    DcMotor shootingMotor;
    DcMotor sorterMotor;
    Servo leverServo;
    HuskyLens huskyLens;
    HuskyLens huskyLens2;
    Deadline rateLimit;

    public static double p = 0.013;
    public static double i = 0;
    public static double d = 0.0004;
    public static double kFF = 0.05;
    public static double targetAngle = 0;

    IMU turretImu;

    private double lastYawDeg = 0.0;
    private double unwrappedYawDeg = 0.0;
    private double yawOffsetDeg = 0.0;

    // ADDED: encoder clip limits
    private static final int TURRET_ENCODER_HIGH_LIMIT = 1400;
    private static final int TURRET_ENCODER_LOW_LIMIT  = -1400;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turretImu = hardwareMap.get(IMU.class, "turretImu");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        pitchServo = hardwareMap.get(Servo.class,"pitchServo");
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        shootingMotor = hardwareMap.get(DcMotor.class, "shootingMotor");
        sorterMotor = hardwareMap.get(DcMotor.class, "sorterMotor");
        leverServo = hardwareMap.get(Servo.class,"leverServo");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens2 = hardwareMap.get(HuskyLens.class, "huskylens2");

        rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();
        leverServo.setPosition(0);

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        turretImu.initialize(new IMU.Parameters(orientationOnRobot));
        turretImu.resetYaw();

        YawPitchRollAngles initAngles = turretImu.getRobotYawPitchRollAngles();
        lastYawDeg = initAngles.getYaw(AngleUnit.DEGREES);
        unwrappedYawDeg = lastYawDeg;
        yawOffsetDeg = lastYawDeg;
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);

        YawPitchRollAngles orientation = turretImu.getRobotYawPitchRollAngles();
        double currentYawDeg = orientation.getYaw(AngleUnit.DEGREES);

        double delta = currentYawDeg - lastYawDeg;
        if (delta > 180.0) {
            delta -= 360.0;
        } else if (delta < -180.0) {
            delta += 360.0;
        }

        unwrappedYawDeg += delta;
        lastYawDeg = currentYawDeg;

        double currentAngleDeg = unwrappedYawDeg - yawOffsetDeg;
        double currentTargetDeg = (targetAngle > 180.0) ? targetAngle - 360.0 : targetAngle;
        double error = currentTargetDeg - currentAngleDeg;

        double pidOutput = controller.calculate(currentAngleDeg, currentTargetDeg);
        double feedforward = Math.copySign(kFF, error);
        double motorPower = pidOutput + feedforward;

        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

        // encoder limit protection
        int turretEnc = rotationMotor.getCurrentPosition();
        if ((turretEnc >= TURRET_ENCODER_HIGH_LIMIT && motorPower > 0.0) ||
                (turretEnc <= TURRET_ENCODER_LOW_LIMIT && motorPower < 0.0)) {
            motorPower = 0.0;
        }

        rotationMotor.setPower(motorPower);

        telemetry.addData("Turret Angle (deg)", currentAngleDeg);
        telemetry.addData("Target (deg)", currentTargetDeg);
        telemetry.addData("Error (deg)", error);
        telemetry.update();
    }
}
