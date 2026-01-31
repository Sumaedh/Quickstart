package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Turret {

    private PIDController controllerTurret;

    // Your working values
    public static double pTurret = 0.03;
    public static double iTurret = 0.00;
    public static double dTurret = 0.0002;
    public static double kFFTurret = 0.02;

    Limelight3A limelight;
    DcMotor rotationMotorTurret;
    IMU turretImuTurret;

    public static int TURRET_ENCODER_HIGH_LIMIT_Turret = 1000;
    public static int TURRET_ENCODER_LOW_LIMIT_Turret = -1000;

    double lastTx = 0;
    private ElapsedTime lostTargetTimer = new ElapsedTime();
    private static final double LOST_TARGET_TIMEOUT_MS = 150;
    private boolean wasTargetValid = false;

    public void initTurret(HardwareMap hwMap, Telemetry telemetry) {
        rotationMotorTurret = hwMap.get(DcMotor.class, "rotationMotor");
        turretImuTurret = hwMap.get(IMU.class, "turretImu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        turretImuTurret.initialize(new IMU.Parameters(orientationOnRobot));
        turretImuTurret.resetYaw();

        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        controllerTurret = new PIDController(pTurret, iTurret, dTurret);

        rotationMotorTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotorTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationMotorTurret.setDirection(DcMotorSimple.Direction.REVERSE);
        rotationMotorTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lostTargetTimer.reset();
    }

    public void PIDFTurretLoop() {
        LLResult result = limelight.getLatestResult();

        boolean targetCurrentlyVisible = (result != null && result.isValid() &&
                result.getFiducialResults() != null &&
                !result.getFiducialResults().isEmpty());

        double motorPower = 0.0;
        int turretEnc = rotationMotorTurret.getCurrentPosition();
        boolean atHighLimit = turretEnc >= TURRET_ENCODER_HIGH_LIMIT_Turret;
        boolean atLowLimit = turretEnc <= TURRET_ENCODER_LOW_LIMIT_Turret;

        if (targetCurrentlyVisible) {
            lastTx = result.getTx();
            wasTargetValid = true;
            lostTargetTimer.reset();

            controllerTurret.setPID(pTurret, iTurret, dTurret);

            // CRITICAL: Check your sign here!
            // If tx > 0 (target right), you want positive power to turn right
            // If calculate(lastTx, 0) returns negative when tx is positive, flip sign below
            double pidOutput = controllerTurret.calculate(lastTx, 0);

            // Error sign for feedforward
            double error = -lastTx;
            double feedforward = Math.copySign(kFFTurret, error);

            motorPower = pidOutput + feedforward;

        } else {
            // Flicker grace period
            if (wasTargetValid && lostTargetTimer.milliseconds() < LOST_TARGET_TIMEOUT_MS) {
                double pidOutput = controllerTurret.calculate(lastTx, 0);
                double feedforward = Math.copySign(kFFTurret, -lastTx);
                motorPower = pidOutput + feedforward;
            } else {
                motorPower = 0.0;
                wasTargetValid = false;
            }
        }

        // Clamp to [-1, 1] BEFORE limit check
        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

        // LIMIT LOGIC: Only zero power if trying to move DEEPER into limit
        // This allows recovery when target moves to other side (or manual movement)
        if (atHighLimit && motorPower > 0.0) {
            motorPower = 0.0; // Trying to go right at right limit? Stop.
        } else if (atLowLimit && motorPower < 0.0) {
            motorPower = 0.0; // Trying to go left at left limit? Stop.
        }
        // NOTE: Removed controller.reset() - since I=0, no windup to clear!
        // Reset was potentially causing the "dead" state after limit

        rotationMotorTurret.setPower(motorPower);
    }
}