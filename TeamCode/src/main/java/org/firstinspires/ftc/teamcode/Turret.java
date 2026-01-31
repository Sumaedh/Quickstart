package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Turret {

    private PIDController controllerTurret;

    public static double pTurret = 0.03;
    public static double iTurret = 0.001;
    public static double dTurret = 0.0004;
    public static double kFFTurret = 0.045;
    public static double targetAngleTurret = 0;

    Limelight3A limelight;
    DcMotor rotationMotorTurret;
    IMU turretImuTurret;

    public static int TURRET_ENCODER_HIGH_LIMIT_Turret = 1000;
    public static int TURRET_ENCODER_LOW_LIMIT_Turret  = -1000;

    double target = 0;
    double lastTx = 0;

    // ANTI-FLICKER: Timeout variables
    private ElapsedTime lostTargetTimer = new ElapsedTime();
    private static final double LOST_TARGET_TIMEOUT_MS = 250; // Allow 150ms of flickering
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
        telemetry.setMsTransmissionInterval(11);

        controllerTurret = new PIDController(pTurret, iTurret, dTurret);

        rotationMotorTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotorTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationMotorTurret.setDirection(DcMotorSimple.Direction.REVERSE);
        rotationMotorTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lostTargetTimer.reset();
    }

    public void PIDFTurretLoop() {
        LLResult result = limelight.getLatestResult();

        // Check if we have a valid target RIGHT NOW
        boolean targetCurrentlyVisible = (result != null && result.isValid() &&
                result.getFiducialResults() != null &&
                !result.getFiducialResults().isEmpty());

        double motorPower = 0.0;

        if (targetCurrentlyVisible) {
            // We see the tag - update tracking
            lastTx = result.getTx();
            wasTargetValid = true;
            lostTargetTimer.reset(); // Reset the "flicker allowance" timer

            // Calculate PID normally
            controllerTurret.setPID(pTurret, iTurret, dTurret);
            double error = -lastTx;
            double pidOutput = controllerTurret.calculate(lastTx, 0);
            double feedforward = Math.copySign(kFFTurret, error);
            motorPower = pidOutput + feedforward;
            motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

        } else {
            // Tag flickered out or is truly lost
            if (wasTargetValid && lostTargetTimer.milliseconds() < LOST_TARGET_TIMEOUT_MS) {
                // Within grace period (flickering) - keep using last known position
                // Continue calculating PID with lastTx
                double error = -lastTx;
                double pidOutput = controllerTurret.calculate(lastTx, 0);
                double feedforward = Math.copySign(kFFTurret, error);
                motorPower = pidOutput + feedforward;
                motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

                // Optional: Reduce power slightly while coasting on memory
                // motorPower *= 0.8;

            } else {
                // Truly lost for >150ms - stop to prevent drift
                motorPower = 0.0;
                wasTargetValid = false; // Reset so next valid detection starts fresh
            }
        }

        // Safety limits always enforced
        int turretEnc = rotationMotorTurret.getCurrentPosition();
        boolean atHighLimit = turretEnc >= TURRET_ENCODER_HIGH_LIMIT_Turret;
        boolean atLowLimit = turretEnc <= TURRET_ENCODER_LOW_LIMIT_Turret;

        if ((atHighLimit && motorPower > 0.0) ||
                (atLowLimit && motorPower < 0.0)) {
            motorPower = 0.0;
        }

        rotationMotorTurret.setPower(motorPower);
    }
}