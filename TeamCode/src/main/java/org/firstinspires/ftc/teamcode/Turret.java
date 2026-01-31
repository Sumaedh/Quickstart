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
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Config
public class Turret {

    private PIDController controllerTurret;

    // CORRECTED PID VALUES
    public static double pTurret = 0.03;
    public static double iTurret = 0.00;
    public static double dTurret = 0.0002;
    public static double kFFTurret = 0.02;

    Limelight3A limelight;
    DcMotor rotationMotorTurret;
    IMU turretImuTurret;

    public static int TURRET_ENCODER_HIGH_LIMIT_Turret = 1000;
    public static int TURRET_ENCODER_LOW_LIMIT_Turret  = -1000;

    double lastTx = 0;

    // Anti-flicker variables
    private ElapsedTime lostTargetTimer = new ElapsedTime();
    private static final double LOST_TARGET_TIMEOUT_MS = 150;
    private boolean wasTargetValid = false;
    private boolean wasAtLimit = false; // Track limit state for reset

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

        // Initialize PID with corrected values
        controllerTurret = new PIDController(pTurret, iTurret, dTurret);

        rotationMotorTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotorTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationMotorTurret.setDirection(DcMotorSimple.Direction.REVERSE);
        rotationMotorTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lostTargetTimer.reset();
    }

    public void PIDFTurretLoop() {
        LLResult result = limelight.getLatestResult();

        // Check if we have a valid target (with flicker tolerance)
        boolean targetCurrentlyVisible = (result != null && result.isValid() &&
                result.getFiducialResults() != null &&
                !result.getFiducialResults().isEmpty());

        double motorPower = 0.0;

        // Get encoder position for limit checking
        int turretEnc = rotationMotorTurret.getCurrentPosition();
        boolean atHighLimit = turretEnc >= TURRET_ENCODER_HIGH_LIMIT_Turret;
        boolean atLowLimit = turretEnc <= TURRET_ENCODER_LOW_LIMIT_Turret;

        if (targetCurrentlyVisible) {
            // We see the tag - update tracking (same as working LimelightTurret)
            lastTx = result.getTx();
            wasTargetValid = true;
            lostTargetTimer.reset();

            // Calculate PID (identical to working standalone version)
            controllerTurret.setPID(pTurret, iTurret, dTurret);
            double error = -lastTx; // Target is 0, current is lastTx

            double pidOutput = controllerTurret.calculate(lastTx, 0);
            double feedforward = Math.copySign(kFFTurret, error);
            motorPower = pidOutput + feedforward;

        } else {
            // Tag flickered out - use timeout grace period
            if (wasTargetValid && lostTargetTimer.milliseconds() < LOST_TARGET_TIMEOUT_MS) {
                // Keep using last known position (coast through flicker)
                double error = -lastTx;
                double pidOutput = controllerTurret.calculate(lastTx, 0);
                double feedforward = Math.copySign(kFFTurret, error);
                motorPower = pidOutput + feedforward;
            } else {
                // Truly lost for >150ms
                motorPower = 0.0;
                wasTargetValid = false;
            }
        }

        // Clamp to [-1, 1]
        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

        // LIMIT PROTECTION LOGIC
        boolean tryingToGoHigh = motorPower > 0.0;
        boolean tryingToGoLow = motorPower < 0.0;
        boolean hitLimit = false;

        // Only stop if trying to move deeper into limit (allow escaping)
        if (atHighLimit && tryingToGoHigh) {
            motorPower = 0.0;
            hitLimit = true;
        } else if (atLowLimit && tryingToGoLow) {
            motorPower = 0.0;
            hitLimit = true;
        }

        // CRITICAL FIX: Reset PID when we first hit a limit to clear integral windup
        // This prevents getting stuck when the tag returns after hitting limit
        if (hitLimit && !wasAtLimit) {
            controllerTurret.reset();
        }
        wasAtLimit = hitLimit;

        rotationMotorTurret.setPower(motorPower);
    }

    // Emergency reset if turret gets stuck (call from teleop if needed)
    public void resetTurretState() {
        controllerTurret.reset();
        wasTargetValid = false;
        lastTx = 0;
        wasAtLimit = false;
    }

    // uioiojiojioj
    public double getLastTx() {
        return lastTx;
    }

    public boolean hasValidTarget() {
        return wasTargetValid;
    }
}