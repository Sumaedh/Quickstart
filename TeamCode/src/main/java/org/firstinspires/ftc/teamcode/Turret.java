package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Turret {

    private PIDController controllerTurret;

    public static double pTurret = 0.01;
    public static double iTurret = 0.0;
    public static double dTurret = 0.0004;
    public static double kFFTurret = 0.042;
    public static double targetAngleTurret = 0;

    // IMU and yaw unwrap state (turret)
    private double lastYawDegTurret = 0.0;
    private double unwrappedYawDegTurret = 0.0;
    private double yawOffsetDegTurret = 0.0;

    private double initialErrorSignTurret = 0.0;   //  -1, 0, or +1

    DcMotor rotationMotorTurret;
    IMU turretImuTurret;

    // encoder clip limits (turret) — names end with Turret
    public static int TURRET_ENCODER_HIGH_LIMIT_Turret = 1700;
    public static int TURRET_ENCODER_LOW_LIMIT_Turret  = -2200;

    // wrap override state (turret)
    private boolean wrapOverrideActiveTurret = false;
    private boolean stuckAtHighLimitTurret = false;

    // ADDED: track when wrap override started so we can enforce the "1 second before forcing stop on limit" rule
    private long wrapStartTimeMs = 0;
    private static final long WRAP_MIN_ACTIVE_MS = 600; // 1 second


    public void initTurret(HardwareMap hwMap) {
        rotationMotorTurret = hwMap.get(DcMotor.class, "rotationMotor");
        turretImuTurret = hwMap.get(IMU.class, "turretImu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        turretImuTurret.initialize(new IMU.Parameters(orientationOnRobot));
        turretImuTurret.resetYaw();

        YawPitchRollAngles initAngles = turretImuTurret.getRobotYawPitchRollAngles();
        lastYawDegTurret = initAngles.getYaw(AngleUnit.DEGREES);
        unwrappedYawDegTurret = lastYawDegTurret;
        yawOffsetDegTurret = lastYawDegTurret;

        controllerTurret = new PIDController(pTurret, iTurret, dTurret);

        // turret motor settings mirror the original turret opmode
        rotationMotorTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotorTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationMotorTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void PIDFTurretLoop() {
        // ---------- Turret PID + IMU integration (exactly as original turret opmode) ----------
        controllerTurret.setPID(pTurret, iTurret, dTurret);

        YawPitchRollAngles orientation = turretImuTurret.getRobotYawPitchRollAngles();
        double currentYawDeg = orientation.getYaw(AngleUnit.DEGREES);

        double delta = currentYawDeg - lastYawDegTurret;
        if (delta > 180.0) delta -= 360.0;
        else if (delta < -180.0) delta += 360.0;

        unwrappedYawDegTurret += delta;
        lastYawDegTurret = currentYawDeg;

        double currentAngleDegTurret = unwrappedYawDegTurret - yawOffsetDegTurret;
        double currentTargetDegTurret = (targetAngleTurret > 180.0) ? targetAngleTurret - 360.0 : targetAngleTurret;
        double errorTurret = currentTargetDegTurret - currentAngleDegTurret;
        double errorTurret2 = currentTargetDegTurret - currentYawDeg;

        double pidOutputTurret = controllerTurret.calculate(currentAngleDegTurret, currentTargetDegTurret);
        double feedforwardTurret = Math.copySign(kFFTurret, errorTurret);
        double motorPowerTurret = pidOutputTurret + feedforwardTurret;

        motorPowerTurret = Math.max(-1.0, Math.min(1.0, motorPowerTurret));

        int turretEnc = rotationMotorTurret.getCurrentPosition();

        boolean atHighLimitTurret = turretEnc >= TURRET_ENCODER_HIGH_LIMIT_Turret;
        boolean atLowLimitTurret  = turretEnc <= TURRET_ENCODER_LOW_LIMIT_Turret;

        // HARD CLAMP
        if ((atHighLimitTurret && motorPowerTurret > 0.0) ||
                (atLowLimitTurret  && motorPowerTurret < 0.0)) {
            motorPowerTurret = 0.0;
        }

        // ENTER WRAP OVERRIDE
        if (!wrapOverrideActiveTurret &&
                motorPowerTurret == 0.0 &&
                Math.abs(errorTurret) > 35 &&
                (atHighLimitTurret || atLowLimitTurret)) {

            wrapOverrideActiveTurret = true;
            stuckAtHighLimitTurret = atHighLimitTurret;

            initialErrorSignTurret = Math.signum(errorTurret2);
            // ADDED: record the time we entered wrap override
            wrapStartTimeMs = System.currentTimeMillis();
        }

        // WRAP OVERRIDE BEHAVIOR
        if (wrapOverrideActiveTurret) {
            // ADDED: if we've been in wrap override for at least 1 second and we hit any encoder limit,
            // stop and exit wrap override (prevents never-stopping behavior as requested)
            boolean crossedTarget = (Math.signum(errorTurret2) != initialErrorSignTurret) &&
                    (Math.signum(errorTurret2) != 0);

            long elapsed = System.currentTimeMillis() - wrapStartTimeMs;
            if ((elapsed >= WRAP_MIN_ACTIVE_MS && (atHighLimitTurret || atLowLimitTurret)) || crossedTarget) {
                // stop and exit wrap override
                motorPowerTurret = 0.0;
                wrapOverrideActiveTurret = false;
            } else {
                if (Math.abs(errorTurret) < 15) {
                    wrapOverrideActiveTurret = false;
                } else {
                    motorPowerTurret = stuckAtHighLimitTurret ? -0.9 : 0.9;
                }
            }
        }

        rotationMotorTurret.setPower(motorPowerTurret);
    }

    public boolean turretAtTarget() {
        YawPitchRollAngles orientation = turretImuTurret.getRobotYawPitchRollAngles();
        double currentYawDeg = orientation.getYaw(AngleUnit.DEGREES);

        if ((Math.abs(currentYawDeg) - targetAngleTurret) <= 5) {
            return true;
        } else {
            return false;
        }
    }
}
