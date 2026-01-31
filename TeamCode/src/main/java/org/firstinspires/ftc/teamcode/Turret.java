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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Turret {

    private PIDController controllerTurret;

    public static double pTurret = 0.03;
    public static double iTurret = 0.001;
    public static double dTurret = 0.0004;
    public static double kFFTurret = 0.045;
    public static double targetAngleTurret = 0;

    // IMU and yaw unwrap state (turret)

    Limelight3A limelight;
    DcMotor rotationMotorTurret;
    IMU turretImuTurret;

    // encoder clip limits (turret) — names end with Turret
    public static int TURRET_ENCODER_HIGH_LIMIT_Turret = 1000;
    public static int TURRET_ENCODER_LOW_LIMIT_Turret  = -1000;

    double target = 0;

    double lastTx = 0;

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
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();
        limelight.pipelineSwitch(0);
        telemetry.setMsTransmissionInterval(11);

        controllerTurret = new PIDController(pTurret, iTurret, dTurret);

        // turret motor settings mirror the original turret opmode
        rotationMotorTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotorTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationMotorTurret.setDirection(DcMotorSimple.Direction.REVERSE);
        rotationMotorTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void PIDFTurretLoop() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();


            lastTx = result.getTx();
        }

        controllerTurret.setPID(pTurret, iTurret, dTurret);
        double error = 0 - lastTx;

        double pidOutput = controllerTurret.calculate(lastTx, 0);
        double feedforward = Math.copySign(kFFTurret, error);
        double motorPower = pidOutput + feedforward;

        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

        rotationMotorTurret.setPower(motorPower);
    }
}
