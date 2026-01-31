package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Turret2 {

    // -------- Tunables --------
    public static double p = 0.03;
    public static double i = 0.0;
    public static double d = 0.0002;

    public static double kF = 0.02;

    public static int ENCODER_LOW_LIMIT  = -1000;
    public static int ENCODER_HIGH_LIMIT =  1000;

    // -------- Hardware --------
    private DcMotor rotationMotor;
    private Limelight3A limelight;
    double setpoint = 0.0;

    // -------- System --------
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    // -------- Control --------
    private PIDController pid;
    private double lastTx = 0.0;

    // -------- Init --------
    public void initTurret(HardwareMap hwMap, Telemetry telemetry) {

        this.hardwareMap = hwMap;
        this.telemetry = telemetry;

        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        pid = new PIDController(p, i, d);
    }

    public void pipeline(int line) {
        limelight.pipelineSwitch(line);
        if (line == 1) {
            setpoint = -1;
        }
    }

    // -------- Loop (Auto + TeleOp) --------
    public void PIDFTurretLoop() {

        LLResult result = limelight.getLatestResult();

        int pos = rotationMotor.getCurrentPosition();
        double offset = result.getTx();
        pos += (offset * 7.5 / 384.5);
        if (pos > 1000) {
            pos = 1000;
        }
        else if (pos < -1000) {
            pos = -1000;
        }
        rotationMotor.setTargetPosition(pos);
        rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotationMotor.setPower(0.2);
    }
}