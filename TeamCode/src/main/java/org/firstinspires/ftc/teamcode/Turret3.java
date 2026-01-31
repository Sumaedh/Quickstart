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
public class Turret3 {

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

    boolean override = false;

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

    public void setOverride(String set) {
        if (set == "true") {
            override = true;
        } else if (set == "false") {
            override = false;
        }
    }

    // -------- Loop (Auto + TeleOp) --------
    public void PIDFTurretLoop() {

        pid.setPID(p, i, d);

        LLResult result = limelight.getLatestResult();
        boolean hasTarget = (result != null && result.isValid());
        int ticks = rotationMotor.getCurrentPosition();

        // 🚫 If no target, STOP the turret
        if (!hasTarget) {
            rotationMotor.setPower(0);
            return;
        } else if (!hasTarget && override && Math.signum((double) ticks) == -1) {
            rotationMotor.setPower(0.3);
            if (hasTarget) {
                rotationMotor.setPower(0);
                override = false;
            }
        } else if (!hasTarget && override && Math.signum((double) ticks) == 1) {
            rotationMotor.setPower(-0.3);
            if (hasTarget) {
                rotationMotor.setPower(0);
                override = false;
            }
        }

        lastTx = result.getTx();
        double measurement = lastTx;

        double pidOut = pid.calculate(measurement, setpoint);
        double error = setpoint - measurement;

        double motorPower = pidOut + Math.copySign(kF, error);

        if (!override) {
            motorPower = Math.max(-1.0, Math.min(1.0, motorPower));
        }

        if (ticks >= ENCODER_HIGH_LIMIT && motorPower > 0) motorPower = 0;
        if (ticks <= ENCODER_LOW_LIMIT  && motorPower < 0) motorPower = 0;

        rotationMotor.setPower(motorPower);
    }
}