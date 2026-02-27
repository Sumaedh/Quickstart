package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretMotor {
    DcMotorEx rotationMotor;

    private double kP = 0.0001;
    private double kD = 0.000;

    private double goalX = 0;
    private double lastError = 0;
    private double angleTolerance = 0;

    private final double MAX_POWER = 0.8;

    private double power = 0;

    private final ElapsedTime timer = new ElapsedTime();

    public void initLimelight(HardwareMap hwMap, Telemetry telemetry) {
        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setkP( double newKP) {
        kP = newKP;
    }

    public double getkP() {
        return kP;
    }

    public void setkD( double newKD) {
        kD = newKD;
    }

    public double getkD() {
        return kD;
    }

    public void resetTimer() {
        timer.reset();
    }

    public void update () { // put in a input for ID as parameter
        double deltaTime = timer.seconds();
        timer.reset();

        if (false) {   // if limelight object is null
            rotationMotor.setPower(0);
            lastError = 0;
            return;
        }
    }
}
