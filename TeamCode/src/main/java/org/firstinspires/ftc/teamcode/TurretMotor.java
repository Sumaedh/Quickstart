package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretMotor {
    DcMotorEx rotationMotor;

    private double kP = -0.02;
    private double kD = 0.000;

    public double goalX = 2;
    private double lastError = 0;
    private double angleTolerance = 0.2;

    private final double MAX_POWER = 0.8;

    private double power = 0;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime switchbackTimer = new ElapsedTime();


    public double MIN_ENCODER_LIMIT = -804;

    public double MAX_ENCODER_LIMIT = 54;

    boolean switchback = true;
    double SWITCHBACK_POWER = 0;

    public void initLimelight(HardwareMap hwMap) {
        rotationMotor = hwMap.get(DcMotorEx.class, "rotationMotor");
        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
//54, -804
    // huwfhhewf
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

    public void update (LLResult result) { // put in a input for ID as parameter
        double deltaTime = timer.seconds();
        timer.reset();

        /*
        if ((result == null) && (MIN_ENCODER_LIMIT >= rotationMotor.getCurrentPosition() || MAX_ENCODER_LIMIT <= rotationMotor.getCurrentPosition())) {
            if (MIN_ENCODER_LIMIT >= rotationMotor.getCurrentPosition()) {
                SWITCHBACK_POWER = 0.6 * -1 * Math.signum(MIN_ENCODER_LIMIT);
                rotationMotor.setPower(SWITCHBACK_POWER);
                lastError = 0;
                return;
            } else {
                SWITCHBACK_POWER = 0.6 * -1 * Math.signum(MAX_ENCODER_LIMIT);
                rotationMotor.setPower(SWITCHBACK_POWER);
                lastError = 0;
                return;
            }
        }
         */


        if (result == null) {
            rotationMotor.setPower(0);
            lastError = 0;
            return;
        }

        double error = goalX - result.getTx(); // - where we are (input)
        double pTerm = error * kP;

        double dTerm = 0;
        if (deltaTime > 0) {
            dTerm = ((error - lastError) / deltaTime) * kD;
        }

        power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);

        if (Math.abs(error) < angleTolerance) {
            power = 0;
        } else if ((MIN_ENCODER_LIMIT >= rotationMotor.getCurrentPosition() || MAX_ENCODER_LIMIT <= rotationMotor.getCurrentPosition())) {
            if (MIN_ENCODER_LIMIT >= rotationMotor.getCurrentPosition()  && Math.signum(power) == -1) {
                power = 0;
                lastError = 0;
                return;
            } else {
                SWITCHBACK_POWER = 0.6 * -1 * Math.signum(MAX_ENCODER_LIMIT);
                rotationMotor.setPower(SWITCHBACK_POWER);
                lastError = 0;
                return;

            }
        }
        else {
            power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
        }


        rotationMotor.setPower(power);
        lastError = error;
    }
}
