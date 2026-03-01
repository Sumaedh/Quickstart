package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Shooter {

    private DcMotorEx shootingMotor;


    public double fShooting = 16.5;
    public double pShooting = 400;


    public double curTargetVelocity = 0;


    public double LONG_VELOCITY = 1680;
    public double SHORT_VELOCITY = 1420;

    public double SHOOTER_TOLERANCE = 20;


    // Regression coefficients replace with Desmos values
    public double A = 0.0;   // quadratic coefficient
    public double B = 0.0;   // linear coefficient
    public double C = 0.0;   // constant term


    public void initShooter(HardwareMap hwMap) {
        shootingMotor = hwMap.get(DcMotorEx.class, "shootingMotor");

        PIDFCoefficients pidfShooting =
                new PIDFCoefficients(pShooting, 0, 0, fShooting);
        shootingMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, pidfShooting
        );

        shootingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shootingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void setCurTargetVelocity(String distance, double custom) {
        if (distance.equals("long")) {
            curTargetVelocity = LONG_VELOCITY;
        } else if (distance.equals("short")) {
            curTargetVelocity = SHORT_VELOCITY;
        } else if (distance.equals("0")) {
            curTargetVelocity = 0;
        } else if (distance.equals("custom") && custom != 0) {
            curTargetVelocity = custom;
        }
    }

    public void setCurTargetVelocityDynamic(double distanceInches) {
        curTargetVelocity = getVelocityFromDistance(distanceInches);
    }


    // y = A·x² + B·x + C

    public double getVelocityFromDistance(double d) {
        double result = A*d*d + B*d + C;
        return clamp(result, 0, 1650);
    }


    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }


    public boolean shooterAtTarget() {
        double vel = shootingMotor.getVelocity();
        return vel >= (curTargetVelocity - SHOOTER_TOLERANCE)
                && vel <= (curTargetVelocity + SHOOTER_TOLERANCE);
    }

    public void PIDFShootingLoop() {
        PIDFCoefficients newPidf =
                new PIDFCoefficients(pShooting, 0, 0, fShooting);
        shootingMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, newPidf
        );

        shootingMotor.setVelocity(curTargetVelocity);
    }
}
