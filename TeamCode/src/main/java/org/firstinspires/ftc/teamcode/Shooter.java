package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Shooter {
    DcMotorEx shootingMotor;

    public double fShooting = 15;
    public double pShooting = 250;
    public double curTargetVelocity = 0;

    public double LONG_VELOCITY = 1680;
    public double SHORT_VELOCITY = 1420;

    public double SHOOTER_TOLERANCE = 20;

    public void initShooter(HardwareMap hwMap) {
        shootingMotor = hwMap.get(DcMotorEx.class, "shootingMotor");

        PIDFCoefficients pidfShooting =
                new PIDFCoefficients(pShooting, 0, 0, fShooting);
        shootingMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, pidfShooting
        );

        shootingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setCurTargetVelocityParametric(String distance) {
        if (distance == "long") {
            curTargetVelocity = LONG_VELOCITY;
        } else if (distance == "short") {
            curTargetVelocity = SHORT_VELOCITY;
        } else if (distance == "0") {
            curTargetVelocity = 0;
        } else {
            curTargetVelocity = 0;
        }
    }


    public void setCurTargetVelocity(String distance) {
        if (distance == "long") {
            curTargetVelocity = LONG_VELOCITY;
        } else if (distance == "short") {
            curTargetVelocity = SHORT_VELOCITY;
        } else if (distance == "0") {
            curTargetVelocity = 0;
        } else {
            curTargetVelocity = 0;
        }
    }

    public boolean ShooterAtTarget() {
        if ((shootingMotor.getVelocity() >= (curTargetVelocity - SHOOTER_TOLERANCE)) && (shootingMotor.getVelocity() <= (curTargetVelocity + SHOOTER_TOLERANCE))) {
            return true;
        } else {
            return false;
        }
    }

    public void PIDFShootingLoop() {
        PIDFCoefficients newPidf = new PIDFCoefficients(pShooting, 0, 0, fShooting);
        shootingMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPidf);
        shootingMotor.setVelocity(curTargetVelocity);
    }


    public void setTargetVekocity(int i) {

    }
}
