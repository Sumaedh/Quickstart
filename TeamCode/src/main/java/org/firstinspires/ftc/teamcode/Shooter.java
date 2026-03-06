package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

    public class Shooter {
        DcMotorEx shootingMotor;

        public double fShooting = 21.5;
        public double pShooting = 500;
        public double curTargetVelocity = 0;

        public double LONG_VELOCITY = 1470;
        public double SHORT_VELOCITY = 1200;

        public double SHOOTER_TOLERANCE = 50;

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

        public void setCurTargetVelocityParametric(String distance, int customLength) {
            if (distance == "long") {
                curTargetVelocity = LONG_VELOCITY;
            } else if (distance == "short") {
                curTargetVelocity = SHORT_VELOCITY;
            } else if (distance == "0" && customLength != 0) {
                curTargetVelocity = customLength;
            } else {
                curTargetVelocity = 0;
            }
        }


        public void setCurTargetVelocity(String distance, int customLength) {
            if (distance == "long") {
                curTargetVelocity = LONG_VELOCITY;
            } else if (distance == "short") {
                curTargetVelocity = SHORT_VELOCITY;
            } else if (distance == "0" && customLength != 0) {
                curTargetVelocity = customLength;
            } else {
                curTargetVelocity = 0;
            }
        }

        public boolean shooterAtTarget() {
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

    }

