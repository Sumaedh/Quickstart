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

        public double LONG_VELOCITY = 2000;
        public double SHORT_VELOCITY = 1680;

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
                fShooting = 15;
            } else if (distance == "short") {
                curTargetVelocity = SHORT_VELOCITY;
                fShooting = 15.5;
            } else if (distance == "0") {
                curTargetVelocity = 0;
            } else {
                curTargetVelocity = 0;
            }
        }


        public void setCurTargetVelocity(String distance, int customLength) {
            if (distance == "long") {
                curTargetVelocity = LONG_VELOCITY;
                fShooting = 15;
            } else if (distance == "short") {
                curTargetVelocity = SHORT_VELOCITY;
                fShooting = 15.5;
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

