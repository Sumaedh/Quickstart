package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Sorter {
    DcMotor sorterMotor;

    private PIDController sorterController;

    private double pSorting = 0.0029;
    private double iSorting = 0.0;
    private double dSorting = 0.00017;

    public static double kSSorting = 0.02;

    private static final double TICKS_PER_REV = 537.6;

    public static double INCREMENT = TICKS_PER_REV / 6;

    public double SORTER_TOLERANCE = 5;

    public double target = 0;

    public void initSorter(HardwareMap hwMap) {
        sorterController = new PIDController(pSorting,iSorting,dSorting);
        sorterMotor = hwMap.get(DcMotor.class, "sorterMotor");
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sorterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void turnSorter(int turns) {
        for (int i = 0; i < turns; i++) {
            target += INCREMENT;
        }
    }

    public boolean SorterAtTarget() {
        if ((sorterMotor.getCurrentPosition() >= (target - SORTER_TOLERANCE)) && (sorterMotor.getCurrentPosition() <= (target+ SORTER_TOLERANCE))) {
            return true;
        } else {
            return false;
        }
    }

    public void PIDFSorterLoop() {

        sorterController.setPID(pSorting,iSorting,dSorting);
        double currentPos = sorterMotor.getCurrentPosition();
        double pidOutput = sorterController.calculate(currentPos, target);
        double error = target - currentPos;
        double staticFF = kSSorting * Math.signum(error);

        sorterMotor.setPower(pidOutput + staticFF);
    }
}
