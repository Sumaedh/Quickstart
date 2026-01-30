package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Sorter {
    DcMotor sorterMotor;

    private PIDController sorterController;

    private double pSorting = 0.007;
    private double iSorting = 0.0;
    private double dSorting = 0.00033;

    public static double kSSorting = 0.02;

    private static final double TICKS_PER_REV = 537.6 * ((double) 10 / 14);

    public static double INCREMENT = TICKS_PER_REV / 6;

    public double SORTER_TOLERANCE = 2.25;

    public double target = 0;

    private ElapsedTime boostTimer = new ElapsedTime();
    private boolean boostActive = false;

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

    public void setSorterTarget(double ticks) {
        target = ticks;
        boostActive = true;
        boostTimer.reset();
    }

    public Runnable setSorterTargetParametric(double ticks) {
        target = ticks;
        return null;
    }

    public boolean SorterAtTarget() {
        if ((sorterMotor.getCurrentPosition() >= (target - SORTER_TOLERANCE)) && (sorterMotor.getCurrentPosition() <= (target+ SORTER_TOLERANCE))) {
            return true;
        } else {
            return false;
        }
    }

    public void PIDFSorterLoop() {

        sorterController.setPID(pSorting, iSorting, dSorting);
        double currentPos = sorterMotor.getCurrentPosition();
        double error = target - currentPos;

        double motorPower;

        // BOOST PHASE: Full power for 0.25s when target changes
        if (boostActive && boostTimer.seconds() < 0.1) {
            // Full speed in the direction we need to go
            // Use Math.signum(error) to get -1 or 1 based on direction
            motorPower = Math.signum(error); // or 0.8 if full power is too violent
        }
        // PID PHASE: Normal control after 0.25s or if boost finished
        else {
            boostActive = false; // Reset flag
            double pidOutput = sorterController.calculate(currentPos, target);
            double staticFF = kSSorting * Math.signum(error);
            motorPower = pidOutput + staticFF;
        }

        sorterMotor.setPower(motorPower);
    }
}
