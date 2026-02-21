package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Sorter {

    private DcMotor sorterMotor;
    private PIDController sorterController;

    private final double pSorting = 0.004;
    private final double iSorting = 0.0;
    private final double dSorting = 0.00027;

    public static double kSSorting = 0.034;

    private static final double TICKS_PER_REV = 537.6 * (10.0 / 14.0);
    public static double INCREMENT = TICKS_PER_REV / 6.0;

    public double SORTER_TOLERANCE = 2.25;

    private double target = 0;

    public void initSorter(HardwareMap hwMap) {
        sorterController = new PIDController(pSorting, iSorting, dSorting);
        sorterMotor = hwMap.get(DcMotor.class, "sorterMotor");

        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setSorterTarget(double ticks) {
        target = ticks;
    }

    public void setSorterTargetParametric(double ticks) {
        target = ticks;
    }

    public boolean SorterAtTarget() {
        double pos = sorterMotor.getCurrentPosition();
        return Math.abs(pos - target) <= SORTER_TOLERANCE;
    }

    public boolean isBusy() {
        return !SorterAtTarget();
    }

    public void update() {
        sorterController.setPID(pSorting, iSorting, dSorting);

        double currentPos = sorterMotor.getCurrentPosition();
        double error = target - currentPos;

        double pidOutput = sorterController.calculate(currentPos, target);
        double staticFF = kSSorting * Math.signum(error);

        double motorPower = pidOutput + staticFF;

        // clamp
        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

        sorterMotor.setPower(motorPower);
    }
}
