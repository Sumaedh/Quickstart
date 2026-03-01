package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.seattlesolvers.solverslib.controller.PIDController;

@TeleOp(name="AutoSorterTest")
public class AutoSorterTest extends LinearOpMode {

    enum State {
        WAITING_FOR_BALL,
        SORTING,
        READY_FOR_NEXT
    }

    DistanceSensor dsRight, dsMid, dsLeft;
    DcMotor sorterMotor;
    DcMotor intakeMotor;

    State currentState = State.WAITING_FOR_BALL;

    double baselineRight, baselineMid, baselineLeft;
    final double DROP_PERCENT = 0.75;

    int ballCount = 0;

    public static double kSSorting = 0.034;

    private static final double TICKS_PER_REV = 537.6 * ((double) 10 / 14);
    public static double INCREMENT = TICKS_PER_REV / 6;

    private double target = 0;
    private int turns = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        dsRight = hardwareMap.get(DistanceSensor.class, "dsRight");
        dsMid   = hardwareMap.get(DistanceSensor.class, "dsMid");
        dsLeft  = hardwareMap.get(DistanceSensor.class, "dsLeft");

        sorterMotor = hardwareMap.get(DcMotor.class, "sorterMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sorterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        sorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        double pSorting = 0.004;
        double iSorting = 0.0;
        double dSorting = 0.00027;
        PIDController sorterController = new PIDController(pSorting, iSorting, dSorting);

        baselineRight = dsRight.getDistance(DistanceUnit.MM);
        baselineMid   = dsMid.getDistance(DistanceUnit.MM);
        baselineLeft  = dsLeft.getDistance(DistanceUnit.MM );

        telemetry.addLine("Calibrated baselines:");
        telemetry.addData("Right", baselineRight);
        telemetry.addData("Mid", baselineMid);
        telemetry.addData("Left", baselineLeft);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean rightBall = dsRight.getDistance(DistanceUnit.MM) < baselineRight * DROP_PERCENT;
            boolean midBall   = dsMid.getDistance(DistanceUnit.MM) < baselineMid * DROP_PERCENT;

            if (ballCount < 3) intakeMotor.setPower(1.0);
            else intakeMotor.setPower(0);

            switch (currentState) {

                case WAITING_FOR_BALL:
                    if (rightBall && ballCount < 3) {
                        turns += 1;
                        target = (turns % 6) * INCREMENT;
                        currentState = State.SORTING;
                    }
                    break;

                case SORTING:
                    sorterController.setPID(pSorting, iSorting, dSorting);
                    double currentPos = sorterMotor.getCurrentPosition();
                    double error = target - currentPos;

                    double pidOutput = sorterController.calculate(currentPos, target);
                    double staticFF = kSSorting * Math.signum(error);
                    double motorPowerSorter = pidOutput + staticFF;

                    sorterMotor.setPower(motorPowerSorter);

                    if (!midBall) {
                        ballCount++;
                        currentState = State.READY_FOR_NEXT;
                    }
                    break;

                case READY_FOR_NEXT:
                    sorterMotor.setPower(0);

                    if (ballCount < 3 && rightBall) {
                        turns += 1;
                        target = (turns % 6) * INCREMENT;
                        currentState = State.SORTING;
                    }
                    break;
            }

            telemetry.addData("State", currentState);
            telemetry.addData("Ball Count", ballCount);
            telemetry.addData("Sorter Target", target);
            telemetry.addData("Sorter Pos", sorterMotor.getCurrentPosition());
            telemetry.addData("Right", dsRight.getDistance(DistanceUnit.MM));
            telemetry.addData("Mid", dsMid.getDistance(DistanceUnit.MM));
            telemetry.addData("Left", dsLeft.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
