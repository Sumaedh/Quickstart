package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class LimelightAlignmentTest extends OpMode {


    Limelight3A limelight;
    TurretMotor turret = new TurretMotor();

    public static double currentKp = -0.02;

    public static double currentKd = 0;

    public static double currentGoalx = 2;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turret.initLimelight(hardwareMap);
        limelight.start();
        limelight.pipelineSwitch(0);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        turret.resetTimer();
    }

    // -0.02 for p, 2 for goalx

    @Override
    public void loop() {

        turret.setkP(currentKp);
        turret.setkD(currentKd);
        turret.goalX = currentGoalx;

        LLResult limelightResult = limelight.getLatestResult();

        turret.update(limelightResult);

        if (limelightResult != null) {
            telemetry.addData("Tx", limelightResult.getTx());
        } else {
            telemetry.addLine("nothing detected");
        }

        telemetry.addData("goalX", turret.goalX);
        telemetry.addData("motor ticks", turret.rotationMotor.getCurrentPosition());
        telemetry.update();
    }
}
