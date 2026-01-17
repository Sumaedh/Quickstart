package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class LeaveAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private boolean justCompletedPath = false;
    private boolean sorterWaiting = false;
    private boolean shootingSequenceStarted = false;

    private ShooterSorterLeverFSM fsm;

    public static double WAIT_FOR_POCKET = 0.5;
    public static double INTAKE_BALL_WAIT = 0.5;
    public static double SHOOTING_SEQUENCE_WAIT = 0;

    // Poses
    private final Pose startPose = new Pose(18.20540540540541, 115.97837837837838, Math.toRadians(90));
    private final Pose secondPose = new Pose(18.20540540540541, 102.7459459459459, Math.toRadians(90));
    private final Pose scorePose = new Pose(60, 20, Math.toRadians(160));
    private final Pose pickup3Pose = new Pose(41.124, 29.584, Math.toRadians(180));
    private final Pose pickup3PoseIntake1 = new Pose(36.622, 29.584, Math.toRadians(180));
    private final Pose pickup3PoseIntake2 = new Pose(31.292, 29.584, Math.toRadians(180));
    private final Pose pickup3PoseIntake3 = new Pose(26.470, 29.584, Math.toRadians(180));
    private final Pose endPose = new Pose(55.362, 44.038, Math.toRadians(160));

    private Path startPreload;
    private PathChain score1, alignToBalls, intakeBall1, intakeBall2, intakeBall3, score2, goToEnd;

    public void buildPaths() {
        startPreload = new Path(new BezierLine(startPose, secondPose));
        startPreload.setConstantHeadingInterpolation(secondPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                // Drive from start to secondPose
                follower.followPath(startPreload);
                setPathState(1);
                break;
            case 1:
                // Parked / done
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        justCompletedPath = false;
        sorterWaiting = false;
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        fsm.run(opmodeTimer.getElapsedTimeSeconds(), shootingSequenceStarted);
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {}
}
