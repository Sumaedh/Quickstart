package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class SixBallLeaveFarBlue extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    Intake intake = new Intake();
    Lever lever = new Lever();
    Pitch pitch = new Pitch();
    Shooter shooter = new Shooter();
    Sorter sorter = new Sorter();

    // Poses
    private final Pose startPose = new Pose(56.625, 8.75, Math.toRadians(90));
    private final Pose secondPose = new Pose(55.6108, 19.8324, Math.toRadians(90));
    private final Pose scorePose = new Pose(59.535, 16.93, Math.toRadians(137));
    private final Pose pickup3Pose = new Pose(43.681, 35.484, Math.toRadians(180));
    private final Pose pickup3PoseIntake1 = new Pose(37.622, 35.484, Math.toRadians(180));
    private final Pose pickup3PoseIntake2 = new Pose(32.292, 35.484, Math.toRadians(180));
    private final Pose pickup3PoseIntake3 = new Pose(27.17, 35.484, Math.toRadians(180));
    private final Pose endPose = new Pose(60.362, 44.038, Math.toRadians(90));

    private Path startPreload;
    private PathChain score1, alignToBalls, intakeBall1, intakeBall2, intakeBall3, score2, goToEnd;

    public void buildPaths() {
        startPreload = new Path(new BezierLine(startPose, secondPose));
        startPreload.setConstantHeadingInterpolation(secondPose.getHeading());

        score1 = follower.pathBuilder()
                .addPath(new BezierLine(secondPose, scorePose))
                .setLinearHeadingInterpolation(secondPose.getHeading(), scorePose.getHeading())
                .build();

        alignToBalls = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        intakeBall1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, pickup3PoseIntake1))
                .setConstantHeadingInterpolation(pickup3PoseIntake1.getHeading())
                .setBrakingStrength(0.75)
                .build();

        intakeBall2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3PoseIntake1, pickup3PoseIntake2))
                .setConstantHeadingInterpolation(pickup3PoseIntake2.getHeading())
                .setBrakingStrength(0.75)
                .build();

        intakeBall3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3PoseIntake2, pickup3PoseIntake3))
                .setConstantHeadingInterpolation(pickup3PoseIntake3.getHeading())
                .setBrakingStrength(0.75)
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3PoseIntake3, scorePose))
                .setLinearHeadingInterpolation(pickup3PoseIntake3.getHeading(), scorePose.getHeading())
                .build();

        goToEnd = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(startPreload);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    shooter.setCurTargetVelocity("custom", 1630);
                    follower.followPath(score1, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy() && shooter.ShooterAtTarget()) {
                    lever.leverUp();
                    actionTimer.resetTimer();
                    setPathState(3);
                }
                break;

            case 3:
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    lever.leverDown();
                    actionTimer.resetTimer();
                    setPathState(4);
                }
                break;

            case 4:
                sorter.setSorterTarget(179.2);
                if (sorter.SorterAtTarget()) setPathState(5);
                break;

            case 5:
                if (shooter.ShooterAtTarget()) {
                    lever.leverUp();
                    actionTimer.resetTimer();
                    setPathState(6);
                }
                break;

            case 6:
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    lever.leverDown();
                    actionTimer.resetTimer();
                    setPathState(7);
                }
                break;

            case 7:
                sorter.setSorterTarget(358.4);
                if (sorter.SorterAtTarget()) setPathState(8);
                break;

            case 8:
                if (shooter.ShooterAtTarget()) {
                    lever.leverUp();
                    actionTimer.resetTimer();
                    setPathState(9);
                }
                break;

            case 9:
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    lever.leverDown();
                    actionTimer.resetTimer();
                    setPathState(10);
                }
                break;

            case 10:
                sorter.setSorterTarget(448);
                shooter.setCurTargetVelocity("0", 0);
                follower.followPath(alignToBalls, true);
                if (sorter.SorterAtTarget()) {
                    actionTimer.resetTimer();
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    intake.intakeOn();
                    follower.followPath(intakeBall1, 0.3, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    sorter.setSorterTarget(627.2);
                    if (sorter.SorterAtTarget()) {
                        follower.followPath(intakeBall2, 0.3, true);
                        setPathState(13);
                    }
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    sorter.setSorterTarget(806.4);
                    if (sorter.SorterAtTarget()) {
                        follower.followPath(intakeBall3, 0.3, true);
                        setPathState(14);
                    }
                }
                break;

            // -------------------------
            // NEW STATES FOR SCORING + LEAVE
            // -------------------------

            case 14:
                if (!follower.isBusy()) {
                    intake.intakeOff();
                    shooter.setCurTargetVelocity("custom", 1630);
                    follower.followPath(score2, true);
                    setPathState(15);
                }
                break;

            case 15:
                if (!follower.isBusy() && shooter.ShooterAtTarget()) {
                    lever.leverUp();
                    actionTimer.resetTimer();
                    setPathState(16);
                }
                break;

            case 16:
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    lever.leverDown();
                    shooter.setCurTargetVelocity("0", 0);
                    follower.followPath(goToEnd, true);
                    setPathState(17);
                }
                break;

            case 17:
                // Auto complete
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        shooter.PIDFShootingLoop();
        sorter.PIDFSorterLoop();
        pitch.pitchDown();

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        intake.initIntake(hardwareMap);
        lever.initLever(hardwareMap);
        pitch.initPitch(hardwareMap);
        shooter.initShooter(hardwareMap);
        sorter.initSorter(hardwareMap);

        lever.leverDown();
        pitch.pitchUp();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {}
}