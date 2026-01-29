package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class RedAutoFarTurret2 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    Intake intake = new Intake();
    Lever lever = new Lever();
    Pitch pitch = new Pitch();
    Shooter shooter = new Shooter();
    Sorter sorter = new Sorter();
    Turret turret = new Turret();

    // -----------------------------
    // Poses (coordinates unchanged)
    // -----------------------------
    private final Pose startPose        = new Pose(87.375, 8.75, Math.toRadians(90));
    private final Pose secondPose       = new Pose(88.3892, 14, Math.toRadians(90));
    private final Pose scorePose        = new Pose(84.465, 14, Math.toRadians(57));

    private final Pose pickupLowPose    = new Pose(96, 36, Math.toRadians(0));
    private final Pose pickupLowIntake3 = new Pose(120, 36, Math.toRadians(0));

    private final Pose pickupMidPose    = new Pose(96, 60, Math.toRadians(0));
    private final Pose pickupMidIntake3 = new Pose(120, 60, Math.toRadians(0));

    private final Pose endPose          = new Pose(83.638, 44.038, Math.toRadians(90));

    private Path startPreload;
    private PathChain score1, alignToLow, intakeLow, scoreFromLow;
    private PathChain alignToMid, intakeMid, scoreFromMid, goToEnd;


    // Helps Heading
    private double headingAlong(Pose from, Pose to) {
        return Math.atan2(to.getY() - from.getY(), to.getX() - from.getX());
    }


    public void buildPaths() {


        double hStartToSecond = headingAlong(startPose, secondPose);
        startPreload = new Path(new BezierLine(startPose, secondPose));
        startPreload.setConstantHeadingInterpolation(hStartToSecond);

         double hSecondToScoreMove = headingAlong(secondPose, scorePose);
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(secondPose, scorePose))
                .addParametricCallback(0.01, () -> shooter.setCurTargetVelocityParametric("long"))
                .setLinearHeadingInterpolation(hSecondToScoreMove, scorePose.getHeading())
                .build();


        double hScoreToLow = headingAlong(scorePose, pickupLowPose);
        alignToLow = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickupLowPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), hScoreToLow)
                .build();


        double hLowIntake = headingAlong(pickupLowPose, pickupLowIntake3);
        intakeLow = follower.pathBuilder()
                .addPath(new BezierLine(pickupLowPose, pickupLowIntake3))
                .addParametricCallback(0.5, () -> sorter.setSorterTargetParametric(627.2))
                .addParametricCallback(0.81, () -> sorter.setSorterTargetParametric(806.4))
                .setConstantHeadingInterpolation(hLowIntake)
                .setBrakingStrength(0.5)
                .build();


        double hLowToScoreMove = headingAlong(pickupLowIntake3, scorePose);
        scoreFromLow = follower.pathBuilder()
                .addPath(new BezierLine(pickupLowIntake3, scorePose))
                .addParametricCallback(0.01, () -> shooter.setCurTargetVelocity("long"))
                .setLinearHeadingInterpolation(hLowToScoreMove, scorePose.getHeading())
                .build();


        double hScoreToMid = headingAlong(scorePose, pickupMidPose);
        alignToMid = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickupMidPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), hScoreToMid)
                .build();


        double hMidIntake = headingAlong(pickupMidPose, pickupMidIntake3);
        intakeMid = follower.pathBuilder()
                .addPath(new BezierLine(pickupMidPose, pickupMidIntake3))
                .addParametricCallback(0.5, () -> sorter.setSorterTargetParametric(1523.3))
                .addParametricCallback(0.81, () -> sorter.setSorterTargetParametric(1702.4))
                .setConstantHeadingInterpolation(hMidIntake)
                .setBrakingStrength(0.5)
                .build();


        double hMidToScoreMove = headingAlong(pickupMidIntake3, scorePose);
        scoreFromMid = follower.pathBuilder()
                .addPath(new BezierLine(pickupMidIntake3, scorePose))
                .addParametricCallback(0.01, () -> shooter.setCurTargetVelocity("long"))
                .setLinearHeadingInterpolation(hMidToScoreMove, scorePose.getHeading())
                .build();


        double hScoreToEndMove = headingAlong(scorePose, endPose);
        goToEnd = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(hScoreToEndMove, endPose.getHeading())
                .build();
    }


    // State helpers
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    private boolean leverCycleDone(double wait) {
        return actionTimer.getElapsedTimeSeconds() > wait;
    }


    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(startPreload);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(score1, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy() && shooter.ShooterAtTarget() && turret.turretAtTarget()) {
                    lever.leverUp();
                    actionTimer.resetTimer();
                    setPathState(3);
                }
                break;

            case 3:
                if (leverCycleDone(0.3)) {
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
                if (leverCycleDone(0.3)) {
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
                if (leverCycleDone(0.3)) {
                    lever.leverDown();
                    actionTimer.resetTimer();
                    setPathState(10);
                }
                break;

            case 10:
                sorter.setSorterTarget(448);
                shooter.setCurTargetVelocity("0");
                follower.followPath(alignToLow, true);
                if (sorter.SorterAtTarget()) setPathState(11);
                break;

            case 11:
                if (!follower.isBusy()) {
                    intake.intakeOn();
                    follower.followPath(intakeLow, 0.32, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    intake.intakeOff();
                    sorter.setSorterTarget(896);
                    follower.followPath(scoreFromLow, true);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy() && shooter.ShooterAtTarget() && turret.turretAtTarget()) {
                    lever.leverUp();
                    actionTimer.resetTimer();
                    setPathState(14);
                }
                break;

            case 14:
                if (leverCycleDone(0.3)) {
                    lever.leverDown();
                    actionTimer.resetTimer();
                    setPathState(15);
                }
                break;

            case 15:
                sorter.setSorterTarget(1075.2);
                if (sorter.SorterAtTarget()) setPathState(16);
                break;

            case 16:
                if (shooter.ShooterAtTarget()) {
                    lever.leverUp();
                    actionTimer.resetTimer();
                    setPathState(17);
                }
                break;

            case 17:
                if (leverCycleDone(0.3)) {
                    lever.leverDown();
                    actionTimer.resetTimer();
                    setPathState(18);
                }
                break;

            case 18:
                sorter.setSorterTarget(1254.4);
                if (sorter.SorterAtTarget()) setPathState(19);
                break;

            case 19:
                if (shooter.ShooterAtTarget()) {
                    lever.leverUp();
                    actionTimer.resetTimer();
                    setPathState(20);
                }
                break;

            case 20:
                if (leverCycleDone(0.3)) {
                    lever.leverDown();
                    actionTimer.resetTimer();
                    setPathState(21);
                }
                break;

            case 21:
                sorter.setSorterTarget(1344);
                shooter.setCurTargetVelocity("0");
                follower.followPath(alignToMid, true);
                if (sorter.SorterAtTarget()) setPathState(22);
                break;

            case 22:
                if (!follower.isBusy()) {
                    intake.intakeOn();
                    follower.followPath(intakeMid, 0.32, true);
                    setPathState(23);
                }
                break;

            case 23:
                if (!follower.isBusy()) {
                    intake.intakeOff();
                    sorter.setSorterTarget(1792);
                    follower.followPath(scoreFromMid, true);
                    setPathState(24);
                }
                break;

            case 24:
                if (!follower.isBusy() && shooter.ShooterAtTarget() && turret.turretAtTarget()) {
                    lever.leverUp();
                    actionTimer.resetTimer();
                    setPathState(25);
                }
                break;

            case 25:
                if (leverCycleDone(0.3)) {
                    lever.leverDown();
                    actionTimer.resetTimer();
                    setPathState(26);
                }
                break;

            case 26:
                sorter.setSorterTarget(1971.2);
                if (sorter.SorterAtTarget()) setPathState(27);
                break;

            case 27:
                if (shooter.ShooterAtTarget()) {
                    lever.leverUp();
                    actionTimer.resetTimer();
                    setPathState(28);
                }
                break;

            case 28:
                if (leverCycleDone(0.3)) {
                    lever.leverDown();
                    actionTimer.resetTimer();
                    setPathState(29);
                }
                break;

            case 29:
                sorter.setSorterTarget(2150.4);
                if (sorter.SorterAtTarget()) setPathState(30);
                break;

            case 30:
                if (shooter.ShooterAtTarget()) {
                    lever.leverUp();
                    actionTimer.resetTimer();
                    setPathState(31);
                }
                break;

            case 31:
                if (leverCycleDone(0.3)) {
                    lever.leverDown();
                    actionTimer.resetTimer();
                    setPathState(32);
                }
                break;
        }
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
        turret.initTurret(hardwareMap);

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
    public void loop() {
        shooter.PIDFShootingLoop();
        sorter.PIDFSorterLoop();
        pitch.pitchDown();
        turret.PIDFTurretLoop();

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {}
}
