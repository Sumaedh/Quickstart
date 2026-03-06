package org.firstinspires.ftc.teamcode.marrow;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Lever;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.Sorter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Auto Close Safe (Marrow)", group = "Main")
public class BlueAutoCloseSafeMarrow extends OpMode {

    private final Intake intake = new Intake();
    private final Lever lever = new Lever();
    private final Shooter shooter = new Shooter();
    private final Sorter sorter = new Sorter();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private final Timer intakeWaitTimer = new Timer();

    private int pathState = 0;

    private final Pose startPose = new Pose(34.138, 135.25, Math.toRadians(270));
    private final Pose scorePose = new Pose(51.7, 92.15, Math.toRadians(136));

    private final Pose pickupLowIntake3 = new Pose(51.7, 92.15, Math.toRadians(180));
    private final Pose pickupMidIntake3 = new Pose(23.072, 59.724, Math.toRadians(180));

    private final Pose pickupHighIntake3 = new Pose(16, 36 - 3.25, Math.toRadians(180));
    private final Pose endPose = new Pose(52, 54, Math.toRadians(180));

    private PathChain startPreload;
    private PathChain score1;
    private PathChain pickupLowChain, scoreFromLow;
    private PathChain pickupMidChain, scoreFromMid;
    private PathChain pickupLastChain, scoreFromLast;
    private PathChain goToEnd;

    private static final double AUTO_DURATION  = 30.0;
    private static final double SAFETY_MARGIN  = 2.0;

    public void buildPaths() {
        startPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();


        score1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setHeadingConstraint(4)
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.01, () -> shooter.setCurTargetVelocityParametric("short", 0))
                .build();

        pickupLowChain = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickupLowIntake3))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupLowIntake3.getHeading())
                .setBrakingStrength(0.5)
                .addParametricCallback(0.445, () -> sorter.setSorterTargetParametric(3 * sorter.INCREMENT))
                .addParametricCallback(0.68, () -> sorter.setSorterTargetParametric(4 * sorter.INCREMENT))
                .build();


        scoreFromLow = follower.pathBuilder()
                .addPath(new BezierLine(pickupLowIntake3, scorePose))
                .setLinearHeadingInterpolation(pickupLowIntake3.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.01, () -> shooter.setCurTargetVelocityParametric("long", 0))
                .build();


        pickupMidChain = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickupMidIntake3))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupMidIntake3.getHeading())
                .setBrakingStrength(0.5)
                .addParametricCallback(0.34, () -> sorter.setSorterTargetParametric(7 * sorter.INCREMENT))
                .addParametricCallback(0.5, () -> sorter.setSorterTargetParametric(8 * sorter.INCREMENT))
                .build();


        scoreFromMid = follower.pathBuilder()
                .addPath(new BezierLine(pickupMidIntake3, scorePose))
                .setLinearHeadingInterpolation(pickupMidIntake3.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.01, () -> shooter.setCurTargetVelocity("long", 0))
                .setHeadingConstraint(4)
                .build();


        pickupLastChain = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickupHighIntake3))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupHighIntake3.getHeading())
                .setBrakingStrength(0.5)
                .addParametricCallback(0.78, () -> {
                    intake.intakeOn();
                    intakeWaitTimer.resetTimer();
                })
                .build();


        scoreFromLast = follower.pathBuilder()
                .addPath(new BezierLine(pickupHighIntake3, scorePose))
                .setLinearHeadingInterpolation(pickupHighIntake3.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.01, () -> shooter.setCurTargetVelocity("long", 0))
                .build();


        goToEnd = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();
    }

    @Override
    public void init() {
        pathTimer  = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        intake.initIntake(hardwareMap);
        lever.initLever(hardwareMap);
        shooter.initShooter(hardwareMap);
        sorter.initSorter(hardwareMap);

        buildPaths();

        follower.setStartingPose(startPose);

        lever.leverDown();
    }

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    private double getEstimatedRemainingTimeFromState(int state) {
        switch (state) {
            case 0: case 1: case 2: case 3: case 4: case 5: case 6: case 7: case 8: case 9:
                return 8.0;
            case 10: case 11: case 12: case 13: case 14: case 15: case 16: case 17: case 18: case 19:
            case 20: case 21: case 22: case 23: case 24: case 25: case 26: case 27: case 28: case 29:
                return 10.0;
            default:
                return 3.0;
        }
    }

    private boolean shouldBailToPark() {
        double elapsed   = opmodeTimer.getElapsedTimeSeconds();
        double remaining = AUTO_DURATION - elapsed;
        double required  = getEstimatedRemainingTimeFromState(pathState);
        return remaining < SAFETY_MARGIN && remaining < required;
    }

    private void bailToEndZone() {
        follower.followPath(goToEnd, true);
        pathState = 100;
    }

    public void autonomousPathUpdate() {
        if (pathState <= 32 && shouldBailToPark()) {
            bailToEndZone();
            return;
        }

        switch (pathState) {

            case 0:
                follower.followPath(startPreload, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(score1, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    if (shooter.shooterAtTarget()) {
                        lever.leverUp();
                        actionTimer.resetTimer();
                        setPathState(3);
                    }
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
                sorter.setSorterTarget(1 * Sorter.INCREMENT);
                if (sorter.sorterAtTarget()) setPathState(5);
                break;

            case 5:
                if (shooter.shooterAtTarget()) {
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
                sorter.setSorterTarget(2 * Sorter.INCREMENT);
                if (sorter.sorterAtTarget()) setPathState(8);
                break;

            case 8:
                if (shooter.shooterAtTarget()) {
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
                shooter.setCurTargetVelocity("0", 0);
                follower.followPath(pickupLowChain, true);
                if (sorter.sorterAtTarget()) {
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    intake.intakeOff();
                    follower.followPath(scoreFromLow, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    if (shooter.shooterAtTarget()) {
                        lever.leverUp();
                        actionTimer.resetTimer();
                        setPathState(13);
                    }
                }
                break;

            case 13:
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    lever.leverDown();
                    actionTimer.resetTimer();
                    setPathState(14);
                }
                break;

            case 14:
                sorter.setSorterTarget(5 * Sorter.INCREMENT);
                if (sorter.sorterAtTarget()) {
                    setPathState(15);
                }
                break;

            case 15:
                if (shooter.shooterAtTarget()) {
                    lever.leverUp();
                    actionTimer.resetTimer();
                    setPathState(16);
                }
                break;

            case 16:
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    lever.leverDown();
                    actionTimer.resetTimer();
                    setPathState(17);
                }
                break;

            case 17:
                sorter.setSorterTarget(6 * Sorter.INCREMENT);
                if (sorter.sorterAtTarget()) {
                    setPathState(18);
                }
                break;

            case 18:
                if (shooter.shooterAtTarget()) {
                    lever.leverUp();
                    actionTimer.resetTimer();
                    setPathState(19);
                }
                break;

            case 19:
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    lever.leverDown();
                    actionTimer.resetTimer();
                    setPathState(20);
                }
                break;

            case 20:
                shooter.setCurTargetVelocity("0", 0);
                follower.followPath(pickupMidChain, true);
                if (sorter.sorterAtTarget()) {
                    setPathState(21);
                }
                break;

            case 21:
                if (!follower.isBusy()) {
                    intake.intakeOff();
                    follower.followPath(scoreFromMid, true);
                    setPathState(22);
                }
                break;

            case 22:
                if (!follower.isBusy()) {
                    if (shooter.shooterAtTarget()) {
                        lever.leverUp();
                        actionTimer.resetTimer();
                        setPathState(23);
                    }
                }
                break;

            case 23:
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    lever.leverDown();
                    actionTimer.resetTimer();
                    setPathState(24);
                }
                break;

            case 24:
                sorter.setSorterTarget(9 * Sorter.INCREMENT);
                if (sorter.sorterAtTarget()) {
                    setPathState(25);
                }
                break;

            case 25:
                if (shooter.shooterAtTarget()) {
                    lever.leverUp();
                    actionTimer.resetTimer();
                    setPathState(26);
                }
                break;

            case 26:
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    lever.leverDown();
                    actionTimer.resetTimer();
                    setPathState(27);
                }
                break;

            case 27:
                sorter.setSorterTarget(10 * Sorter.INCREMENT);
                if (sorter.sorterAtTarget()) {
                    setPathState(28);
                }
                break;

            case 28:
                if (shooter.shooterAtTarget()) {
                    lever.leverUp();
                    actionTimer.resetTimer();
                    setPathState(29);
                }
                break;

            case 29:
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    lever.leverDown();
                    actionTimer.resetTimer();
                    setPathState(30);
                }
                break;

            case 30:
                shooter.setCurTargetVelocity("0", 0);
                follower.followPath(pickupLastChain, true);
                setPathState(31);
                break;

            case 31:
                if (!follower.isBusy()) {
                    intake.intakeOff();
                    follower.followPath(scoreFromLast, true);
                    setPathState(32);
                }
                break;

            case 32:
                if (!follower.isBusy()) {
                    if (shooter.shooterAtTarget()) {
                        lever.leverUp();
                        actionTimer.resetTimer();
                        setPathState(33);
                    }
                }
                break;

            case 33:
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    lever.leverDown();
                    actionTimer.resetTimer();
                    setPathState(34);
                }
                break;

            case 34:
                sorter.setSorterTarget(11 * Sorter.INCREMENT);
                if (sorter.sorterAtTarget()) setPathState(35);
                break;

            case 35:
                if (shooter.shooterAtTarget()) {
                    lever.leverUp();
                    actionTimer.resetTimer();
                    setPathState(36);
                }
                break;

            case 36:
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    lever.leverDown();
                    actionTimer.resetTimer();
                    setPathState(37);
                }
                break;

            case 37:
                sorter.setSorterTarget(12 * Sorter.INCREMENT);
                if (sorter.sorterAtTarget()) setPathState(38);
                break;

            case 38:
                if (shooter.shooterAtTarget()) {
                    lever.leverUp();
                    actionTimer.resetTimer();
                    setPathState(39);
                }
                break;

            case 39:
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    lever.leverDown();
                    actionTimer.resetTimer();
                    setPathState(40);
                }
                break;

            case 40:
                follower.followPath(goToEnd, true);
                setPathState(41);
                break;

            case 41:
                break;
        }
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        shooter.PIDFShootingLoop();
        sorter.PIDFSorterLoop();

        follower.update();
        autonomousPathUpdate();

        double elapsed   = opmodeTimer.getElapsedTimeSeconds();
        double remaining = AUTO_DURATION - elapsed;

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("time elapsed", elapsed);
        telemetry.addData("time remaining", remaining);
        telemetry.update();
    }
}