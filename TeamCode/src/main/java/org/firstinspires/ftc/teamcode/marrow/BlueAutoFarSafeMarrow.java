package org.firstinspires.ftc.teamcode.marrow;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Lever;
import org.firstinspires.ftc.teamcode.Pitch;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.Sorter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Auto Far Safe (Marrow)", group = "Marrow")
public class BlueAutoFarSafeMarrow extends OpMode {


    private final Intake intake = new Intake();
    private final Lever lever = new Lever();
    private final Pitch pitch = new Pitch();
    private final Shooter shooter = new Shooter();
    private final Sorter sorter = new Sorter();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState = 0;


    private final Pose startPose = new Pose(56.625, 8.75, Math.toRadians(90));
    private final Pose secondPose = new Pose(55.6108, 14, Math.toRadians(90));
    private final Pose scorePose = new Pose(59.421, 15.18, Math.toRadians(124.5));
    private final Pose pickupLowPose = new Pose(48, 36, Math.toRadians(180));
    private final Pose pickupLowIntake3 = new Pose(24, 36, Math.toRadians(180));
    private final Pose pickupMidPose = new Pose(44, 60, Math.toRadians(180));
    private final Pose pickupMidIntake3 = new Pose(24, 60, Math.toRadians(180));
    private final Pose endPose = new Pose(60.362, 44.038, Math.toRadians(90));


    private Path startPreload;
    private PathChain score1, alignToLow, intakeLow, scoreFromLow;
    private PathChain alignToMid, intakeMid, scoreFromMid, goToEnd;


    private static final double AUTO_DURATION = 30.0;
    private static final double SAFETY_MARGIN = 5.0;

    private static final double SHOOTER_TIMEOUT = 1.5;
    private static final int SHOOTER_MAX_RETRIES = 1;
    private int shooterRetryCount = 0;
    private final Timer shooterWaitTimer = new Timer();
    private static final double INTAKE_TIMEOUT = 2.0;
    private static final int INTAKE_MAX_RETRIES = 1;
    private int intakeRetryCount = 0;
    private final Timer intakeWaitTimer = new Timer();


    private static class Point { final double x, y; Point(double x, double y){this.x=x;this.y=y;} }
    private final Point[] launchZone = new Point[]{
            new Point(56, 8),
            new Point(72, 24),
            new Point(40, 24)
    };


    private boolean isInLaunchZone(double x, double y) {
        boolean inside = false;
        for (int i = 0, j = launchZone.length - 1; i < launchZone.length; j = i++) {
            Point pi = launchZone[i];
            Point pj = launchZone[j];
            boolean intersect = ((pi.y > y) != (pj.y > y)) &&
                    (x < (pj.x - pi.x) * (y - pi.y) / ((pj.y - pi.y) + 1e-6) + pi.x);
            if (intersect) inside = !inside;
        }
        return inside;
    }

    public void buildPaths() {
        startPreload = new Path(new BezierLine(startPose, secondPose));
        startPreload.setConstantHeadingInterpolation(secondPose.getHeading());

        score1 = follower.pathBuilder()
                .addPath(new BezierLine(secondPose, scorePose))
                .addParametricCallback(0.01, () -> shooter.setCurTargetVelocityParametric("custom", 1630))
                .setHeadingConstraint(4)
                .setLinearHeadingInterpolation(secondPose.getHeading(), scorePose.getHeading())
                .build();

        alignToLow = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickupLowPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupLowPose.getHeading())
                .build();

        intakeLow = follower.pathBuilder()
                .addPath(new BezierLine(pickupLowPose, pickupLowIntake3))
                .addParametricCallback(0.5, () -> sorter.setSorterTargetParametric(627.2))
                .addParametricCallback(0.81, () -> sorter.setSorterTargetParametric(806.4))
                .setConstantHeadingInterpolation(pickupLowIntake3.getHeading())
                .setBrakingStrength(0.5)
                .build();

        scoreFromLow = follower.pathBuilder()
                .addPath(new BezierLine(pickupLowIntake3, scorePose))
                .addParametricCallback(0.01, () -> shooter.setCurTargetVelocity("custom", 1630))
                .setLinearHeadingInterpolation(pickupLowIntake3.getHeading(), scorePose.getHeading())
                .build();

        alignToMid = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickupMidPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupMidPose.getHeading())
                .build();

        intakeMid = follower.pathBuilder()
                .addPath(new BezierLine(pickupMidPose, pickupMidIntake3))
                .addParametricCallback(0.5, () -> sorter.setSorterTargetParametric(1523.3))
                .addParametricCallback(0.81, () -> sorter.setSorterTargetParametric(1702.4))
                .setConstantHeadingInterpolation(pickupMidIntake3.getHeading())
                .setBrakingStrength(0.5)
                .build();

        scoreFromMid = follower.pathBuilder()
                .addPath(new BezierLine(pickupMidIntake3, scorePose))
                .addParametricCallback(0.01, () -> shooter.setCurTargetVelocity("custom", 1630))
                .setHeadingConstraint(4)
                .setLinearHeadingInterpolation(pickupMidIntake3.getHeading(), scorePose.getHeading())
                .build();

        goToEnd = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

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

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    private double getEstimatedRemainingTimeFromState(int state) {
        switch (state) {
            case 0: case 1: case 2: case 3: case 4:
            case 5: case 6: case 7: case 8: case 9:
                return 8.0;

            case 10: case 11: case 12: case 13: case 14:
            case 15: case 16: case 17: case 18: case 19:
            case 20:

            case 21:
            case 22:
            case 23:
            case 24:
            case 25:
            case 26:
            case 27:
            case 28:
            case 29:
            case 30:
            case 31:
                return 10.0;

            default:
                return 3.0;
        }
    }

    private boolean shouldBailToPark() {
        double elapsed = opmodeTimer.getElapsedTimeSeconds();
        double remaining = AUTO_DURATION - elapsed;
        double required = getEstimatedRemainingTimeFromState(pathState);
        return remaining < SAFETY_MARGIN && remaining < required;
    }

    private void bailToEndZone() {
        follower.followPath(goToEnd, true);
        pathState = 100;
    }


    private boolean ensureShooterAtTargetOrRetry() {
        if (shooter.ShooterAtTarget()) {
            shooterRetryCount = 0;
            return true;
        }

        if (shooterRetryCount == 0) {
            shooterRetryCount++;
            shooterWaitTimer.resetTimer();
            return false;
        }

        if (shooterWaitTimer.getElapsedTimeSeconds() > SHOOTER_TIMEOUT) {
            if (shooterRetryCount < SHOOTER_MAX_RETRIES) {
                shooterRetryCount++;
                shooterWaitTimer.resetTimer();
            } else {
                shooterRetryCount = 0;
            }
            return false;
        }

        return false;
    }

    private boolean ensureIntakeFinishedOrRetry(boolean pathBusy,
                                                Runnable startIntakePath,
                                                Runnable onGiveUp) {
        if (!pathBusy) {
            intakeRetryCount = 0;
            return true;
        }

        if (intakeRetryCount == 0) {
            intakeRetryCount++;
            intakeWaitTimer.resetTimer();
            return false;
        }

        if (intakeWaitTimer.getElapsedTimeSeconds() > INTAKE_TIMEOUT) {
            if (intakeRetryCount < INTAKE_MAX_RETRIES) {
                intakeRetryCount++;
                intakeWaitTimer.resetTimer();
                if (startIntakePath != null) startIntakePath.run();
            } else {
                intakeRetryCount = 0;
                if (onGiveUp != null) onGiveUp.run();
            }
            return false;
        }

        return false;
    }

    private void moveSorterTo(double ticks, int nextState) {
        sorter.setSorterTarget(ticks);
        if (!sorter.isBusy()) {
            setPathState(nextState);
        }
    }


    public void autonomousPathUpdate() {
        if (pathState <= 32 && shouldBailToPark()) {
            bailToEndZone();
            return;
        }

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
                if (!follower.isBusy()) {
                    boolean inZone = isInLaunchZone(follower.getPose().getX(), follower.getPose().getY());
                    if (!inZone) {
                        bailToEndZone();
                        break;
                    }
                    if (ensureShooterAtTargetOrRetry()) {
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
                moveSorterTo(179.2, 5);
                break;

            case 5:
                if (ensureShooterAtTargetOrRetry()) {
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
                moveSorterTo(358.4, 8);
                break;

            case 8:
                if (ensureShooterAtTargetOrRetry()) {
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
                follower.followPath(alignToLow, true);
                if (!sorter.isBusy()) {
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    intake.intakeOn();
                    follower.followPath(intakeLow, 0.32, true);
                    intakeWaitTimer.resetTimer();
                    setPathState(12);
                }
                break;

            case 12:
                if (ensureIntakeFinishedOrRetry(
                        follower.isBusy(),
                        () -> follower.followPath(intakeLow, 0.32, true),
                        intake::intakeOff
                )) {
                    intake.intakeOff();
                    sorter.setSorterTarget(896);
                    follower.followPath(scoreFromLow, true);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    boolean inZone2 = isInLaunchZone(follower.getPose().getX(), follower.getPose().getY());
                    if (!inZone2) {
                        bailToEndZone();
                        break;
                    }
                    if (ensureShooterAtTargetOrRetry()) {
                        lever.leverUp();
                        actionTimer.resetTimer();
                        setPathState(14);
                    }
                }
                break;

            case 14:
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    lever.leverDown();
                    actionTimer.resetTimer();
                    setPathState(15);
                }
                break;

            case 15:
                moveSorterTo(1075.2, 16);
                break;

            case 16:
                if (ensureShooterAtTargetOrRetry()) {
                    lever.leverUp();
                    actionTimer.resetTimer();
                    setPathState(17);
                }
                break;

            case 17:
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    lever.leverDown();
                    actionTimer.resetTimer();
                    setPathState(18);
                }
                break;

            case 18:
                moveSorterTo(1254.4, 19);
                break;

            case 19:
                if (ensureShooterAtTargetOrRetry()) {
                    lever.leverUp();
                    actionTimer.resetTimer();
                    setPathState(20);
                }
                break;

            case 20:
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    lever.leverDown();
                    actionTimer.resetTimer();
                    setPathState(21);
                }
                break;

            case 21:
                sorter.setSorterTarget(1344);
                shooter.setCurTargetVelocity("0", 0);
                follower.followPath(alignToMid, true);
                if (!sorter.isBusy()) {
                    setPathState(22);
                }
                break;

            case 22:
                if (!follower.isBusy()) {
                    intake.intakeOn();
                    follower.followPath(intakeMid, 0.32, true);
                    intakeWaitTimer.resetTimer();
                    setPathState(23);
                }
                break;

            case 23:
                if (ensureIntakeFinishedOrRetry(
                        follower.isBusy(),
                        () -> follower.followPath(intakeMid, 0.32, true),
                        intake::intakeOff
                )) {
                    intake.intakeOff();
                    sorter.setSorterTarget(1792);
                    follower.followPath(scoreFromMid, true);
                    setPathState(24);
                }
                break;

            case 24:
                if (!follower.isBusy()) {
                    boolean inZone3 = isInLaunchZone(follower.getPose().getX(), follower.getPose().getY());
                    if (!inZone3) {
                        bailToEndZone();
                        break;
                    }
                    if (ensureShooterAtTargetOrRetry()) {
                        lever.leverUp();
                        actionTimer.resetTimer();
                        setPathState(25);
                    }
                }
                break;

            case 25:
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    lever.leverDown();
                    actionTimer.resetTimer();
                    setPathState(26);
                }
                break;

            case 26:
                moveSorterTo(1971.2, 27);
                break;

            case 27:
                if (ensureShooterAtTargetOrRetry()) {
                    lever.leverUp();
                    actionTimer.resetTimer();
                    setPathState(28);
                }
                break;

            case 28:
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    lever.leverDown();
                    actionTimer.resetTimer();
                    setPathState(29);
                }
                break;

            case 29:
                moveSorterTo(2150.4, 30);
                break;

            case 30:
                if (ensureShooterAtTargetOrRetry()) {
                    lever.leverUp();
                    actionTimer.resetTimer();
                    setPathState(31);
                }
                break;

            case 31:
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    lever.leverDown();
                    actionTimer.resetTimer();
                    setPathState(32);
                }
                break;

            case 100:
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
        sorter.update();
        pitch.pitchDown();

        follower.update();
        autonomousPathUpdate();

        double elapsed = opmodeTimer.getElapsedTimeSeconds();
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
