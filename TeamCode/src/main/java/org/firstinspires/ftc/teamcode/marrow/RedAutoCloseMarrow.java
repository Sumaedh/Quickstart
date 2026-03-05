package org.firstinspires.ftc.teamcode.marrow;
/*
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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

@Autonomous(name = "Red Auto Close Safe (Marrow)", group = "Marrow")
public class RedAutoCloseMarrow extends OpMode {

    private final Intake intake = new Intake();
    private final Lever lever = new Lever();
    private final Pitch pitch = new Pitch();
    private final Shooter shooter = new Shooter();
    private final Sorter sorter = new Sorter();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState = 0;


    private final Pose startPose    = new Pose(20.39, 121.42, Math.toRadians(142.5)).mirror();
    private final Pose scorePose    = new Pose(65.09, 78.28, Math.toRadians(135)).mirror();
    private final Pose lowIntakePose  = new Pose(26.5, 84.24, Math.toRadians(180)).mirror();
    private final Pose midIntakePose  = new Pose(26.5, 59.69, Math.toRadians(180)).mirror();
    private final Pose lastIntakePose = new Pose(26.5, 36, Math.toRadians(180)).mirror();
    private final Pose endPose      = new Pose(65.5, 36.95, Math.toRadians(180)).mirror();

    private Path startPreload;
    private PathChain score1, scoreFromLow;
    private PathChain scoreFromMid, goToEnd;
    private PathChain scoreFromLast;
    private PathChain pickupLowChain, pickupMidChain, pickupLastChain;

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

    private static class Point {
        final double x, y;
        Point(double x, double y){this.x=x;this.y=y;}
    }

    private final Point[] launchZone = new Point[]{
            new Point(40, 8),
            new Point(60, 26),
            new Point(32, 26)
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

    private boolean isPoseTooFarFrom(Pose target, double maxDist) {
        Pose cur = follower.getPose();
        double dx = cur.getX() - target.getX();
        double dy = cur.getY() - target.getY();
        return Math.hypot(dx, dy) > maxDist;
    }

    public void buildPaths() {

        startPreload = new Path(
                new BezierCurve(
                        startPose,
                        new Pose(32, 110),
                        new Pose(52, 90),
                        scorePose
                )
        );
        startPreload.setConstantHeadingInterpolation(scorePose.getHeading());

        score1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        startPose,
                        new Pose(32, 110),
                        new Pose(52, 90),
                        scorePose
                ))
                .addParametricCallback(0.01, () -> {
                    Pose cur = follower.getPose();
                    double dx = 144.0 - cur.getX();
                    double dy = 72.0 - cur.getY();
                    double d = Math.hypot(dx, dy);
                    shooter.setCurTargetVelocityDynamic(d);
                })
                .setHeadingConstraint(4)
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();


        pickupLowChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePose,
                        new Pose(55, 82),
                        new Pose(35, 86),
                        lowIntakePose
                ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(180))
                .setBrakingStrength(0.5)
                .addParametricCallback(0.50, () -> {
                    intake.intakeOn();
                    intakeWaitTimer.resetTimer();
                })
                .addParametricCallback(0.75, () -> sorter.setSorterTargetParametric(627.2))
                .addParametricCallback(0.90, () -> sorter.setSorterTargetParametric(806.4))
                .build();

        scoreFromLow = follower.pathBuilder()
                .addPath(new BezierCurve(
                        lowIntakePose,
                        new Pose(34, 84),
                        new Pose(50, 80),
                        scorePose
                ))
                .addParametricCallback(0.01, () -> {
                    Pose cur = follower.getPose();
                    double dx = 144.0 - cur.getX();
                    double dy = 72.0 - cur.getY();
                    double d = Math.hypot(dx, dy);
                    shooter.setCurTargetVelocityDynamic(d);
                })
                .setLinearHeadingInterpolation(lowIntakePose.getHeading(), scorePose.getHeading())
                .build();

        pickupMidChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePose,
                        new Pose(57.140540540540535, 76.33513513513513),
                        new Pose(50.17837837837838, 58.94054054054053),
                        midIntakePose
                ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(180))
                .setBrakingStrength(0.5)
                .addParametricCallback(0.70, () -> {
                    intake.intakeOn();
                    intakeWaitTimer.resetTimer();
                })
                .addParametricCallback(0.86, () -> sorter.setSorterTargetParametric(1523.3))
                .addParametricCallback(0.95, () -> sorter.setSorterTargetParametric(1702.4))
                .build();

        scoreFromMid = follower.pathBuilder()
                .addPath(new BezierCurve(
                        midIntakePose,
                        new Pose(34, 60),
                        new Pose(50, 72),
                        scorePose
                ))
                .addParametricCallback(0.01, () -> {
                    Pose cur = follower.getPose();
                    double dx = 144.0 - cur.getX();
                    double dy = 72.0 - cur.getY();
                    double d = Math.hypot(dx, dy);
                    shooter.setCurTargetVelocityDynamic(d);
                })
                .setHeadingConstraint(4)
                .setLinearHeadingInterpolation(midIntakePose.getHeading(), scorePose.getHeading())
                .build();

        pickupLastChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePose,
                        new Pose(55, 70),
                        new Pose(52.864864864864856, 33.21081081081078),
                        lastIntakePose
                ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(180))
                .addParametricCallback(0.78, () -> {
                    intake.intakeOn();
                    intakeWaitTimer.resetTimer();
                })
                .build();

        scoreFromLast = follower.pathBuilder()
                .addPath(new BezierCurve(
                        lastIntakePose,
                        new Pose(39.6972972972973, 37.16756756756757),
                        new Pose(37.16756756756757, 36),
                        scorePose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), scorePose.getHeading())
                .build();

        goToEnd = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePose,
                        new Pose(48, 28),
                        new Pose(50, 36),
                        endPose
                ))
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
        if (shooter.shooterAtTarget()) {
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

    private boolean ensureIntakeFinishedOrRetry(boolean pathBusy, Runnable startIntakePath, Runnable onGiveUp) {
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
        if (sorter.isBusy()) {
            setPathState(nextState);
        }
    }

    public void autonomousPathUpdate() {
        Pose cur;
        double dx, dy, d;
        double elapsed = opmodeTimer.getElapsedTimeSeconds();
        double remaining = AUTO_DURATION - elapsed;

        if (pathState <= 37 && shouldBailToPark()) {
            bailToEndZone();
            return;
        }

        if (pathState >= 30 && pathState < 37) {
            if (remaining < 6.0) {
                follower.followPath(goToEnd, true);
                pathState = 100;
                return;
            }
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
                    if (isPoseTooFarFrom(scorePose, 4.0) || !isInLaunchZone(follower.getPose().getX(), follower.getPose().getY())) {
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
                follower.followPath(pickupLowChain, 0.32, true);
                if (sorter.isBusy()) {
                    setPathState(11);
                }
                break;
            case 11:
                if (ensureIntakeFinishedOrRetry(
                        follower.isBusy(),
                        () -> follower.followPath(pickupLowChain, 0.32, true),
                        intake::intakeOff
                )) {
                    intake.intakeOff();
                    sorter.setSorterTarget(896);
                    follower.followPath(scoreFromLow, true);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    if (isPoseTooFarFrom(scorePose, 4.0) || !isInLaunchZone(follower.getPose().getX(), follower.getPose().getY())) {
                        bailToEndZone();
                        break;
                    }
                    if (ensureShooterAtTargetOrRetry()) {
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
                moveSorterTo(1075.2, 15);
                break;
            case 15:
                if (ensureShooterAtTargetOrRetry()) {
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
                moveSorterTo(1254.4, 18);
                break;
            case 18:
                if (ensureShooterAtTargetOrRetry()) {
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
                sorter.setSorterTarget(1344);
                shooter.setCurTargetVelocity("0", 0);
                follower.followPath(pickupMidChain, 0.32, true);
                if (sorter.isBusy()) {
                    setPathState(21);
                }
                break;
            case 21:
                if (ensureIntakeFinishedOrRetry(
                        follower.isBusy(),
                        () -> follower.followPath(pickupMidChain, 0.32, true),
                        intake::intakeOff
                )) {
                    intake.intakeOff();
                    sorter.setSorterTarget(1792);
                    follower.followPath(scoreFromMid, true);
                    setPathState(22);
                }
                break;
            case 22:
                if (!follower.isBusy()) {
                    if (isPoseTooFarFrom(scorePose, 4.0) || !isInLaunchZone(follower.getPose().getX(), follower.getPose().getY())) {
                        bailToEndZone();
                        break;
                    }
                    if (ensureShooterAtTargetOrRetry()) {
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
                moveSorterTo(1971.2, 25);
                break;
            case 25:
                if (ensureShooterAtTargetOrRetry()) {
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
                moveSorterTo(2150.4, 28);
                break;
            case 28:
                if (ensureShooterAtTargetOrRetry()) {
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
                follower.followPath(pickupLastChain, 0.32, true);
                setPathState(31);
                break;
            case 31:
                if (ensureIntakeFinishedOrRetry(
                        follower.isBusy(),
                        () -> follower.followPath(pickupLastChain, 0.32, true),
                        intake::intakeOff
                )) {
                    intake.intakeOff();
                    follower.followPath(scoreFromLast, true);
                    setPathState(32);
                }
                break;
            case 32:
                if (!follower.isBusy()) {
                    if (isPoseTooFarFrom(scorePose, 4.0) || !isInLaunchZone(follower.getPose().getX(), follower.getPose().getY())) {
                        bailToEndZone();
                        break;
                    }
                    if (ensureShooterAtTargetOrRetry()) {
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
                moveSorterTo(2300, 35);
                break;
            case 35:
                if (ensureShooterAtTargetOrRetry()) {
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
                follower.followPath(goToEnd, true);
                setPathState(100);
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

 */
