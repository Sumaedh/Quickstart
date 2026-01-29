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
public class BlueAutoFarTurret2 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Intake intake = new Intake();
    private final Lever lever = new Lever();
    private final Pitch pitch = new Pitch();
    private final Shooter shooter = new Shooter();
    private final Sorter sorter = new Sorter();
    private final Turret turret = new Turret();

    private final Pose startPose        = new Pose(56.625, 8.75, Math.toRadians(90));
    private final Pose secondPose       = new Pose(55.6108, 14, Math.toRadians(90));
    private final Pose scorePose        = new Pose(59.535, 14, Math.toRadians(90));

    private final Pose pickupLowPose        = new Pose(48, 36, Math.toRadians(180));
    private final Pose pickupLowIntake3     = new Pose(22, 36, Math.toRadians(180));

    private final Pose pickupMidPose        = new Pose(48, 60, Math.toRadians(180));
    private final Pose pickupMidIntake3     = new Pose(21, 60, Math.toRadians(180));

    private final Pose endPose          = new Pose(60.362, 44.038, Math.toRadians(90));

    private Path startPreload;
    private PathChain score1;
    private PathChain alignToLow;
    private PathChain intakeLow;
    private PathChain scoreFromLow;
    private PathChain alignToMid;
    private PathChain intakeMid;
    private PathChain scoreFromMid;
    private PathChain goToEnd;

    private double hSecondToScore;
    private double hLowToScore;
    private double hMidToScore;

    private double normAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    private double turretHeadingFromFieldHeading(double fieldHeading) {
        double reversed = normAngle(fieldHeading + Math.PI);
        double offset = -Math.PI / 2;
        return normAngle(reversed + offset);
    }

    private double headingAlong(Pose from, Pose to) {
        return Math.atan2(to.getY() - from.getY(), to.getX() - from.getX());
    }

    public void buildPaths() {

        double hStartToSecond = headingAlong(startPose, secondPose);
        startPreload = new Path(new BezierLine(startPose, secondPose));
        startPreload.setConstantHeadingInterpolation(hStartToSecond);

        hSecondToScore = headingAlong(secondPose, scorePose);
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(secondPose, scorePose))
                .setLinearHeadingInterpolation(hSecondToScore, scorePose.getHeading())
                .build();

        double hScoreToLow = headingAlong(scorePose, pickupLowPose);
        alignToLow = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickupLowPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), hScoreToLow)
                .build();

        double hLowIntake = headingAlong(pickupLowPose, pickupLowIntake3);
        intakeLow = follower.pathBuilder()
                .addPath(new BezierLine(pickupLowPose, pickupLowIntake3))
                .setConstantHeadingInterpolation(hLowIntake)
                .setBrakingStrength(0.4)
                .build();

        hLowToScore = headingAlong(pickupLowIntake3, scorePose);
        scoreFromLow = follower.pathBuilder()
                .addPath(new BezierLine(pickupLowIntake3, scorePose))
                .setLinearHeadingInterpolation(hLowToScore, scorePose.getHeading())
                .build();

        double hScoreToMid = headingAlong(scorePose, pickupMidPose);
        alignToMid = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickupMidPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), hScoreToMid)
                .build();

        double hMidIntake = headingAlong(pickupMidPose, pickupMidIntake3);
        intakeMid = follower.pathBuilder()
                .addPath(new BezierLine(pickupMidPose, pickupMidIntake3))
                .setConstantHeadingInterpolation(hMidIntake)
                .setBrakingStrength(0.4)
                .build();

        hMidToScore = headingAlong(pickupMidIntake3, scorePose);
        scoreFromMid = follower.pathBuilder()
                .addPath(new BezierLine(pickupMidIntake3, scorePose))
                .setLinearHeadingInterpolation(hMidToScore, scorePose.getHeading())
                .build();

        double hScoreToEnd = headingAlong(scorePose, endPose);
        goToEnd = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(hScoreToEnd, endPose.getHeading())
                .build();
    }

    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    private boolean leverCycleDone(double waitSeconds) {
        return actionTimer.getElapsedTimeSeconds() > waitSeconds;
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(startPreload);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    shooter.setCurTargetVelocity("long");
                    turret.setTurretTarget(turretHeadingFromFieldHeading(hSecondToScore));
                    follower.followPath(score1, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    turret.setTurretTarget(turretHeadingFromFieldHeading(scorePose.getHeading()));
                    if (shooter.ShooterAtTarget() && turret.turretAtTarget()) {
                        lever.leverUp();
                        actionTimer.resetTimer();
                        setPathState(3);
                    }
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
                    follower.followPath(intakeLow, 0.35, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    intake.intakeOff();
                    sorter.setSorterTarget(896);

                    // ⭐ FIRST REVERSAL — LOW STACK
                    shooter.setCurTargetVelocity("long");
                    turret.setTurretTarget(
                            turretHeadingFromFieldHeading(hLowToScore + Math.PI)
                    );

                    follower.followPath(scoreFromLow, true);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    turret.setTurretTarget(turretHeadingFromFieldHeading(scorePose.getHeading()));
                    if (shooter.ShooterAtTarget() && turret.turretAtTarget()) {
                        lever.leverUp();
                        actionTimer.resetTimer();
                        setPathState(14);
                    }
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
                    follower.followPath(intakeMid, 0.35, true);
                    setPathState(23);
                }
                break;

            case 23:
                if (!follower.isBusy()) {
                    intake.intakeOff();
                    sorter.setSorterTarget(1792);

                    // ⭐ SECOND REVERSAL — MID STACK
                    shooter.setCurTargetVelocity("long");
                    turret.setTurretTarget(
                            turretHeadingFromFieldHeading(hMidToScore + Math.PI)
                    );

                    follower.followPath(scoreFromMid, true);
                    setPathState(24);
                }
                break;

            case 24:
                if (!follower.isBusy()) {
                    turret.setTurretTarget(turretHeadingFromFieldHeading(scorePose.getHeading()));
                    if (shooter.ShooterAtTarget() && turret.turretAtTarget()) {
                        lever.leverUp();
                        actionTimer.resetTimer();
                        setPathState(25);
                    }
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

            default:
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
}
