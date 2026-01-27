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
public class BlueAutoFarSafe extends OpMode {

    // HEIGHT: 17.25 INCHES
    // WIDTH: 17.75 inches

    private Follower follower;

    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    Intake intake = new Intake();

    Lever lever = new Lever();

    Pitch pitch = new Pitch();

    Shooter shooter = new Shooter();

    Sorter sorter = new Sorter();

    Turret turret = new Turret();

    // Poses

    private final Pose startPose = new Pose(56.625, 8.75, Math.toRadians(90));
    private final Pose secondPose = new Pose(55.6108, 14, Math.toRadians(90));
    private final Pose scorePose = new Pose(59.421, 15.18, Math.toRadians(124.5));
    //137
    private final Pose pickupLowPose = new Pose(48, 36, Math.toRadians(180));
    private final Pose pickupLowIntake1 = new Pose(37.622, 36, Math.toRadians(180));
    private final Pose pickupLowIntake2 = new Pose(32.292, 36, Math.toRadians(180));
    private final Pose pickupLowIntake3 = new Pose(24, 36, Math.toRadians(180));

    private final Pose pickupMidPose = new Pose(44, 36 +24, Math.toRadians(180));
    private final Pose pickupMidIntake1 = new Pose(37.622, 36 + 24, Math.toRadians(180));
    private final Pose pickupMidIntake2 = new Pose(32.292, 36 + 24, Math.toRadians(180));
    private final Pose pickupMidIntake3 = new Pose(24, 36 + 24, Math.toRadians(180));

    private final Pose endPose = new Pose(60.362, 44.038, Math.toRadians(90));

    private Path startPreload;

    private PathChain score1, alignToMid, intakeMid, scoreFromMid, alignToLow, intakeLow, scoreFromLow, goToEnd;

    // PATH BUILDER

    public void buildPaths() {
        startPreload = new Path(new BezierLine(startPose, secondPose));
        startPreload.setConstantHeadingInterpolation(secondPose.getHeading());

        score1 = follower.pathBuilder()
                .addPath(new BezierLine(secondPose, scorePose))
                .addParametricCallback(0.01, () -> shooter.setCurTargetVelocityParametric("long"))
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
                .addParametricCallback(0.01, () -> shooter.setCurTargetVelocity("long"))
                .setLinearHeadingInterpolation(pickupLowIntake3.getHeading(), scorePose.getHeading())
                .build();

        alignToMid = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickupMidPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupMidPose.getHeading())
                .build();

        // TODO: FIX
        intakeMid = follower.pathBuilder()
                .addPath(new BezierLine(pickupMidPose, pickupMidIntake3))
                .addParametricCallback(0.5, () -> sorter.setSorterTargetParametric(1523.3))
                .addParametricCallback(0.81, () -> sorter.setSorterTargetParametric(1702.4))
                .setConstantHeadingInterpolation(pickupMidIntake3.getHeading())
                .setBrakingStrength(0.5)
                .build();

        scoreFromMid = follower.pathBuilder()
                .addPath(new BezierLine(pickupMidIntake3, scorePose))
                .addParametricCallback(0.01, () -> shooter.setCurTargetVelocity("long"))
                .setHeadingConstraint(4)
                .setLinearHeadingInterpolation(pickupMidIntake3.getHeading(), scorePose.getHeading())
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
                // SHOOT SEQUENCE 1 START
                if (!follower.isBusy()) {
                    follower.followPath(score1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    if (shooter.ShooterAtTarget()) {
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
                sorter.setSorterTarget(179.2);
                if (sorter.SorterAtTarget()) {
                    setPathState(5);
                }
                break;
            case 5:
                // SHOOT SEQUENCE 2 START
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
                if (sorter.SorterAtTarget()) {

                    setPathState(8);

                }
                break;
            case 8:
                // SHOOT SEQUENCE 3 START
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
                // GOING TO INTAKE 1
                sorter.setSorterTarget(448);
                shooter.setCurTargetVelocity("0");
                follower.followPath(alignToLow, true);
                if (sorter.SorterAtTarget()) {
                    setPathState(11);
                }
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
                // SHOOTING SEQUENCE 1 (2nd time)
                if (!follower.isBusy()) {
                    if (shooter.ShooterAtTarget()) {
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
                sorter.setSorterTarget(1075.2);
                if (sorter.SorterAtTarget()) {
                    setPathState(16);
                }
                break;
            case 16:
                // SHOOT SEQUENCE 2 START (2nd time)
                if (shooter.ShooterAtTarget()) {
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
                sorter.setSorterTarget(1254.4);
                if (sorter.SorterAtTarget()) {
                    setPathState(19);
                }
                break;
            case 19:
                // SHOOT SEQUENCE 3 START (2nd time)
                if (shooter.ShooterAtTarget()) {
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
                // GOING TO INTAKE 2
                sorter.setSorterTarget(1344);
                shooter.setCurTargetVelocity("0");
                follower.followPath(alignToMid, true);
                if (sorter.SorterAtTarget()) {
                    setPathState(22);
                }
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
                if (!follower.isBusy()) {
                    if (shooter.ShooterAtTarget()) {
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
                sorter.setSorterTarget(1971.2);
                if (sorter.SorterAtTarget()) {
                    setPathState(27);
                }
                break;
            case 27:
                // SHOOT SEQUENCE 2 START
                if (shooter.ShooterAtTarget()) {
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
                sorter.setSorterTarget(2150.4);
                if (sorter.SorterAtTarget()) {

                    setPathState(30);

                }
                break;
            case 30:
                // SHOOT SEQUENCE 3 START
                if (shooter.ShooterAtTarget()) {
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
        }

    }

    /**

     * These change the states of the paths and actions. It will also reset the timers of the individual switches

     **/

    public void setPathState(int pState) {

        pathState = pState;

        pathTimer.resetTimer();

    }

    /**

     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".

     **/

    @Override

    public void loop() {

        shooter.PIDFShootingLoop();

        sorter.PIDFSorterLoop();

        pitch.pitchDown();

        // These loop the movements of the robot, these must be called continuously in order to work

        follower.update();

        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging

        telemetry.addData("path state", pathState);

        telemetry.addData("x", follower.getPose().getX());

        telemetry.addData("y", follower.getPose().getY());

        telemetry.addData("heading", follower.getPose().getHeading());

        telemetry.update();

    }

    /**

     * This method is called once at the init of the OpMode.

     **/

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

    /**

     * This method is called continuously after Init while waiting for "play".

     **/

    @Override

    public void init_loop() {

    }

    /**

     * This method is called once at the start of the OpMode.

     * It runs all the setup actions, including building paths and starting the path system

     **/

    @Override

    public void start() {

        opmodeTimer.resetTimer();

        setPathState(0);

    }

    /**

     * We do not use this because everything should automatically disable

     **/

    @Override

    public void stop() {

    }

}