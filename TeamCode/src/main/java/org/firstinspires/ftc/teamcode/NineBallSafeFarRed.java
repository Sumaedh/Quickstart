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

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class NineBallSafeFarRed extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    Intake intake = new Intake();
    Lever lever = new Lever();
    Pitch pitch = new Pitch();
    Shooter shooter = new Shooter();
    Turret turret = new Turret();

    // Sorter PID
    private PIDController sorterController;
    private double pSorting = 0.004;
    private double iSorting = 0.0;
    private double dSorting = 0.00027;
    public static double kSSorting = 0.034;

    private static final double TICKS_PER_REV = 537.6 * ((double) 10 / 14);
    public static double INCREMENT = TICKS_PER_REV / 6;

    private DcMotor sorterMotor;
    private double target = 0;
    private double sorterTolerance = 20;

    // Poses
    private final Pose startPose = new Pose(88.389, 6.832, Math.toRadians(90));
    private final Pose scorePose = new Pose(84.076, 19.849, Math.toRadians(90));
    private final Pose pickupLowPose = new Pose(101.87567567567568, 36.184, Math.toRadians(0));
    private final Pose pickupLowIntake3 = new Pose(120.53, 36.184, Math.toRadians(0));
    private final Pose pickupMidPose = new Pose(104.876, 36.184, Math.toRadians(0));
    private final Pose pickupMidIntake3 = new Pose(124.857, 60.614, Math.toRadians(0));
    private final Pose endPose = new Pose(84.076, 19.849, Math.toRadians(90));

    // PathChains
    private PathChain startPreload, score1, alignToLowFromStart, alignToLow,
            intakeLow, scoreFromLow, alignToMid, intakeMid, scoreFromMid, goToEnd;

    public void buildPaths() {

        // PRELOAD (wrap Path → PathChain)
        Path preloadPath = new Path(new BezierLine(startPose, scorePose));
        preloadPath.setConstantHeadingInterpolation(scorePose.getHeading());

        startPreload = follower.pathBuilder()
                .addPath(preloadPath)
                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        alignToLowFromStart = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pickupLowPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickupLowPose.getHeading())
                .build();

        alignToLow = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickupLowPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupLowPose.getHeading())
                .build();

        intakeLow = follower.pathBuilder()
                .addPath(new BezierLine(pickupLowPose, pickupLowIntake3))
                .addParametricCallback(0.32, () -> target = INCREMENT * 7)
                .addParametricCallback(0.54, () -> target = INCREMENT * 9)
                .setConstantHeadingInterpolation(pickupLowIntake3.getHeading())
                .setBrakingStrength(0.5)
                .build();

        scoreFromLow = follower.pathBuilder()
                .addPath(new BezierLine(pickupLowIntake3, scorePose))
                .addParametricCallback(0.01, () -> shooter.setCurTargetVelocity("custom", 1660))
                .setLinearHeadingInterpolation(pickupLowIntake3.getHeading(), scorePose.getHeading())
                .build();

        alignToMid = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickupMidPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupMidPose.getHeading())
                .build();

        intakeMid = follower.pathBuilder()
                .addPath(new BezierLine(pickupMidPose, pickupMidIntake3))
                .addParametricCallback(0.32, () -> target = INCREMENT * 17)
                .addParametricCallback(0.54, () -> target = INCREMENT * 19)
                .setConstantHeadingInterpolation(pickupMidIntake3.getHeading())
                .setBrakingStrength(0.5)
                .build();

        scoreFromMid = follower.pathBuilder()
                .addPath(new BezierLine(pickupMidIntake3, scorePose))
                .addParametricCallback(0.01, () -> shooter.setCurTargetVelocity("custom", 1625))
                .addParametricCallback(0.20, () -> follower.setMaxPower(0.85))
                .addParametricCallback(0.70, () -> follower.setMaxPower(0.75))
                .setLinearHeadingInterpolation(pickupMidIntake3.getHeading(), scorePose.getHeading())
                .build();

        goToEnd = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {

        switch (pathState) {

            // PRELOAD → SCORE 3
            case 0:
                shooter.setCurTargetVelocity("custom", 1685);
                follower.getConstraints().setHeadingConstraint(3);
                follower.followPath(startPreload, 1, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) setPathState(2);
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
                target = INCREMENT * 2;
                if (Math.abs(sorterMotor.getCurrentPosition() - target) < sorterTolerance)
                    setPathState(5);
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
                target = INCREMENT * 4;
                if (Math.abs(sorterMotor.getCurrentPosition() - target) < sorterTolerance)
                    setPathState(8);
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

            // GO TO LOW INTAKE
            case 10:
                target = INCREMENT * 6; // FIXED (was 5)
                shooter.setCurTargetVelocity("0", 0);
                follower.getConstraints().setHeadingConstraint(8);
                follower.followPath(alignToLowFromStart, 1, true);
                if (Math.abs(sorterMotor.getCurrentPosition() - target) < sorterTolerance)
                    setPathState(11);
                break;

            case 11:
                if (!follower.isBusy()) {
                    intake.intakeOn();
                    follower.getConstraints().setHeadingConstraint(8);
                    follower.followPath(intakeLow, 0.41, true);
                    setPathState(12);
                }
                break;

            // LOW → SCORE
            case 12:
                if (!follower.isBusy()) {
                    intake.intakeOff();
                    target = INCREMENT * 10;
                    follower.getConstraints().setHeadingConstraint(4);
                    follower.followPath(scoreFromLow, 1, true);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy() && shooter.ShooterAtTarget()) {
                    lever.leverUp();
                    actionTimer.resetTimer();
                    setPathState(14);
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
                target = INCREMENT * 12;
                if (Math.abs(sorterMotor.getCurrentPosition() - target) < sorterTolerance)
                    setPathState(16);
                break;

            case 16:
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
                target = INCREMENT * 14;
                if (Math.abs(sorterMotor.getCurrentPosition() - target) < sorterTolerance)
                    setPathState(19);
                break;

            case 19:
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

            // GO TO MID INTAKE
            case 21:
                target = INCREMENT * 16; // FIXED (was 15)
                shooter.setCurTargetVelocity("0", 0);
                follower.getConstraints().setHeadingConstraint(8);
                follower.followPath(alignToMid, 1, true);
                if (Math.abs(sorterMotor.getCurrentPosition() - target) < sorterTolerance)
                    setPathState(22);
                break;

            case 22:
                if (!follower.isBusy()) {
                    intake.intakeOn();
                    follower.getConstraints().setHeadingConstraint(8);
                    follower.followPath(intakeMid, 0.41, true);
                    setPathState(23);
                }
                break;

            // MID → SCORE
            case 23:
                if (!follower.isBusy()) {
                    target = INCREMENT * 20;
                    intake.intakeOff();
                    follower.getConstraints().setHeadingConstraint(4);
                    follower.followPath(scoreFromMid, 1, true);
                    setPathState(24);
                }
                break;

            case 24:
                if (!follower.isBusy() && shooter.ShooterAtTarget()) {
                    lever.leverUp();
                    actionTimer.resetTimer();
                    setPathState(25);
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
                target = INCREMENT * 22;
                if (Math.abs(sorterMotor.getCurrentPosition() - target) < sorterTolerance)
                    setPathState(27);
                break;

            case 27:
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
                target = INCREMENT * 24;
                if (Math.abs(sorterMotor.getCurrentPosition() - target) < sorterTolerance)
                    setPathState(30);
                break;

            case 30:
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

            // PARK
            case 32:
                follower.getConstraints().setHeadingConstraint(10);
                follower.followPath(goToEnd, 1, true);
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
        pitch.pitchDown();
        turret.PIDFTurretLoop();
        follower.update();
        autonomousPathUpdate();

        sorterController.setPID(pSorting, iSorting, dSorting);
        double currentPos = sorterMotor.getCurrentPosition();
        double error = target - currentPos;
        double pidOutput = sorterController.calculate(currentPos, target);
        double staticFF = kSSorting * Math.signum(error);
        sorterMotor.setPower(pidOutput + staticFF);

        telemetry.addData("path state", pathState);
        telemetry.addData("sorter target", target);
        telemetry.addData("sorter pos", currentPos);
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
        turret.initTurret(hardwareMap, telemetry);

        sorterController = new PIDController(pSorting, iSorting, dSorting);
        sorterMotor = hardwareMap.get(DcMotor.class, "sorterMotor");
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sorterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        sorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lever.leverDown();
        pitch.pitchUp();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {}
}
