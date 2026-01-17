package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Config
@Autonomous(name = "Blue Auto Safe")
public class BlueAutoSafe extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private boolean justCompletedPath = false;
    private boolean sorterWaiting = false;
    private boolean shootingSequenceStarted = false;

    private ShooterSorterLeverFSM fsm;

    Intake intake = new Intake();
    Lever lever = new Lever();
    Pitch pitch = new Pitch();
    Shooter shooter = new Shooter();
    Sorter sorter = new Sorter();

    public static double WAIT_FOR_POCKET = 0.5;
    public static double INTAKE_BALL_WAIT = 0.5;
    public static double SHOOTING_SEQUENCE_WAIT = 0;

    // Poses
    private final Pose startPose = new Pose(55.6, 6.8, Math.toRadians(90));
    private final Pose secondPose = new Pose(55.6108, 19.8324, Math.toRadians(90));
    private final Pose scorePose = new Pose(60, 20, Math.toRadians(160));
    private final Pose pickup3Pose = new Pose(41.124, 29.584, Math.toRadians(180));
    private final Pose pickup3PoseIntake1 = new Pose(36.622, 29.584, Math.toRadians(180));
    private final Pose pickup3PoseIntake2 = new Pose(31.292, 29.584, Math.toRadians(180));
    private final Pose pickup3PoseIntake3 = new Pose(26.470, 29.584, Math.toRadians(180));
    private final Pose endPose = new Pose(37.55675675675676, 12.064864864864854, Math.toRadians(160));

    private Path startPreload;
    private PathChain score1, alignToBalls, intakeBall1, intakeBall2, intakeBall3, score2, goToEnd;

    public void buildPaths() {
        startPreload = new Path(new BezierLine(startPose, secondPose));
        startPreload.setConstantHeadingInterpolation(secondPose.getHeading());

        score1 = follower.pathBuilder()
                .addPath(new BezierLine(secondPose, scorePose))
                .setLinearHeadingInterpolation(secondPose.getHeading(), scorePose.getHeading())
                .build();

        goToEnd = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                // Drive from start to secondPose
                follower.followPath(startPreload);
                setPathState(1);
                break;

            case 1:
                // Go to long-shot score pose
                if (!follower.isBusy()) {
                    follower.followPath(score1, true);
                    setPathState(2);
                }
                break;

            case 2:
                // SHOOT ALL 3 PRELOADS, then go to intake line
                if (!follower.isBusy()) {

                    if (!shootingSequenceStarted) {
                        shootingSequenceStarted = true;
                        actionTimer.resetTimer();
                    }

                    // Wait until FSM finishes 3-ball sequence OR timeout
                    if (fsm.isFinished()) {
                        shootingSequenceStarted = false;

                        // Reset mechanisms so we don't outtake remaining balls
                        shooter.setTargetVekocity(0);  // stop shooter wheel
                        intake.intakeOff();            // intake off
                        lever.leverDown();             // neutral lever
                        sorter.setSorterTarget(0);     // safe/hold position

                        follower.followPath(goToEnd, true);
                        setPathState(3);
                    }
                }
                break;

            case 3:
                // FIRST INTAKE POSITION
                if (!follower.isBusy()) {
                    // Step 1: after path finishes, wait then move sorter ONLY
                    if (!justCompletedPath && !sorterWaiting) {
                        justCompletedPath = true;
                        actionTimer.resetTimer();
                    }

                    if (justCompletedPath && actionTimer.getElapsedTimeSeconds() > WAIT_FOR_POCKET) {
                        justCompletedPath = false;
                        sorter.setSorterTarget(89.6);  // pocket for first new ball
                        sorterWaiting = true;
                        actionTimer.resetTimer();
                    }

                    // Step 2: after sorter moves, turn intake on and go to next ball
                    if (sorterWaiting && actionTimer.getElapsedTimeSeconds() > INTAKE_BALL_WAIT) {
                        sorterWaiting = false;
                        intake.intakeOn();
                        follower.followPath(intakeBall1, true);
                        setPathState(8);
                    }
                }
                break;
            case 8:
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
        shooter.PIDFShootingLoop();
        sorter.PIDFSorterLoop();
        pitch.pitchDown();

        follower.update();
        autonomousPathUpdate();

        fsm.run(opmodeTimer.getElapsedTimeSeconds(), shootingSequenceStarted);

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();

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

        fsm = new ShooterSorterLeverFSM(shooter, sorter, lever);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {}
}
