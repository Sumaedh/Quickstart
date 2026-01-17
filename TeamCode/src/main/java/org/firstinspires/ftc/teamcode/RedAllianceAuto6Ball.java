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

@Autonomous(name = "Blue Auto With Actions 2")
public class RedAllianceAuto6Ball extends OpMode {

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

    // Poses
    private final Pose startPose = new Pose(88.4, 6.8, Math.toRadians(90));
    private final Pose secondPose = new Pose(88.3892, 19.8324, Math.toRadians(90));
    private final Pose scorePose = new Pose(84, 20, Math.toRadians(38));
    private final Pose pickup3Pose = new Pose(102.876, 29.584, Math.toRadians(0));
    private final Pose pickup3PoseIntake1 = new Pose(107.378, 29.584, Math.toRadians(0));
    private final Pose pickup3PoseIntake2 = new Pose(112.708, 29.584, Math.toRadians(0));
    private final Pose pickup3PoseIntake3 = new Pose(117.530, 29.584, Math.toRadians(0));
    private final Pose endPose = new Pose(83.638, 44.038, Math.toRadians(90));


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
                .build();

        intakeBall2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3PoseIntake1, pickup3PoseIntake2))
                .setConstantHeadingInterpolation(pickup3PoseIntake2.getHeading())
                .build();

        intakeBall3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3PoseIntake2, pickup3PoseIntake3))
                .setConstantHeadingInterpolation(pickup3PoseIntake3.getHeading())
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

                        follower.followPath(alignToBalls, true);
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

                    if (justCompletedPath && actionTimer.getElapsedTimeSeconds() > 1.25) {
                        justCompletedPath = false;
                        sorter.setSorterTarget(89.6);  // pocket for first new ball
                        sorterWaiting = true;
                        actionTimer.resetTimer();
                    }

                    // Step 2: after sorter moves, turn intake on and go to next ball
                    if (sorterWaiting && actionTimer.getElapsedTimeSeconds() > 0.75) {
                        sorterWaiting = false;
                        intake.intakeOn();
                        follower.followPath(intakeBall1, true);
                        setPathState(4);
                    }
                }
                break;

            case 4:
                // SECOND INTAKE POSITION
                if (!follower.isBusy()) {
                    if (!justCompletedPath && !sorterWaiting) {
                        justCompletedPath = true;
                        actionTimer.resetTimer();
                    }

                    if (justCompletedPath && actionTimer.getElapsedTimeSeconds() > 1.25) {
                        justCompletedPath = false;
                        sorter.setSorterTarget(268.8); // pocket for second new ball
                        sorterWaiting = true;
                        actionTimer.resetTimer();
                    }

                    if (sorterWaiting && actionTimer.getElapsedTimeSeconds() > 0.75) {
                        sorterWaiting = false;
                        follower.followPath(intakeBall2, true);
                        setPathState(5);
                    }
                }
                break;

            case 5:
                // THIRD INTAKE POSITION
                if (!follower.isBusy()) {
                    if (!justCompletedPath && !sorterWaiting) {
                        justCompletedPath = true;
                        actionTimer.resetTimer();
                    }

                    if (justCompletedPath && actionTimer.getElapsedTimeSeconds() > 1.25) {
                        justCompletedPath = false;
                        sorter.setSorterTarget(448);   // pocket for third new ball
                        sorterWaiting = true;
                        actionTimer.resetTimer();
                    }

                    if (sorterWaiting && actionTimer.getElapsedTimeSeconds() > 0.75) {
                        sorterWaiting = false;
                        follower.followPath(intakeBall3, true);
                        setPathState(6);
                    }
                }
                break;

            case 6:
                // Done intaking 3, go back to scorePose
                if (!follower.isBusy()) {
                    sorter.setSorterTarget(537.6); // shooting pocket
                    intake.intakeOff();
                    follower.followPath(score2, true);
                    setPathState(7);
                }
                break;

            case 7:
                // SECOND SHOOTING SEQUENCE (for the 3 newly collected balls)
                if (!follower.isBusy()) {
                    if (!shootingSequenceStarted) {
                        shootingSequenceStarted = true;
                        actionTimer.resetTimer();
                    }

                    if (fsm.isFinished()) {
                        shootingSequenceStarted = false;

                        // Reset mechanisms again after shooting
                        shooter.setTargetVekocity(0);
                        intake.intakeOff();
                        lever.leverDown();
                        sorter.setSorterTarget(0);

                        follower.followPath(goToEnd, true);
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
