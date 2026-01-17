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

@Autonomous(name = "Red Alliance Auto 6-Ball")
public class RedAllianceAuto6Ball extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private boolean justCompletedPath = false; // Flag to handle wait after path completion
    private boolean sorterWaiting = false;

    Intake intake = new Intake();
    Lever lever = new Lever();
    Pitch pitch = new Pitch();
    Shooter shooter = new Shooter();
    Sorter sorter = new Sorter();

    // TODO: INITIALIZING POSES
    private final Pose startPose = new Pose(88.389, 6.832, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose secondPose = new Pose(88.24324324324324, 19.832432432432437, Math.toRadians(90));
    private final Pose scorePose = new Pose(84.07567567567567, 19.848648648648634, Math.toRadians(63)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup3Pose = new Pose(101.87567567567568, 36.18378378378379, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup3PoseIntake1 = new Pose(108.37837837837839, 36.18378378378379, Math.toRadians(0));
    private final Pose pickup3PoseIntake2 = new Pose(113.7081081081081, 36.18378378378379, Math.toRadians(0));
    private final Pose pickup3PoseIntake3 = new Pose(118.52972972972972, 36.18378378378379, Math.toRadians(0));
    private final Pose endPose = new Pose(84.22162162162162, 45.39999999999999, Math.toRadians(63));

    private Path startPreload;
    private PathChain score1, alignToBalls, intakeBall1, intakeBall2, intakeBall3, score2, goToEnd;

    // TODO: PATH INITIALIZING
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        startPreload = new Path(new BezierLine(startPose, secondPose));
        startPreload.setConstantHeadingInterpolation(secondPose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

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

    // TODO: MANAGING PATH STATES (FSM)
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(startPreload);
                setPathState(1);
                break;
            case 1:
                /* You could check for - Follower State: "if(!follower.isBusy()) {}" - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}" - Robot Position: "if(follower.getPose().getX() > 36) {}" */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    // shooter.setCurTargetVelocity("long");
                    follower.followPath(score1,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    // TODO: DO SHOOTING SEQUENCE
                    follower.followPath(alignToBalls,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    if (!justCompletedPath && !sorterWaiting) {
                        justCompletedPath = true;
                        actionTimer.resetTimer();
                    }
                    if (justCompletedPath && actionTimer.getElapsedTimeSeconds() > 1.25) {
                        justCompletedPath = false;
                        sorter.setSorterTarget(89.6);
                        intake.intakeOn();
                        actionTimer.resetTimer();
                        sorterWaiting = true;
                    }
                    if (sorterWaiting && actionTimer.getElapsedTimeSeconds() > 0.75) {
                        sorterWaiting = false;
                        // Wait for sorter to reach target before continuing
                        follower.followPath(intakeBall1, true);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    if (!justCompletedPath && !sorterWaiting) {
                        justCompletedPath = true;
                        actionTimer.resetTimer();
                    }
                    if (justCompletedPath && actionTimer.getElapsedTimeSeconds() > 1.25) {
                        justCompletedPath = false;
                        sorter.setSorterTarget(268.8);
                        intake.intakeOn();
                        actionTimer.resetTimer();
                        sorterWaiting = true;
                    }
                    if (sorterWaiting && actionTimer.getElapsedTimeSeconds() > 0.75) {
                        sorterWaiting = false;
                        /* Grab Sample */
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(intakeBall2, true);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    if (!justCompletedPath && !sorterWaiting) {
                        justCompletedPath = true;
                        actionTimer.resetTimer();
                    }
                    if (justCompletedPath && actionTimer.getElapsedTimeSeconds() > 1.25) {
                        justCompletedPath = false;
                        sorter.setSorterTarget(448);
                        intake.intakeOn();
                        actionTimer.resetTimer();
                        sorterWaiting = true;
                    }
                    if (sorterWaiting && actionTimer.getElapsedTimeSeconds() > 0.75) {
                        sorterWaiting = false;
                        /* Score Sample */
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        follower.followPath(intakeBall3, true);
                        setPathState(6);
                    }
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    sorter.setSorterTarget(537.6);
                    intake.intakeOff();
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    intake.intakeOff();
                    follower.followPath(score2, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(goToEnd, true);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
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
     This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
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

    /**
     This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {}

    /**
     This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     We do not use this because everything should automatically able
     **/
    @Override
    public void stop() {}
}