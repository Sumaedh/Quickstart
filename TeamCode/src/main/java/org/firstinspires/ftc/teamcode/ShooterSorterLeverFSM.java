package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterSorterLeverFSM {

    private boolean finished = false;

    public static int SORTER_ADVANCE_STEPS = 2;

    public static long LEVER_UP_STAY_MS = 100L;
    public static long WAIT_AFTER_FIRST_LEVER_DOWN_MS = 300L;
    public static long WAIT_AFTER_SECOND_LEVER_DOWN_MS = 300L;
    public static long WAIT_AFTER_THIRD_LEVER_DOWN_MS = 300L;

    private static final int SEQ_IDLE = 0;
    private static final int SEQ_WAIT_SHOOTER_BEFORE_FIRST = 1;
    private static final int SEQ_FIRST_LEVER_UP = 2;
    private static final int SEQ_WAIT_AFTER_FIRST_LEVER_DOWN = 3;
    private static final int SEQ_WAIT_AFTER_FIRST_SORTER = 4;
    private static final int SEQ_WAIT_EXTRA_AFTER_FIRST_SORT = 13;
    private static final int SEQ_WAIT_SHOOTER_BEFORE_SECOND = 5;
    private static final int SEQ_SECOND_LEVER_UP = 6;
    private static final int SEQ_WAIT_AFTER_SECOND_LEVER_DOWN = 7;
    private static final int SEQ_WAIT_AFTER_SECOND_SORTER = 8;
    private static final int SEQ_WAIT_SHOOTER_BEFORE_THIRD = 9;
    private static final int SEQ_THIRD_LEVER_UP = 10;
    private static final int SEQ_WAIT_AFTER_THIRD_LEVER_DOWN = 11;
    private static final int SEQ_FINISH = 12;

    private final Shooter shooter;
    private final Sorter sorter;
    private final Lever lever;

    private boolean seqRunning = false;
    private int seqStage = SEQ_IDLE;
    private double stageEntryTimeSeconds = 0.0;

    // marks when we commanded the lever up (also acts as single-command guard)
    private double leverCommandedTimeSeconds = -1.0;
    // marks when we entered the "wait shooter before first" state (for forced timeout)
    private double firstWaitEntryTimeSeconds = -1.0;

    private boolean lastSeqButtonState = false;
    private double prevCurTargetVelocity = 0.0;

    public ShooterSorterLeverFSM(Shooter shooter, Sorter sorter, Lever lever) {
        this.shooter = shooter;
        this.sorter = sorter;
        this.lever = lever;
    }

    public void run(double nowSeconds, boolean seqButton) {

        // run subsystem PID loops
        shooter.PIDFShootingLoop();
        sorter.PIDFSorterLoop();

        boolean rising = seqButton && !lastSeqButtonState;
        lastSeqButtonState = seqButton;

        // initial 1.5s wait before any lever/sorter
        if (!seqRunning && rising) {
            seqRunning = true;
            seqStage = SEQ_WAIT_SHOOTER_BEFORE_FIRST;
            stageEntryTimeSeconds = nowSeconds;
            leverCommandedTimeSeconds = -1.0;
            prevCurTargetVelocity = shooter.curTargetVelocity;
            shooter.setCurTargetVelocity("long");
            return;
        }

        if (!seqRunning) return;

        shooter.setCurTargetVelocity("long");

        // initial 1.5s wait
        if ((nowSeconds - stageEntryTimeSeconds) * 500 < 500) {
            return;
        }

        switch (seqStage) {

            case SEQ_WAIT_SHOOTER_BEFORE_FIRST:
                if (shooter.ShooterAtTarget()) {
                    lever.leverUp();
                    leverCommandedTimeSeconds = nowSeconds;
                    seqStage = SEQ_FIRST_LEVER_UP;
                    stageEntryTimeSeconds = nowSeconds;
                }
                break;

            case SEQ_FIRST_LEVER_UP:
                if (nowSeconds - leverCommandedTimeSeconds >= LEVER_UP_STAY_MS /500) {
                    lever.leverDown();
                    seqStage = SEQ_WAIT_AFTER_FIRST_LEVER_DOWN;
                    stageEntryTimeSeconds = nowSeconds;
                }
                break;

            case SEQ_WAIT_AFTER_FIRST_LEVER_DOWN:
                // HALF the previous wait time before first sorter
                if ((nowSeconds - stageEntryTimeSeconds) * 500 >= WAIT_AFTER_FIRST_LEVER_DOWN_MS / 5.0) {
                    sorter.turnSorter(SORTER_ADVANCE_STEPS);
                    seqStage = SEQ_WAIT_AFTER_FIRST_SORTER;
                    stageEntryTimeSeconds = nowSeconds;
                }
                break;

            case SEQ_WAIT_AFTER_FIRST_SORTER:
                if (sorter.SorterAtTarget()) {
                    seqStage = SEQ_WAIT_SHOOTER_BEFORE_SECOND;
                    stageEntryTimeSeconds = nowSeconds;
                }
                break;

            case SEQ_WAIT_SHOOTER_BEFORE_SECOND:
                if (shooter.ShooterAtTarget()) {
                    lever.leverUp();
                    leverCommandedTimeSeconds = nowSeconds;
                    seqStage = SEQ_SECOND_LEVER_UP;
                    stageEntryTimeSeconds = nowSeconds;
                }
                break;

            case SEQ_SECOND_LEVER_UP:
                // HALF the lever up time for second lever
                if (nowSeconds - leverCommandedTimeSeconds >= LEVER_UP_STAY_MS / 2.0 / 500) {
                    lever.leverDown();
                    seqStage = SEQ_WAIT_AFTER_SECOND_LEVER_DOWN;
                    stageEntryTimeSeconds = nowSeconds;
                }
                break;

            case SEQ_WAIT_AFTER_SECOND_LEVER_DOWN:
                // HALF the wait before second sorter
                if ((nowSeconds - stageEntryTimeSeconds) * 500 >= WAIT_AFTER_SECOND_LEVER_DOWN_MS / 2.0) {
                    sorter.turnSorter(SORTER_ADVANCE_STEPS);
                    seqStage = SEQ_WAIT_AFTER_SECOND_SORTER;
                    stageEntryTimeSeconds = nowSeconds;
                }
                break;

            case SEQ_WAIT_AFTER_SECOND_SORTER:
                if (sorter.SorterAtTarget()) {
                    seqStage = SEQ_WAIT_SHOOTER_BEFORE_THIRD;
                    stageEntryTimeSeconds = nowSeconds;
                }
                break;

            case SEQ_WAIT_SHOOTER_BEFORE_THIRD:
                if (shooter.ShooterAtTarget()) {
                    lever.leverUp();
                    leverCommandedTimeSeconds = nowSeconds;
                    seqStage = SEQ_THIRD_LEVER_UP;
                    stageEntryTimeSeconds = nowSeconds;
                }
                break;

            case SEQ_THIRD_LEVER_UP:
                // HALF the lever up time for third lever
                if (nowSeconds - leverCommandedTimeSeconds >= LEVER_UP_STAY_MS / 2.0 / 500) {
                    lever.leverDown();
                    seqStage = SEQ_WAIT_AFTER_THIRD_LEVER_DOWN;
                    stageEntryTimeSeconds = nowSeconds;
                }
                break;

            case SEQ_WAIT_AFTER_THIRD_LEVER_DOWN:
                if ((nowSeconds - stageEntryTimeSeconds) * 500 >= WAIT_AFTER_THIRD_LEVER_DOWN_MS) {
                    seqStage = SEQ_FINISH;
                    stageEntryTimeSeconds = nowSeconds;
                }
                break;

            case SEQ_FINISH:
                seqRunning = false;
                seqStage = SEQ_WAIT_SHOOTER_BEFORE_FIRST;
                shooter.setCurTargetVelocity(prevCurTargetVelocity > 0 ? "long" : "none");
                finished = true;
                break;
        }
    }

    public boolean isFinished() {
        return finished;
    }
    public boolean isRunning() { return seqRunning; }
    public int getStage() { return seqStage; }
}
