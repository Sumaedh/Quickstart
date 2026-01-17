package org.firstinspires.ftc.teamcode;

public class ShooterSorterLeverFSM {

    // Timing/constants — copied from your TeleopOld values
    private final int SORTER_ADVANCE_STEPS = 2;
    private final double SEQ_SHOOTER_VELOCITY = 1680.0;
    private final double SHOOTER_TOLERANCE = 40.0;

    private final long LEVER_UP_STAY_MS = 150L;
    private final long WAIT_AFTER_FIRST_LEVER_DOWN_MS = 1100L;
    private final long WAIT_AFTER_FIRST_SORTER_MS = 500L;
    private final long WAIT_AFTER_SECOND_LEVER_DOWN_MS = 1500L;
    private final long WAIT_AFTER_SECOND_SORTER_MS = 850L;
    private final long WAIT_AFTER_THIRD_LEVER_DOWN_MS = 2000L;

    // Because Lever has no getter in your provided class, we assume lever moves within this ms
    private final long LEVER_MOVE_ASSUMED_MS = 80L;

    // States (same as TeleopOld)
    private static final int SEQ_IDLE = 0;
    private static final int SEQ_WAIT_SHOOTER_BEFORE_FIRST = 1;
    private static final int SEQ_FIRST_LEVER_UP = 2;
    private static final int SEQ_WAIT_AFTER_FIRST_LEVER_DOWN = 3;
    private static final int SEQ_WAIT_AFTER_FIRST_SORTER = 4;
    private static final int SEQ_WAIT_SHOOTER_BEFORE_SECOND = 5;
    private static final int SEQ_SECOND_LEVER_UP = 6;
    private static final int SEQ_WAIT_AFTER_SECOND_LEVER_DOWN = 7;
    private static final int SEQ_WAIT_AFTER_SECOND_SORTER = 8;
    private static final int SEQ_WAIT_SHOOTER_BEFORE_THIRD = 9;
    private static final int SEQ_THIRD_LEVER_UP = 10;
    private static final int SEQ_WAIT_AFTER_THIRD_LEVER_DOWN = 11;
    private static final int SEQ_FINISH = 12;

    // Subsystems provided by you (must be initialized before using FSM)
    private final Shooter shooter;
    private final Sorter sorter;
    private final Lever lever;

    // FSM internals
    private boolean seqRunning = false;
    private int seqStage = SEQ_IDLE;
    private double stageEntryTimeSeconds = 0.0;
    private double leverCommandedTimeSeconds = 0.0; // when we commanded leverUp
    private boolean lastSeqButtonState = false;
    private double prevCurTargetVelocity = 0.0; // saved shooter velocity to restore at end

    public ShooterSorterLeverFSM(Shooter shooter, Sorter sorter, Lever lever) {
        this.shooter = shooter;
        this.sorter = sorter;
        this.lever = lever;
    }

    /**
     * Single public run function. Call this every loop.
     * @param nowSeconds monotonic time in seconds (System.currentTimeMillis()/1000.0)
     * @param seqButton whether the start button is currently pressed (rising edge starts sequence)
     */
    public void run(double nowSeconds, boolean seqButton) {
        // Run subsystem control loops here (FSM drives them)
        shooter.PIDFShootingLoop();
        sorter.PIDFSorterLoop();

        // Rising-edge detect
        boolean rising = seqButton && !lastSeqButtonState;
        lastSeqButtonState = seqButton;

        if (rising && !seqRunning) {
            // start sequence
            seqRunning = true;
            seqStage = SEQ_WAIT_SHOOTER_BEFORE_FIRST;
            stageEntryTimeSeconds = nowSeconds;
            // save previous shooter target so we can restore on finish
            prevCurTargetVelocity = shooter.curTargetVelocity;
            // force shooter target to "long" as original sequence
            shooter.setCurTargetVelocity("long");
        }

        // if sequence not running, nothing to do
        if (!seqRunning) return;

        // while running, ensure shooter target held
        shooter.setCurTargetVelocity("long");

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
                // We assume lever reaches commanded position after LEVER_MOVE_ASSUMED_MS
                if ((nowSeconds - leverCommandedTimeSeconds) * 1000.0 >= LEVER_MOVE_ASSUMED_MS) {
                    // now begin hold time
                    if ((nowSeconds - leverCommandedTimeSeconds) * 1000.0 >= (LEVER_MOVE_ASSUMED_MS + LEVER_UP_STAY_MS)) {
                        lever.leverDown();
                        seqStage = SEQ_WAIT_AFTER_FIRST_LEVER_DOWN;
                        stageEntryTimeSeconds = nowSeconds;
                    }
                }
                break;

            case SEQ_WAIT_AFTER_FIRST_LEVER_DOWN:
                if ((nowSeconds - stageEntryTimeSeconds) * 1000.0 >= WAIT_AFTER_FIRST_LEVER_DOWN_MS) {
                    sorter.turnSorter(SORTER_ADVANCE_STEPS);
                    seqStage = SEQ_WAIT_AFTER_FIRST_SORTER;
                    stageEntryTimeSeconds = nowSeconds;
                }
                break;

            case SEQ_WAIT_AFTER_FIRST_SORTER:
                if ((nowSeconds - stageEntryTimeSeconds) * 1000.0 >= WAIT_AFTER_FIRST_SORTER_MS) {
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
                if ((nowSeconds - leverCommandedTimeSeconds) * 1000.0 >= LEVER_MOVE_ASSUMED_MS) {
                    if ((nowSeconds - leverCommandedTimeSeconds) * 1000.0 >= (LEVER_MOVE_ASSUMED_MS + LEVER_UP_STAY_MS)) {
                        lever.leverDown();
                        seqStage = SEQ_WAIT_AFTER_SECOND_LEVER_DOWN;
                        stageEntryTimeSeconds = nowSeconds;
                    }
                }
                break;

            case SEQ_WAIT_AFTER_SECOND_LEVER_DOWN:
                if ((nowSeconds - stageEntryTimeSeconds) * 1000.0 >= WAIT_AFTER_SECOND_LEVER_DOWN_MS) {
                    sorter.turnSorter(SORTER_ADVANCE_STEPS);
                    seqStage = SEQ_WAIT_AFTER_SECOND_SORTER;
                    stageEntryTimeSeconds = nowSeconds;
                }
                break;

            case SEQ_WAIT_AFTER_SECOND_SORTER:
                if ((nowSeconds - stageEntryTimeSeconds) * 1000.0 >= WAIT_AFTER_SECOND_SORTER_MS) {
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
                if ((nowSeconds - leverCommandedTimeSeconds) * 1000.0 >= LEVER_MOVE_ASSUMED_MS) {
                    if ((nowSeconds - leverCommandedTimeSeconds) * 1000.0 >= (LEVER_MOVE_ASSUMED_MS + LEVER_UP_STAY_MS)) {
                        lever.leverDown();
                        seqStage = SEQ_WAIT_AFTER_THIRD_LEVER_DOWN;
                        stageEntryTimeSeconds = nowSeconds;
                    }
                }
                break;

            case SEQ_WAIT_AFTER_THIRD_LEVER_DOWN:
                if ((nowSeconds - stageEntryTimeSeconds) * 1000.0 >= WAIT_AFTER_THIRD_LEVER_DOWN_MS) {
                    seqStage = SEQ_FINISH;
                    stageEntryTimeSeconds = nowSeconds;
                }
                break;

            case SEQ_FINISH:
                // restore shooter target
                shooter.curTargetVelocity = prevCurTargetVelocity;
                seqRunning = false;
                seqStage = SEQ_IDLE;
                stageEntryTimeSeconds = nowSeconds;
                break;

            default:
                // safety: reset
                seqRunning = false;
                seqStage = SEQ_IDLE;
                stageEntryTimeSeconds = nowSeconds;
                break;
        }
    }

    // lightweight getters for testing / telemetry (not required for operation)
    public boolean isRunning() { return seqRunning; }
    public int getStage() { return seqStage; }
}
