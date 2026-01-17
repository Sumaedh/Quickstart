package org.firstinspires.ftc.teamcode;

/**
 * ShooterSorterLeverFSM — improved timing robustness.
 *
 * Uses the original Shooter, Sorter, Lever classes (unchanged).
 * Public function: run(nowSeconds, seqButton).
 */
public class ShooterSorterLeverFSM {

    // Constants copied from your TeleopOld
    private final int SORTER_ADVANCE_STEPS = 2;
    private final double SEQ_SHOOTER_VELOCITY = 1680.0;
    private final double SHOOTER_TOLERANCE = 40.0;

    private final long LEVER_UP_STAY_MS = 150L;
    private final long WAIT_AFTER_FIRST_LEVER_DOWN_MS = 1100L;
    private final long WAIT_AFTER_FIRST_SORTER_MS = 500L;
    private final long WAIT_AFTER_SECOND_LEVER_DOWN_MS = 1500L;
    private final long WAIT_AFTER_SECOND_SORTER_MS = 850L;
    private final long WAIT_AFTER_THIRD_LEVER_DOWN_MS = 2000L;

    // Because your Lever class has no getter, we still assume lever move time.
    private final long LEVER_MOVE_ASSUMED_MS = 80L;

    // New safety/tuning: require shooter to be stable at target for this long before levers fire
    private final long SHOOTER_STABLE_MIN_MS = 100L; // must be at target continuously for 100ms

    // Allow small extension of lever hold if shooter dips briefly
    private final long LEVER_HOLD_MAX_EXTENSION_MS = 300L;

    // FSM states (same as TeleopOld)
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

    // Subsystems (unchanged classes you provided)
    private final Shooter shooter;
    private final Sorter sorter;
    private final Lever lever;

    // FSM internals
    private boolean seqRunning = false;
    private int seqStage = SEQ_IDLE;
    private double stageEntryTimeSeconds = 0.0;

    // When lever was commanded up
    private double leverCommandedTimeSeconds = 0.0;
    // When shooter first observed at target continuously
    private double shooterStableSinceSeconds = -1.0;

    private boolean lastSeqButtonState = false;
    private double prevCurTargetVelocity = 0.0;

    public ShooterSorterLeverFSM(Shooter shooter, Sorter sorter, Lever lever) {
        this.shooter = shooter;
        this.sorter = sorter;
        this.lever = lever;
    }

    /**
     * Single public method. Call once per loop.
     *
     * @param nowSeconds monotonic time in seconds (System.currentTimeMillis()/1000.0)
     * @param seqButton  whether the sequence start button (gamepad2.y) is pressed
     */
    public void run(double nowSeconds, boolean seqButton) {
        // run PID loops for subsystems (FSM drives them)
        shooter.PIDFShootingLoop();
        sorter.PIDFSorterLoop();

        // rising edge detection for starting the sequence
        boolean rising = seqButton && !lastSeqButtonState;
        lastSeqButtonState = seqButton;

        if (rising && !seqRunning) {
            seqRunning = true;
            seqStage = SEQ_WAIT_SHOOTER_BEFORE_FIRST;
            stageEntryTimeSeconds = nowSeconds;
            shooterStableSinceSeconds = -1.0;
            prevCurTargetVelocity = shooter.curTargetVelocity;
            // force shooter to long shot target
            shooter.setCurTargetVelocity("long");
        }

        // If not running, nothing else to do
        if (!seqRunning) return;

        // Keep shooter target forced while seq runs
        shooter.setCurTargetVelocity("long");

        switch (seqStage) {

            case SEQ_WAIT_SHOOTER_BEFORE_FIRST:
                // track continuous shooter-stable time
                if (shooter.ShooterAtTarget()) {
                    if (shooterStableSinceSeconds < 0.0) shooterStableSinceSeconds = nowSeconds;
                    // require continuous stable time before firing
                    if ((nowSeconds - shooterStableSinceSeconds) * 1000.0 >= SHOOTER_STABLE_MIN_MS) {
                        // shooter stable long enough -> command lever up
                        lever.leverUp();
                        leverCommandedTimeSeconds = nowSeconds;
                        seqStage = SEQ_FIRST_LEVER_UP;
                        stageEntryTimeSeconds = nowSeconds;
                    }
                } else {
                    // reset stable timer
                    shooterStableSinceSeconds = -1.0;
                }
                break;

            case SEQ_FIRST_LEVER_UP:
                // Wait for lever to move (assumed) + hold, but ensure shooter stayed near target while holding.
                double elapsedSinceLeverCmdMs = (nowSeconds - leverCommandedTimeSeconds) * 1000.0;
                if (elapsedSinceLeverCmdMs >= LEVER_MOVE_ASSUMED_MS) {
                    // we've reached the point where lever has (likely) moved; start hold period timer reference
                    double holdElapsedMs = elapsedSinceLeverCmdMs - LEVER_MOVE_ASSUMED_MS;
                    // During hold, require shooter at target; if it dips, allow brief extension up to LEVER_HOLD_MAX_EXTENSION_MS
                    if (shooter.ShooterAtTarget()) {
                        // OK: shooter good; if full hold completed, move on
                        if (holdElapsedMs >= LEVER_UP_STAY_MS) {
                            lever.leverDown();
                            seqStage = SEQ_WAIT_AFTER_FIRST_LEVER_DOWN;
                            stageEntryTimeSeconds = nowSeconds;
                        }
                    } else {
                        // shooter dipped; allow extension window
                        if (holdElapsedMs >= (LEVER_UP_STAY_MS + LEVER_HOLD_MAX_EXTENSION_MS)) {
                            // still not back up — bail and proceed (safer than locking forever)
                            lever.leverDown();
                            seqStage = SEQ_WAIT_AFTER_FIRST_LEVER_DOWN;
                            stageEntryTimeSeconds = nowSeconds;
                        }
                        // else: wait a bit longer for shooter to recover
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
                    // reset shooter stable tracker so we require fresh stable time for next shot
                    shooterStableSinceSeconds = -1.0;
                    seqStage = SEQ_WAIT_SHOOTER_BEFORE_SECOND;
                    stageEntryTimeSeconds = nowSeconds;
                }
                break;

            case SEQ_WAIT_SHOOTER_BEFORE_SECOND:
                if (shooter.ShooterAtTarget()) {
                    if (shooterStableSinceSeconds < 0.0) shooterStableSinceSeconds = nowSeconds;
                    if ((nowSeconds - shooterStableSinceSeconds) * 1000.0 >= SHOOTER_STABLE_MIN_MS) {
                        lever.leverUp();
                        leverCommandedTimeSeconds = nowSeconds;
                        seqStage = SEQ_SECOND_LEVER_UP;
                        stageEntryTimeSeconds = nowSeconds;
                    }
                } else {
                    shooterStableSinceSeconds = -1.0;
                }
                break;

            case SEQ_SECOND_LEVER_UP:
                elapsedSinceLeverCmdMs = (nowSeconds - leverCommandedTimeSeconds) * 1000.0;
                if (elapsedSinceLeverCmdMs >= LEVER_MOVE_ASSUMED_MS) {
                    double holdElapsedMs = elapsedSinceLeverCmdMs - LEVER_MOVE_ASSUMED_MS;
                    if (shooter.ShooterAtTarget()) {
                        if (holdElapsedMs >= LEVER_UP_STAY_MS) {
                            lever.leverDown();
                            seqStage = SEQ_WAIT_AFTER_SECOND_LEVER_DOWN;
                            stageEntryTimeSeconds = nowSeconds;
                        }
                    } else {
                        if (holdElapsedMs >= (LEVER_UP_STAY_MS + LEVER_HOLD_MAX_EXTENSION_MS)) {
                            lever.leverDown();
                            seqStage = SEQ_WAIT_AFTER_SECOND_LEVER_DOWN;
                            stageEntryTimeSeconds = nowSeconds;
                        }
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
                    shooterStableSinceSeconds = -1.0;
                    seqStage = SEQ_WAIT_SHOOTER_BEFORE_THIRD;
                    stageEntryTimeSeconds = nowSeconds;
                }
                break;

            case SEQ_WAIT_SHOOTER_BEFORE_THIRD:
                if (shooter.ShooterAtTarget()) {
                    if (shooterStableSinceSeconds < 0.0) shooterStableSinceSeconds = nowSeconds;
                    if ((nowSeconds - shooterStableSinceSeconds) * 1000.0 >= SHOOTER_STABLE_MIN_MS) {
                        lever.leverUp();
                        leverCommandedTimeSeconds = nowSeconds;
                        seqStage = SEQ_THIRD_LEVER_UP;
                        stageEntryTimeSeconds = nowSeconds;
                    }
                } else {
                    shooterStableSinceSeconds = -1.0;
                }
                break;

            case SEQ_THIRD_LEVER_UP:
                elapsedSinceLeverCmdMs = (nowSeconds - leverCommandedTimeSeconds) * 1000.0;
                if (elapsedSinceLeverCmdMs >= LEVER_MOVE_ASSUMED_MS) {
                    double holdElapsedMs = elapsedSinceLeverCmdMs - LEVER_MOVE_ASSUMED_MS;
                    if (shooter.ShooterAtTarget()) {
                        if (holdElapsedMs >= LEVER_UP_STAY_MS) {
                            lever.leverDown();
                            seqStage = SEQ_WAIT_AFTER_THIRD_LEVER_DOWN;
                            stageEntryTimeSeconds = nowSeconds;
                        }
                    } else {
                        if (holdElapsedMs >= (LEVER_UP_STAY_MS + LEVER_HOLD_MAX_EXTENSION_MS)) {
                            lever.leverDown();
                            seqStage = SEQ_WAIT_AFTER_THIRD_LEVER_DOWN;
                            stageEntryTimeSeconds = nowSeconds;
                        }
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
                // restore shooter velocity as previous code did
                shooter.curTargetVelocity = prevCurTargetVelocity;
                seqRunning = false;
                seqStage = SEQ_IDLE;
                stageEntryTimeSeconds = nowSeconds;
                break;

            default:
                // safety fallback
                seqRunning = false;
                seqStage = SEQ_IDLE;
                stageEntryTimeSeconds = nowSeconds;
                break;
        }
    }

    // small helpers for testing
    public boolean isRunning() { return seqRunning; }
    public int getStage() { return seqStage; }
}
