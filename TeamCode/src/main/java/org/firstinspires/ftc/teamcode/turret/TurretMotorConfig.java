package org.firstinspires.ftc.teamcode.turret;

/**
 * TurretMotorConfig — DECODE Season (2025-2026)
 *
 * Single source of truth for all camera pan motor and PID tuning constants.
 * Both RedGoalCenteringTeleOp and BlueGoalCenteringTeleOp share these values.
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │  TO TUNE: edit the values in this file only. Do NOT touch the base     │
 * │  class or the individual goal OpModes.                                  │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * Motor: GoBilda 5203-2402-0014 (435 RPM, 384.5 ticks/rev)
 * Camera hardwareMap name : "Webcam 1"
 * Motor  hardwareMap name : "turret"
 */
public final class TurretMotorConfig {

    // Prevent instantiation — this is a constants-only class
    private TurretMotorConfig() {}

    // ═══════════════════════════════════════════════════════════════════════
    //  HARDWARE
    // ═══════════════════════════════════════════════════════════════════════

    /** hardwareMap name for the webcam. */
    public static final String CAMERA_NAME     = "Webcam 1";

    /** hardwareMap name for the pan motor. */
    public static final String MOTOR_NAME      = "turret";

    /**
     * Encoder ticks per full revolution.
     * GoBilda 5203-2402-0014 spec: ((1 + 46/17)² × 28) = 384.5 PPR
     */
    public static final double TICKS_PER_REV   = 384.5;

    /**
     * Camera resolution width in pixels — Logitech C270 max resolution.
     * Frame center X = CAMERA_WIDTH / 2 = 480.
     * NOTE: The C270 does not support 1280×720. Its max is 960×720 (4:3).
     */
    public static final int    CAMERA_WIDTH    = 960;

    /** Camera resolution height in pixels — Logitech C270 max resolution. */
    public static final int    CAMERA_HEIGHT   = 720;

    // ═══════════════════════════════════════════════════════════════════════
    //  PAN LIMITS  (degrees → ticks conversion is automatic)
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Soft limit — maximum pan to the RIGHT in degrees.
     * Converted to ticks automatically; do not edit the tick constants below.
     */
    public static final double LIMIT_DEGREES_MAX =  45.0;

    /**
     * Soft limit — maximum pan to the LEFT in degrees.
     * Converted to ticks automatically; do not edit the tick constants below.
     */
    public static final double LIMIT_DEGREES_MIN = -45.0;

    /** Computed right-limit in encoder ticks. Do NOT edit directly. */
    public static final int ENCODER_LIMIT_MAX =
            (int) Math.round(degreesToTicks(LIMIT_DEGREES_MAX));

    /** Computed left-limit in encoder ticks. Do NOT edit directly. */
    public static final int ENCODER_LIMIT_MIN =
            (int) Math.round(degreesToTicks(LIMIT_DEGREES_MIN));

    // ═══════════════════════════════════════════════════════════════════════
    //  PID GAINS
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Proportional gain.
     * Raise if response is sluggish; lower if motor oscillates or overshoots.
     */
    public static final double kP = 0.030;

    /**
     * Integral gain.
     * Eliminates steady-state offset. Keep small and raise slowly.
     */
    public static final double kI = 0.0008;

    /**
     * Derivative gain.
     * Dampens overshoot. Raise if oscillation persists after kP tuning.
     */
    public static final double kD = 0.012;

    /**
     * Anti-windup clamp on the integral accumulator.
     * Prevents runaway I-term when tag is lost or error is large for a while.
     */
    public static final double INTEGRAL_LIMIT = 50.0;

    // ═══════════════════════════════════════════════════════════════════════
    //  MOTION PROFILE
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Deadband in pixels.
     * Motor stops correcting when the tag X error is within ±DEADBAND_PX.
     * Prevents micro-jitter when the tag is essentially centered.
     */
    public static final double DEADBAND_PX = 12.0;

    /**
     * Maximum motor velocity as a fraction of rated max [0.0 – 1.0].
     * Lower = smoother but slower pan corrections.
     */
    public static final double MAX_VELOCITY_FRACTION = 0.65;

    /**
     * Ramp rate — maximum power change allowed per control loop cycle [0.0 – 1.0].
     * Lower = smoother acceleration and deceleration.
     */
    public static final double RAMP_RATE = 0.04;

    /**
     * Milliseconds after losing the tag before switching to RUN_TO_POSITION
     * hold mode to actively resist physical disturbances.
     */
    public static final long HOLD_DELAY_MS = 400;

    /**
     * Power applied to hold position in RUN_TO_POSITION mode [0.0 – 1.0].
     * Should be enough to resist gravity/vibration but not cause jerk on reacquire.
     */
    public static final double HOLD_POWER = 0.3;

    /**
     * Integral bleed factor applied each loop cycle when the tag is not visible.
     * Range (0.0 – 1.0): 0.90 means the integral decays by 10% per cycle.
     * Prevents stale integral from causing drift on tag reacquisition.
     */
    public static final double INTEGRAL_BLEED = 0.90;

    // ═══════════════════════════════════════════════════════════════════════
    //  APRILTAG DIMENSIONS  (meters — DO NOT change unless tag size changes)
    // ═══════════════════════════════════════════════════════════════════════

    /** Full outer square of the AprilTag: 8.00 inches */
    public static final double TAG_SIZE_OUTER_IN = 8.0;

    /** Dark inner data region of the AprilTag: 6.50 inches */
    public static final double TAG_SIZE_INNER_IN = 6.5;

    // ═══════════════════════════════════════════════════════════════════════
    //  UTILITY
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Converts degrees of rotation to encoder ticks.
     *
     * Formula: ticks = (degrees / 360) × TICKS_PER_REV
     *
     * Examples (GoBilda 5203-2402-0014, 384.5 ticks/rev):
     *   45° →  48 ticks
     *  -45° → -48 ticks
     *   90° →  96 ticks
     */
    public static double degreesToTicks(double degrees) {
        return (degrees / 360.0) * TICKS_PER_REV;
    }
}
