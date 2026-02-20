package org.firstinspires.ftc.teamcode.turret;

/**
 * TurretMotorConfig
 *
 * Single source of truth for all turret/centering tuning parameters.
 * Edit values here — no need to touch any other file.
 *
 * Hardware references
 * -------------------
 *   Motor  : GoBilda 5203-2402-0014  (435 RPM, 384.5 PPR at output shaft)
 *   Camera : Logitech C270  ("Webcam 1")
 *   Target : AprilTag ID 24, family 36h11 (8 in outer / 6.5 in dark inner)
 */
public final class TurretMotorConfig {

    // -------------------------------------------------------------------------
    // Motor / camera hardware names (must match Driver Station config)
    // -------------------------------------------------------------------------
    public static final String MOTOR_NAME  = "turret";
    public static final String CAMERA_NAME = "Webcam 1";

    // -------------------------------------------------------------------------
    // GoBilda 5203-2402-0014 encoder spec
    // -------------------------------------------------------------------------
    /** Encoder counts per full 360° revolution at the output shaft. */
    public static final double TICKS_PER_REV = 384.5;

    // -------------------------------------------------------------------------
    // Physical travel limits
    // -------------------------------------------------------------------------
    /** Maximum turret travel from centre, in degrees (±45°). */
    public static final double MAX_ANGLE_DEG = 45.0;

    /** Encoder counts that correspond to MAX_ANGLE_DEG. (~48 ticks) */
    public static final double MAX_TICKS = (MAX_ANGLE_DEG / 360.0) * TICKS_PER_REV;

    // -------------------------------------------------------------------------
    // AprilTag physical dimensions (inches)
    // -------------------------------------------------------------------------
    public static final double TAG_SIZE_INCHES        = 8.0;   // outer square
    public static final double TAG_BORDER_SIZE_INCHES = 6.5;   // dark inner square

    // -------------------------------------------------------------------------
    // Vision pipeline
    // -------------------------------------------------------------------------
    /** Pixel column that represents "centred" — half the stream width. */
    public static final double IMAGE_CENTER_X = 320.0;   // 640 × 480 stream

    /**
     * Dead-band: motor is completely silent when the smoothed error is
     * within this many pixels of centre.
     */
    public static final double CENTERING_DEADBAND_PX = 20.0;

    /**
     * Exponential moving-average smoothing factor for the raw pixel error.
     *
     *   smoothed = ALPHA * raw + (1 - ALPHA) * previousSmoothed
     *
     * Lowered from 0.25 → 0.12 so the filter absorbs more frame-to-frame
     * detection noise before it reaches the PID and the slew limiter.
     *
     * Increase toward 0.20 if tracking feels too sluggish on fast movement.
     * Decrease toward 0.08 if there is still high-frequency jitter.
     */
    public static final double ERROR_SMOOTHING_ALPHA = 0.12;

    // -------------------------------------------------------------------------
    // PID gains  (as set by the user)
    // -------------------------------------------------------------------------
    public static final double KP = 0.0005;
    public static final double KI = 0.00005;
    public static final double KD = 0.0008;

    // -------------------------------------------------------------------------
    // Integrator anti-windup clamp (pixel units)
    // -------------------------------------------------------------------------
    public static final double INTEGRAL_CLAMP = 150.0;

    // -------------------------------------------------------------------------
    // Motor output limits
    // -------------------------------------------------------------------------
    /** Absolute maximum power sent to the turret motor. */
    public static final double MAX_MOTOR_POWER = 0.65;

    /**
     * Minimum motor power threshold.
     *
     * IMPORTANT: this is no longer applied as a hard step (0 → MIN instantly).
     * Instead it is used only as the floor inside the slew ramp so the motor
     * never stalls at a tiny power level.  The slew limiter controls how
     * quickly the output rises from 0 to this value, eliminating the jerk.
     *
     * If the turret stalls under friction, raise this slightly (e.g. 0.06).
     */
    public static final double MIN_MOTOR_POWER = 0.04;

    /**
     * Slew rate limit — maximum change in motor power per second.
     *
     * This is the primary smoothness control. It prevents the motor command
     * from jumping abruptly between any two values in a single loop tick.
     *
     *   At 0.60/s the output ramps from 0 → 0.60 in 1 second.
     *   A typical control loop runs ~20 ms, so the per-tick cap is:
     *       0.60 * 0.020 = 0.012 power units per loop
     *
     * Increase toward 1.0/s for faster (but potentially jerkier) response.
     * Decrease toward 0.35/s for a slower, silkier ramp.
     */
    public static final double SLEW_RATE_PER_SECOND = 0.60;

    /**
     * Soft-stop buffer in ticks.
     * Power is linearly ramped to zero in the last N ticks before each limit.
     */
    public static final double SOFT_STOP_BUFFER_TICKS = 10.0;

    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------
    /** Driver Station telemetry update interval (ms). */
    public static final long TELEMETRY_UPDATE_MS = 50;

    // Prevent instantiation
    private TurretMotorConfig() {}
}
