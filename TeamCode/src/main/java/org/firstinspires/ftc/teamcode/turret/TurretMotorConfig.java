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
 *   Target : AprilTag ID 24, family 36h11 (8 in outer / 6.5 in inner)
 */
public final class TurretMotorConfig {

    // -------------------------------------------------------------------------
    // Motor hardware name (must match Driver Station / Control Hub config)
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
    /**
     * Maximum turret travel from center position, in degrees.
     * 45° left / 45° right  →  ±45°
     */
    public static final double MAX_ANGLE_DEG = 45.0;

    /** Encoder counts that correspond to MAX_ANGLE_DEG. */
    public static final double MAX_TICKS =
            (MAX_ANGLE_DEG / 360.0) * TICKS_PER_REV;   // ≈ 48.06 ticks

    // -------------------------------------------------------------------------
    // AprilTag physical dimensions  (inches)
    // -------------------------------------------------------------------------
    public static final double TAG_SIZE_INCHES = 8.0;      // outer square
    
    public static final double TAG_BORDER_SIZE_INCHES = 6.5;   // dark inner square

    // -------------------------------------------------------------------------
    // Vision pipeline
    // -------------------------------------------------------------------------
    /** Pixel column that is "centred" — set to half your stream width. */
    public static final double IMAGE_CENTER_X = 320.0;   // 640 × 480 stream

    /**
     * Dead-band: if the tag's horizontal offset (px) is within this window
     * the motor will not move.  Prevents jitter on a well-centred tag.
     */
    public static final double CENTERING_DEADBAND_PX = 10.0;

    // -------------------------------------------------------------------------
    // PID gains  (all three loops share this one set — tune here)
    // -------------------------------------------------------------------------
    /**
     * Proportional gain.
     * Start small (e.g. 0.003) and increase until the turret snaps to centre
     * quickly without overshooting.
     */
    public static final double KP = 0.003;

    /**
     * Integral gain.
     * Helps eliminate steady-state offset.  Keep very small to avoid windup.
     */
    public static final double KI = 0.0002;

    /**
     * Derivative gain.
     * Damps oscillation / overshoot.  Increase if the turret hunts.
     */
    public static final double KD = 0.0004;

    // -------------------------------------------------------------------------
    // Integrator anti-windup clamp  (raw-pixel units)
    // -------------------------------------------------------------------------
    public static final double INTEGRAL_CLAMP = 200.0;

    // -------------------------------------------------------------------------
    // Motor output limits  [0.0 – 1.0]
    // -------------------------------------------------------------------------
    /** Absolute maximum power sent to the turret motor. */
    public static final double MAX_MOTOR_POWER = 0.80;

    /**
     * Minimum "kick" power needed to overcome static friction.
     * If the PID output magnitude is below this and the error is outside the
     * dead-band, the motor receives at least this power (signed).
     */
    public static final double MIN_MOTOR_POWER = 0.08;

    /**
     * Soft-stop buffer in ticks.
     * When the encoder is within this many ticks of a hard limit the maximum
     * allowable power in that direction is ramped down linearly to zero.
     */
    public static final double SOFT_STOP_BUFFER_TICKS = 10.0;

    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------
    /** Update interval for Driver Station telemetry, in milliseconds. */
    public static final long TELEMETRY_UPDATE_MS = 50;

    // Prevent instantiation
    private TurretMotorConfig() {}
}
