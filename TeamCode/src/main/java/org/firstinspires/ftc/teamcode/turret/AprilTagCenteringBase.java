package org.firstinspires.ftc.teamcode.turret;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * AprilTagCenteringBase
 *
 * Abstract TeleOp base that:
 *  • Opens a VisionPortal with an AprilTagProcessor.
 *  • Applies exponential smoothing (EMA) to raw pixel error to suppress
 *    frame-to-frame detection noise before feeding the PID controller.
 *  • Runs a time-scaled PID loop to centre a specific AprilTag.
 *  • Passes the PID output through a slew rate limiter so the motor
 *    command can never change faster than SLEW_RATE_PER_SECOND, eliminating
 *    abrupt power steps.
 *  • Enforces encoder-based soft limits (±45° from zeroed centre).
 *  • Emits clean Driver Station telemetry.
 *
 * Subclasses only need to call super(tagId, tagLabel) and annotate
 * themselves with @TeleOp.
 *
 * All tuning lives in {@link TurretMotorConfig}.
 */
public abstract class AprilTagCenteringBase extends LinearOpMode {

    // ------------------------------------------------------------------
    // Subclass identity
    // ------------------------------------------------------------------
    private final int    targetTagId;
    private final String targetTagLabel;

    // ------------------------------------------------------------------
    // Hardware
    // ------------------------------------------------------------------
    private DcMotorEx turretMotor;

    // ------------------------------------------------------------------
    // Vision
    // ------------------------------------------------------------------
    private VisionPortal      visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    // ------------------------------------------------------------------
    // Signal pipeline state
    // ------------------------------------------------------------------
    /** Exponential moving average of the raw pixel error. */
    private double smoothedError  = 0.0;

    /** Last power value actually sent to the motor (used by slew limiter). */
    private double currentPower   = 0.0;

    // ------------------------------------------------------------------
    // PID state
    // ------------------------------------------------------------------
    private double integralSum    = 0.0;
    private double lastError      = 0.0;   // operates on smoothedError

    // ------------------------------------------------------------------
    // Timers
    // ------------------------------------------------------------------
    private final ElapsedTime pidTimer       = new ElapsedTime();
    private final ElapsedTime slewTimer      = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();

    // ------------------------------------------------------------------
    // Constructor
    // ------------------------------------------------------------------
    protected AprilTagCenteringBase(int tagId, String tagLabel) {
        this.targetTagId    = tagId;
        this.targetTagLabel = tagLabel;
    }

    // ======================================================================
    //  LinearOpMode entry point
    // ======================================================================
    @Override
    public final void runOpMode() {

        initHardware();
        initVision();
        zeroTurretEncoder();

        telemetry.addLine("Turret ready — waiting for Start");
        telemetry.addData("Tracking tag", targetTagId + " / " + targetTagLabel);
        telemetry.update();

        waitForStart();
        pidTimer.reset();
        slewTimer.reset();
        telemetryTimer.reset();

        // ── Main loop ────────────────────────────────────────────────
        while (opModeIsActive()) {

            AprilTagDetection detection = findTargetTag();

            if (detection != null) {
                // 1. Raw pixel error
                double rawError   = computeHorizontalError(detection);

                // 2. Smooth it — removes frame-to-frame detection noise
                smoothedError     = applySmoothing(rawError);

                // 3. PID on the smoothed signal
                double pidOutput  = computePid(smoothedError);

                // 4. Soft limits — scale power near travel ends
                double safePower  = applySoftLimits(pidOutput);

                // 5. Slew rate limit — prevents abrupt power steps
                currentPower      = applySlewLimit(safePower);

                // 6. Floor — ensure motor overcomes friction when moving
                currentPower      = applyMinPower(currentPower, smoothedError);

                turretMotor.setPower(currentPower);

            } else {
                // Tag lost → ramp motor gently back to zero, then clear state
                currentPower = applySlewLimit(0.0);
                turretMotor.setPower(currentPower);

                if (currentPower == 0.0) {
                    resetAllState();
                }
            }

            updateTelemetry(detection);
            idle();
        }

        // ── Cleanup ──────────────────────────────────────────────────
        turretMotor.setPower(0.0);
        visionPortal.close();
    }

    // ======================================================================
    //  Initialisation helpers
    // ======================================================================

    private void initHardware() {
        turretMotor = hardwareMap.get(DcMotorEx.class, TurretMotorConfig.MOTOR_NAME);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initVision() {
        // TAG_36h11 is the default family — no setTagFamily() call needed.
        // setTagLibrary(null) is invalid; omitting it uses the built-in library.
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, TurretMotorConfig.CAMERA_NAME))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .build();

        while (!isStopRequested()
                && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("Waiting for camera…");
            telemetry.update();
            sleep(20);
        }
    }

    private void zeroTurretEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // ======================================================================
    //  Vision helpers
    // ======================================================================

    private AprilTagDetection findTargetTag() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection d : detections) {
            if (d.id == targetTagId) return d;
        }
        return null;
    }

    /**
     * Raw horizontal pixel offset from image centre.
     * Positive → tag right of centre → turret rotates right (+power).
     * Negative → tag left of centre  → turret rotates left  (−power).
     * detection.center is an org.opencv.core.Point; .x is a plain double.
     */
    private double computeHorizontalError(AprilTagDetection detection) {
        return detection.center.x - TurretMotorConfig.IMAGE_CENTER_X;
    }

    // ======================================================================
    //  Signal pipeline
    // ======================================================================

    /**
     * Exponential moving average filter.
     *
     *   smoothed = α × raw + (1 − α) × previousSmoothed
     *
     * Removes single-frame AprilTag detection noise before the signal
     * reaches PID. Critical for the derivative term, which amplifies noise.
     */
    private double applySmoothing(double rawError) {
        double alpha = TurretMotorConfig.ERROR_SMOOTHING_ALPHA;
        smoothedError = alpha * rawError + (1.0 - alpha) * smoothedError;
        return smoothedError;
    }

    /**
     * Slew rate limiter — the core fix for abrupt motor movement.
     *
     * Caps how fast the motor power command can change per second.
     * No matter what the PID requests, the actual command sent to the motor
     * can only move by (SLEW_RATE_PER_SECOND × dt) per loop iteration.
     *
     * This means:
     *  • Power never jumps from 0 → 0.60 in one tick (was causing the jerk).
     *  • Direction reversals are smooth ramps through zero, not instant flips.
     *  • The feel is tunable independently of PID gains via SLEW_RATE_PER_SECOND.
     *
     * @param target  the power level the PID wants to achieve
     * @return        the slew-limited power to actually send to the motor
     */
    private double applySlewLimit(double target) {
        double dt      = slewTimer.seconds();
        slewTimer.reset();
        if (dt <= 0.0) dt = 0.02;

        double maxStep = TurretMotorConfig.SLEW_RATE_PER_SECOND * dt;
        double delta   = target - currentPower;

        // Clamp the step this tick to ±maxStep
        delta = clamp(delta, -maxStep, maxStep);
        return currentPower + delta;
    }

    // ======================================================================
    //  PID controller
    // ======================================================================

    /**
     * Time-scaled PID operating on the smoothed pixel error.
     *
     * @param error  smoothed horizontal pixel offset
     * @return       desired motor power in [−MAX_MOTOR_POWER, +MAX_MOTOR_POWER]
     */
    private double computePid(double error) {

        // Dead-band: suppress motor output entirely when centred
        if (Math.abs(error) <= TurretMotorConfig.CENTERING_DEADBAND_PX) {
            resetPidState();
            return 0.0;
        }

        double dt = pidTimer.seconds();
        pidTimer.reset();
        if (dt <= 0.0) dt = 0.02;

        // P term
        double pTerm = TurretMotorConfig.KP * error;

        // I term with anti-windup clamp
        integralSum += error * dt;
        integralSum  = clamp(integralSum,
                             -TurretMotorConfig.INTEGRAL_CLAMP,
                              TurretMotorConfig.INTEGRAL_CLAMP);
        double iTerm = TurretMotorConfig.KI * integralSum;

        // D term on smoothed error (noise already filtered by EMA)
        double dTerm = TurretMotorConfig.KD * ((error - lastError) / dt);
        lastError = error;

        double output = pTerm + iTerm + dTerm;
        return clamp(output,
                     -TurretMotorConfig.MAX_MOTOR_POWER,
                      TurretMotorConfig.MAX_MOTOR_POWER);
    }

    // ======================================================================
    //  Motor safety helpers
    // ======================================================================

    /**
     * Linearly scales power to zero in the last SOFT_STOP_BUFFER_TICKS
     * before each hard encoder limit.
     */
    private double applySoftLimits(double requestedPower) {
        double pos    = turretMotor.getCurrentPosition();
        double buffer = TurretMotorConfig.SOFT_STOP_BUFFER_TICKS;
        double limit  = TurretMotorConfig.MAX_TICKS;

        if (requestedPower > 0.0) {
            double remaining = limit - pos;
            if (remaining <= 0.0)   return 0.0;
            if (remaining < buffer) requestedPower *= (remaining / buffer);
        }

        if (requestedPower < 0.0) {
            double remaining = pos - (-limit);
            if (remaining <= 0.0)   return 0.0;
            if (remaining < buffer) requestedPower *= (remaining / buffer);
        }

        return requestedPower;
    }

    /**
     * Applies a minimum power floor only when the motor is already moving
     * (currentPower != 0) to prevent stalling under friction.
     *
     * Unlike the previous version this does NOT create a step from 0 → MIN.
     * The slew limiter already handles the ramp-up from zero; this only
     * prevents the motor from dropping below the friction threshold once moving.
     */
    private double applyMinPower(double power, double error) {
        if (power == 0.0) return 0.0;
        if (Math.abs(error) <= TurretMotorConfig.CENTERING_DEADBAND_PX) return 0.0;

        // Only apply floor if the motor is already in motion (avoids the hard step)
        if (Math.abs(currentPower) > 0.0
                && Math.abs(power) < TurretMotorConfig.MIN_MOTOR_POWER) {
            power = Math.copySign(TurretMotorConfig.MIN_MOTOR_POWER, power);
        }
        return power;
    }

    // ======================================================================
    //  State reset
    // ======================================================================

    private void resetPidState() {
        integralSum = 0.0;
        lastError   = 0.0;
        pidTimer.reset();
    }

    private void resetAllState() {
        resetPidState();
        smoothedError = 0.0;
        currentPower  = 0.0;
        slewTimer.reset();
    }

    // ======================================================================
    //  Telemetry
    // ======================================================================

    private void updateTelemetry(AprilTagDetection detection) {
        if (telemetryTimer.milliseconds() < TurretMotorConfig.TELEMETRY_UPDATE_MS) return;
        telemetryTimer.reset();

        telemetry.addLine("══ AprilTag Centering ══════════════════");
        telemetry.addData("Target", targetTagId + " / " + targetTagLabel);

        if (detection != null) {
            double rawError = computeHorizontalError(detection);
            telemetry.addData("Tag visible",    "YES");
            telemetry.addData("Tag centre X",   "%.1f px",  detection.center.x);
            telemetry.addData("Raw error",      "%.1f px",  rawError);
            telemetry.addData("Smoothed error", "%.1f px",  smoothedError);
            telemetry.addData("Range",          "%.2f in",  detection.ftcPose.range);
            telemetry.addData("Bearing",        "%.1f deg", detection.ftcPose.bearing);
        } else {
            telemetry.addData("Tag visible", "NO — holding");
        }

        telemetry.addLine("────────────────────────────────────────");
        telemetry.addData("Motor power",   "%.3f", currentPower);
        telemetry.addData("Encoder ticks",          turretMotor.getCurrentPosition());
        telemetry.addData("Limit ± ticks", "%.1f", TurretMotorConfig.MAX_TICKS);
        telemetry.addData("Integral sum",  "%.1f", integralSum);
        telemetry.update();
    }

    // ======================================================================
    //  Utility
    // ======================================================================

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
