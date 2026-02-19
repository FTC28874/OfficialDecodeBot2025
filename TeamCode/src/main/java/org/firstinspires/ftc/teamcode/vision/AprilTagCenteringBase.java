package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * AprilTagCenteringBase — DECODE Season (2025-2026)
 *
 * Abstract base class that implements all shared camera pan logic:
 *   - AprilTag detection via VisionPortal
 *   - PID centering controller
 *   - Encoder soft limits
 *   - Smooth motion ramping
 *   - Tag-lost hold behaviour
 *   - Telemetry
 *
 * Subclasses must implement {@link #getTargetTagId()} and
 * {@link #getTargetTagName()} to specify which AprilTag to track.
 *
 * All tuning constants live in {@link PanMotorConfig}.
 *
 * @see RedGoalCenteringTeleOp
 * @see BlueGoalCenteringTeleOp
 */
public abstract class AprilTagCenteringBase extends LinearOpMode {

    // ═══════════════════════════════════════════════════════════════════════
    //  ABSTRACT CONTRACT — implemented by each goal-specific subclass
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Returns the AprilTag ID this OpMode should track.
     * e.g. 24 for RED Goal, 20 for BLUE Goal.
     */
    protected abstract int getTargetTagId();

    /**
     * Returns a human-readable name for the tag used in telemetry and the
     * AprilTag library entry.
     * e.g. "RED Goal", "BLUE Goal"
     */
    protected abstract String getTargetTagName();

    // ═══════════════════════════════════════════════════════════════════════
    //  HARDWARE
    // ═══════════════════════════════════════════════════════════════════════

    private DcMotorEx      panMotor;
    private VisionPortal   visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    // ═══════════════════════════════════════════════════════════════════════
    //  PID STATE
    // ═══════════════════════════════════════════════════════════════════════

    private double integralSum  = 0.0;
    private double lastError    = 0.0;
    private double currentPower = 0.0;
    private final ElapsedTime pidTimer = new ElapsedTime();

    // ═══════════════════════════════════════════════════════════════════════
    //  TAG-LOST STATE
    // ═══════════════════════════════════════════════════════════════════════

    private long    lastSeenTimestamp = 0;
    private boolean motorHolding      = false;

    // ═══════════════════════════════════════════════════════════════════════
    //  ENTRY POINT
    // ═══════════════════════════════════════════════════════════════════════

    @Override
    public final void runOpMode() {

        initHardware();
        initVision();

        telemetry.addData("OpMode",  "%s Centering ready", getTargetTagName());
        telemetry.addData("Tag ID",  getTargetTagId());
        telemetry.addData("Limits",  "%.0f° / %.0f°  →  %d / %d ticks",
                PanMotorConfig.LIMIT_DEGREES_MIN, PanMotorConfig.LIMIT_DEGREES_MAX,
                PanMotorConfig.ENCODER_LIMIT_MIN, PanMotorConfig.ENCODER_LIMIT_MAX);
        telemetry.addLine("Waiting for START…");
        telemetry.update();

        waitForStart();
        pidTimer.reset();
        lastSeenTimestamp = System.currentTimeMillis();

        // ── Main loop ────────────────────────────────────────────────────
        while (opModeIsActive()) {

            AprilTagDetection target = findTargetTag();

            if (target != null) {
                onTagDetected(target);
            } else {
                onTagLost();
            }

            telemetry.update();
        }

        // ── Cleanup ──────────────────────────────────────────────────────
        safeStopMotor();
        visionPortal.close();
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  DETECTION HANDLERS
    // ═══════════════════════════════════════════════════════════════════════

    private void onTagDetected(AprilTagDetection target) {
        lastSeenTimestamp = System.currentTimeMillis();
        motorHolding      = false;

        // Restore velocity control if we were in hold mode
        if (panMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            panMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        double error = computeXError(target);
        driveMotorPID(error);
        sendTrackingTelemetry(target, error);
    }

    private void onTagLost() {
        long elapsedMs = System.currentTimeMillis() - lastSeenTimestamp;

        if (!motorHolding && elapsedMs > PanMotorConfig.HOLD_DELAY_MS) {
            holdPosition();
            motorHolding = true;
        }

        // Bleed integral so stale accumulation doesn't cause drift on reacquire
        integralSum *= PanMotorConfig.INTEGRAL_BLEED;

        sendLostTelemetry(elapsedMs);
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  HARDWARE INIT
    // ═══════════════════════════════════════════════════════════════════════

    private void initHardware() {
        panMotor = hardwareMap.get(DcMotorEx.class, PanMotorConfig.MOTOR_NAME);

        // Zero encoder on init — limits are relative to starting position
        panMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        panMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Brake at zero power to hold position
        panMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Change to REVERSE if the camera pans in the wrong direction
        panMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  VISION INIT
    // ═══════════════════════════════════════════════════════════════════════

    private void initVision() {

        // Register tag with correct physical dimensions for accurate pose data
        AprilTagLibrary tagLibrary = new AprilTagLibrary.Builder()
                .addTag(new AprilTagMetadata(
                        getTargetTagId(),
                        getTargetTagName(),
                        PanMotorConfig.TAG_SIZE_OUTER_IN,
                        null,               // fieldPosition  (not needed for centering)
                        DistanceUnit.INCH,  // matches TAG_SIZE_OUTER_IN units
                        null))              // fieldOrientation (not needed for centering)
                .build();

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setDrawAxes(true)
                .setNumThreads(2)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, PanMotorConfig.CAMERA_NAME))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new android.util.Size(
                        PanMotorConfig.CAMERA_WIDTH,
                        PanMotorConfig.CAMERA_HEIGHT))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .build();
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  APRILTAG DETECTION
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Scans current detections and returns the one matching our tag ID,
     * or null if not currently visible.
     */
    private AprilTagDetection findTargetTag() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection d : detections) {
            if (d.id == getTargetTagId()) return d;
        }
        return null;
    }

    /**
     * Computes horizontal pixel error between the tag center and the frame center.
     * Positive error  = tag is to the RIGHT of center → motor should pan right.
     * Negative error  = tag is to the LEFT  of center → motor should pan left.
     */
    private double computeXError(AprilTagDetection detection) {
        double frameCenterX = PanMotorConfig.CAMERA_WIDTH / 2.0;
        return detection.center.x - frameCenterX;
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  PID MOTOR CONTROL
    // ═══════════════════════════════════════════════════════════════════════

    private void driveMotorPID(double error) {

        // Deadband — coast to zero when the tag is close enough to centered
        if (Math.abs(error) < PanMotorConfig.DEADBAND_PX) {
            applySmoothedPower(0.0);
            integralSum = 0.0;
            lastError   = 0.0;
            return;
        }

        double dt = pidTimer.seconds();
        pidTimer.reset();
        // Guard against stale or invalid dt values
        if (dt <= 0 || dt > 0.5) dt = 0.02;

        // ── Proportional ────────────────────────────────────────────────
        double pTerm = PanMotorConfig.kP * error;

        // ── Integral (anti-windup clamp) ─────────────────────────────────
        integralSum = clamp(
                integralSum + error * dt,
                -PanMotorConfig.INTEGRAL_LIMIT,
                 PanMotorConfig.INTEGRAL_LIMIT);
        double iTerm = PanMotorConfig.kI * integralSum;

        // ── Derivative ──────────────────────────────────────────────────
        double dTerm = PanMotorConfig.kD * (error - lastError) / dt;
        lastError = error;

        // ── Combine, clamp, scale ────────────────────────────────────────
        double rawPower = clamp(pTerm + iTerm + dTerm, -1.0, 1.0)
                        * PanMotorConfig.MAX_VELOCITY_FRACTION;

        // ── Encoder soft limits ──────────────────────────────────────────
        int currentTick = panMotor.getCurrentPosition();
        if (currentTick >= PanMotorConfig.ENCODER_LIMIT_MAX && rawPower > 0) rawPower = 0;
        if (currentTick <= PanMotorConfig.ENCODER_LIMIT_MIN && rawPower < 0) rawPower = 0;

        applySmoothedPower(rawPower);
    }

    /**
     * Ramps the applied motor power toward {@code targetPower} at most
     * RAMP_RATE per cycle, giving smooth acceleration and deceleration.
     */
    private void applySmoothedPower(double targetPower) {
        double delta = clamp(
                targetPower - currentPower,
                -PanMotorConfig.RAMP_RATE,
                 PanMotorConfig.RAMP_RATE);
        currentPower += delta;
        panMotor.setPower(currentPower);
    }

    /**
     * Switches the motor to RUN_TO_POSITION at the current tick to actively
     * resist disturbances while the tag is not visible.
     */
    private void holdPosition() {
        int holdTick = panMotor.getCurrentPosition();
        panMotor.setTargetPosition(holdTick);
        panMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        panMotor.setPower(PanMotorConfig.HOLD_POWER);
        // Reset ramp and PID state so we start fresh on reacquisition
        currentPower = 0.0;
        integralSum  = 0.0;
        lastError    = 0.0;
    }

    private void safeStopMotor() {
        panMotor.setPower(0);
        panMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  TELEMETRY
    // ═══════════════════════════════════════════════════════════════════════

    private void sendTrackingTelemetry(AprilTagDetection d, double error) {
        telemetry.addLine("── " + getTargetTagName() + " Centering ──────────────");
        telemetry.addData("Status",        "TRACKING Tag #%d", getTargetTagId());
        telemetry.addData("Tag center X",  "%.1f px  (frame center = %.0f)",
                d.center.x, PanMotorConfig.CAMERA_WIDTH / 2.0);
        telemetry.addData("X error",       "%.1f px  (deadband ±%.0f)",
                error, PanMotorConfig.DEADBAND_PX);
        telemetry.addData("Motor power",   "%.3f", currentPower);
        telemetry.addData("Encoder pos",   "%d  (limits: %d / %d)",
                panMotor.getCurrentPosition(),
                PanMotorConfig.ENCODER_LIMIT_MIN,
                PanMotorConfig.ENCODER_LIMIT_MAX);
        telemetry.addLine("── PID ─────────────────────────────────");
        telemetry.addData("kP / kI / kD",  "%.4f / %.4f / %.4f",
                PanMotorConfig.kP, PanMotorConfig.kI, PanMotorConfig.kD);
        telemetry.addData("Integral sum",  "%.2f  (cap ±%.0f)",
                integralSum, PanMotorConfig.INTEGRAL_LIMIT);
        telemetry.addLine("── Tag Pose (camera frame) ──────────────");
        telemetry.addData("Range",         "%.2f in", d.ftcPose.range);
        telemetry.addData("Bearing",       "%.2f°",   d.ftcPose.bearing);
        telemetry.addData("Elevation",     "%.2f°",   d.ftcPose.elevation);
    }

    private void sendLostTelemetry(long elapsedMs) {
        telemetry.addLine("── " + getTargetTagName() + " Centering ──────────────");
        telemetry.addData("Status",      motorHolding
                ? "HOLDING position (tag lost)"
                : "Searching… (tag lost)");
        telemetry.addData("Lost for",    "%d ms  (hold after %d ms)",
                elapsedMs, PanMotorConfig.HOLD_DELAY_MS);
        telemetry.addData("Encoder pos", "%d", panMotor.getCurrentPosition());
        telemetry.addData("Motor power", "%.3f", currentPower);
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  UTILITIES
    // ═══════════════════════════════════════════════════════════════════════

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
