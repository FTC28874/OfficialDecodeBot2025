package org.firstinspires.ftc.teamcode.turret;

import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
 * Abstract helper base class (NOT a LinearOpMode) that encapsulates all
 * shared AprilTag tracking logic:
 *
 *   • VisionPortal / AprilTagProcessor initialisation
 *   • Exponential moving-average noise filter on pixel error
 *   • Time-scaled PID controller
 *   • Slew-rate limiter on motor output
 *   • Encoder soft-limit safety
 *   • Driver Station telemetry
 *
 * Concrete subclasses (e.g. RedGoalCentering) supply the tag ID and label
 * via the constructor. The owning TeleOp (e.g. RedTeleOp) calls:
 *
 *   init(hardwareMap, telemetry)  — once during OpMode init
 *   update(trackingEnabled)       — every loop tick
 *   stop()                        — on OpMode end
 *
 * All tuning lives in {@link TurretMotorConfig}.
 */
public abstract class AprilTagCenteringBase {

    // ------------------------------------------------------------------
    // Subclass identity (set via constructor)
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
    // Telemetry (injected from parent OpMode)
    // ------------------------------------------------------------------
    private Telemetry telemetry;

    // ------------------------------------------------------------------
    // Signal pipeline state
    // ------------------------------------------------------------------
    /** Exponential moving average of raw pixel error. */
    private double smoothedError = 0.0;

    /** Last power value sent to the motor — used by slew limiter. */
    private double currentPower  = 0.0;

    /** Tracks whether the VisionPortal is currently streaming. */
    private boolean streamingActive = false;

    // ------------------------------------------------------------------
    // PID state
    // ------------------------------------------------------------------
    private double integralSum = 0.0;
    private double lastError   = 0.0;

    // ------------------------------------------------------------------
    // Cached sensor output
    // ------------------------------------------------------------------
    /** Last measured distance to tag in inches. −1 = tag not visible. */
    private double distanceInches = -1.0;

    // ------------------------------------------------------------------
    // Timers
    // ------------------------------------------------------------------
    private final ElapsedTime pidTimer       = new ElapsedTime();
    private final ElapsedTime slewTimer      = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();

    // ------------------------------------------------------------------
    // Guard
    // ------------------------------------------------------------------
    private boolean initialised = false;

    // ======================================================================
    //  Constructor
    // ======================================================================

    /**
     * @param tagId    AprilTag ID this instance will track
     * @param tagLabel Human-readable name shown on the Driver Station
     */
    protected AprilTagCenteringBase(int tagId, String tagLabel) {
        this.targetTagId    = tagId;
        this.targetTagLabel = tagLabel;
    }

    // ======================================================================
    //  Public API  (called by the owning TeleOp)
    // ======================================================================

    /**
     * Initialises the turret motor, VisionPortal, and resets all state.
     * Must be called once from the parent OpMode's init block.
     *
     * @param hardwareMap from the parent LinearOpMode
     * @param telemetry   from the parent LinearOpMode
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        initHardware(hardwareMap);
        initVision(hardwareMap);
        zeroTurretEncoder();
        pidTimer.reset();
        slewTimer.reset();
        telemetryTimer.reset();
        initialised = true;
    }

    /**
     * Main update — call every loop tick from the parent OpMode.
     *
     * @param trackingEnabled true  → PID runs, turret tracks the tag
     *                        false → turret ramps smoothly back to centre
     */
    public void update(boolean trackingEnabled) {
        if (!initialised) return;

        // ── Gate the vision processor to the button ───────────────────────
        // When tracking is not requested, stop the camera stream entirely
        // to free CPU on the Control Hub.  Resume instantly when needed.
        if (trackingEnabled && !streamingActive) {
            visionPortal.resumeStreaming();
            streamingActive = true;
        } else if (!trackingEnabled && streamingActive) {
            visionPortal.stopStreaming();
            streamingActive = false;
        }

        // Only poll detections when the stream is actually running
        AprilTagDetection detection = streamingActive ? findTargetTag() : null;

        // Always refresh distance, regardless of tracking mode
        distanceInches = (detection != null) ? detection.ftcPose.range : -1.0;

        if (trackingEnabled && detection != null) {
            // Full signal pipeline: smooth → PID → soft limits → slew → floor
            double rawError  = computeHorizontalError(detection);
            smoothedError    = applySmoothing(rawError);
            double pidOutput = computePid(smoothedError);
            double safePower = applySoftLimits(pidOutput);
            currentPower     = applySlewLimit(safePower);
            currentPower     = applyMinPower(currentPower, smoothedError);
        } else {
            // Tracking off or tag lost → ramp gently back to zero
            currentPower = applySlewLimit(0.0);
            if (currentPower == 0.0) {
                resetAllState();
            }
        }

        turretMotor.setPower(currentPower);
        updateTelemetry(detection, trackingEnabled);
    }

    /**
     * Stops the motor and closes the camera stream.
     * Call from the parent OpMode's stop / end-of-runOpMode block.
     */
    public void stop() {
        if (turretMotor  != null) turretMotor.setPower(0.0);
        if (visionPortal != null) visionPortal.close();
    }

    // ======================================================================
    //  Public getters  (for use by the parent TeleOp and other subsystems)
    // ======================================================================

    /** Distance to tag in inches. Returns −1.0 if tag is not visible. */
    public double getDistanceInches()  { return distanceInches; }

    /** True if the tag was seen on the most recent update() call. */
    public boolean isTagVisible()      { return distanceInches >= 0.0; }

    /** Smoothed horizontal pixel error (0 = perfectly centred). */
    public double getSmoothedError()   { return smoothedError; }

    /** True when the smoothed error is within the dead-band. */
    public boolean isCentred() {
        return Math.abs(smoothedError) <= TurretMotorConfig.CENTERING_DEADBAND_PX;
    }

    /** Tag ID this instance tracks — useful for telemetry in subclasses. */
    protected int getTargetTagId()     { return targetTagId; }

    /** Tag label this instance tracks — useful for telemetry in subclasses. */
    protected String getTargetTagLabel() { return targetTagLabel; }

    // ======================================================================
    //  Initialisation helpers
    // ======================================================================

    private void initHardware(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, TurretMotorConfig.MOTOR_NAME);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        // PID is software-side; no built-in velocity controller needed
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initVision(HardwareMap hardwareMap) {
        // TAG_36h11 is the SDK default family — no setTagFamily() call needed.
        // Omitting setTagLibrary() uses the built-in library automatically.
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

        // Wait for the camera to be ready, then immediately suspend streaming.
        // The stream will be resumed on demand inside update() when the
        // tracking button is pressed — keeping CPU free until then.
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine(targetTagLabel + " centering: waiting for camera…");
            telemetry.update();
            try { Thread.sleep(20); } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        visionPortal.stopStreaming();
        streamingActive = false;
    }

    /** Zeros encoder at the mechanical centre so ±MAX_TICKS is symmetric. */
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
     * Exponential moving-average filter.
     * smoothed = α × raw + (1 − α) × previousSmoothed
     * Removes frame-to-frame detection noise before the PID sees it.
     */
    private double applySmoothing(double rawError) {
        double alpha = TurretMotorConfig.ERROR_SMOOTHING_ALPHA;
        smoothedError = alpha * rawError + (1.0 - alpha) * smoothedError;
        return smoothedError;
    }

    /**
     * Slew rate limiter — caps how fast motor power can change per second.
     * Prevents abrupt power steps regardless of what the PID requests.
     * Direction reversals ramp smoothly through zero instead of flipping.
     */
    private double applySlewLimit(double target) {
        double dt      = slewTimer.seconds();
        slewTimer.reset();
        if (dt <= 0.0) dt = 0.02;

        double maxStep = TurretMotorConfig.SLEW_RATE_PER_SECOND * dt;
        double delta   = clamp(target - currentPower, -maxStep, maxStep);
        return currentPower + delta;
    }

    // ======================================================================
    //  PID controller
    // ======================================================================

    /**
     * Time-scaled PID on smoothed pixel error.
     * Dead-band, integral anti-windup, and per-second Ki/Kd scaling included.
     */
    private double computePid(double error) {
        if (Math.abs(error) <= TurretMotorConfig.CENTERING_DEADBAND_PX) {
            resetPidState();
            return 0.0;
        }

        double dt = pidTimer.seconds();
        pidTimer.reset();
        if (dt <= 0.0) dt = 0.02;

        double pTerm = TurretMotorConfig.KP * error;

        integralSum += error * dt;
        integralSum  = clamp(integralSum,
                             -TurretMotorConfig.INTEGRAL_CLAMP,
                              TurretMotorConfig.INTEGRAL_CLAMP);
        double iTerm = TurretMotorConfig.KI * integralSum;

        double dTerm = TurretMotorConfig.KD * ((error - lastError) / dt);
        lastError = error;

        return clamp(pTerm + iTerm + dTerm,
                     -TurretMotorConfig.MAX_MOTOR_POWER,
                      TurretMotorConfig.MAX_MOTOR_POWER);
    }

    // ======================================================================
    //  Motor safety helpers
    // ======================================================================

    /** Linearly scales power to zero in the last SOFT_STOP_BUFFER_TICKS. */
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
     * Prevents motor stall under friction at low PID outputs.
     * Only applied once already in motion — the slew limiter handles ramp-up
     * from zero, so this never creates a hard 0 → MIN step.
     */
    private double applyMinPower(double power, double error) {
        if (power == 0.0) return 0.0;
        if (Math.abs(error) <= TurretMotorConfig.CENTERING_DEADBAND_PX) return 0.0;
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

    private void updateTelemetry(AprilTagDetection detection, boolean trackingEnabled) {
        if (telemetryTimer.milliseconds() < TurretMotorConfig.TELEMETRY_UPDATE_MS) return;
        telemetryTimer.reset();

        telemetry.addLine("── Turret: " + targetTagLabel + " (ID " + targetTagId + ") ─────");
        telemetry.addData("Tracking",       trackingEnabled ? "ACTIVE" : "off");
        telemetry.addData("Centred",        isCentred()     ? "YES"    : "no");

        if (detection != null) {
            telemetry.addData("Tag visible",    "YES");
            telemetry.addData("Smoothed error", "%.1f px",  smoothedError);
            telemetry.addData("Distance",       "%.2f in",  distanceInches);
            telemetry.addData("Bearing",        "%.1f deg", detection.ftcPose.bearing);
        } else {
            telemetry.addData("Tag visible", "NO");
        }

        telemetry.addData("Motor power",   "%.3f", currentPower);
        telemetry.addData("Encoder ticks",          turretMotor.getCurrentPosition());
    }

    // ======================================================================
    //  Utility
    // ======================================================================

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
