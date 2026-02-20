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
 *  • Runs a PID loop to keep a specific AprilTag horizontally centred
 *    in the camera frame by driving a turret motor.
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
    // PID state
    // ------------------------------------------------------------------
    private double integralSum  = 0.0;
    private double lastError    = 0.0;
    private final ElapsedTime pidTimer       = new ElapsedTime();
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
        telemetryTimer.reset();

        // ── Main loop ────────────────────────────────────────────────
        while (opModeIsActive()) {

            AprilTagDetection detection = findTargetTag();

            if (detection != null) {
                double error = computeHorizontalError(detection);
                double power = computePid(error);
                power = applySoftLimits(power);
                power = applyMinPower(power, error);
                turretMotor.setPower(power);
            } else {
                // Tag lost → hold position, reset PID to avoid windup
                turretMotor.setPower(0.0);
                resetPidState();
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
        // Power is controlled entirely by our PID — no built-in velocity controller
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initVision() {
        /*
         * TAG_36h11 is the AprilTagProcessor default family, so no
         * setTagFamily() call is required.  The SDK's built-in tag library
         * is used automatically; passing null to setTagLibrary() is invalid.
         *
         * setOutputUnits() controls the units of detection.ftcPose fields
         * (range, bearing, yaw, x, y, z).  INCH keeps everything consistent
         * with FTC field-measurement conventions.
         */
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        /*
         * android.util.Size is the type accepted by setCameraResolution().
         * It is now imported explicitly at the top of the file.
         */
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, TurretMotorConfig.CAMERA_NAME))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .build();

        // Block until the camera is streaming before leaving init
        while (!isStopRequested()
                && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("Waiting for camera…");
            telemetry.update();
            sleep(20);
        }
    }

    /** Reset encoder at mechanical centre so ±MAX_TICKS is symmetric. */
    private void zeroTurretEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // ======================================================================
    //  Vision helpers
    // ======================================================================

    /**
     * Scans the latest AprilTag detections and returns the one matching
     * {@link #targetTagId}, or {@code null} if not currently visible.
     */
    private AprilTagDetection findTargetTag() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection d : detections) {
            if (d.id == targetTagId) return d;
        }
        return null;
    }

    /**
     * Horizontal pixel offset of the tag centre from the image centre.
     *
     * {@code detection.center} is an {@code org.opencv.core.Point} whose
     * {@code .x} and {@code .y} fields are plain {@code double} — no cast needed.
     *
     * Positive → tag is right of centre → turret should rotate right (+power).
     * Negative → tag is left of centre  → turret should rotate left  (−power).
     */
    private double computeHorizontalError(AprilTagDetection detection) {
        return detection.center.x - TurretMotorConfig.IMAGE_CENTER_X;
    }

    // ======================================================================
    //  PID controller
    // ======================================================================

    /**
     * Time-scaled PID on horizontal pixel error.
     *
     * Using elapsed time (dt) makes Ki and Kd physically meaningful and
     * consistent even when the main-loop period varies slightly.
     *
     * Derivative is computed on the error signal directly; since the
     * setpoint is always zero (centre) this is equivalent to the
     * "derivative on measurement" technique and avoids derivative kick
     * when the tag reappears after being lost.
     *
     * @param error  horizontal pixel offset (positive = tag right of centre)
     * @return       motor power in [−MAX_MOTOR_POWER, +MAX_MOTOR_POWER]
     */
    private double computePid(double error) {

        // Dead-band: suppress motor noise when already centred
        if (Math.abs(error) <= TurretMotorConfig.CENTERING_DEADBAND_PX) {
            resetPidState();
            return 0.0;
        }

        double dt = pidTimer.seconds();
        pidTimer.reset();
        if (dt <= 0.0) dt = 0.02;   // guard: avoid divide-by-zero on first call

        // P term
        double pTerm = TurretMotorConfig.KP * error;

        // I term — clamp accumulator to prevent windup
        integralSum += error * dt;
        integralSum  = clamp(integralSum,
                             -TurretMotorConfig.INTEGRAL_CLAMP,
                              TurretMotorConfig.INTEGRAL_CLAMP);
        double iTerm = TurretMotorConfig.KI * integralSum;

        // D term
        double dTerm = TurretMotorConfig.KD * ((error - lastError) / dt);
        lastError = error;

        double output = pTerm + iTerm + dTerm;
        return clamp(output,
                     -TurretMotorConfig.MAX_MOTOR_POWER,
                      TurretMotorConfig.MAX_MOTOR_POWER);
    }

    private void resetPidState() {
        integralSum = 0.0;
        lastError   = 0.0;
        pidTimer.reset();
    }

    // ======================================================================
    //  Motor safety helpers
    // ======================================================================

    /**
     * Linearly ramps motor power to zero as the encoder approaches a hard
     * limit, providing smooth deceleration rather than an abrupt cut-off.
     *
     * Full power is allowed until within {@link TurretMotorConfig#SOFT_STOP_BUFFER_TICKS}
     * ticks of the limit; power is then scaled proportionally 1.0 → 0.0.
     */
    private double applySoftLimits(double requestedPower) {

        double pos    = turretMotor.getCurrentPosition();
        double buffer = TurretMotorConfig.SOFT_STOP_BUFFER_TICKS;
        double limit  = TurretMotorConfig.MAX_TICKS;

        // Right (positive) limit
        if (requestedPower > 0.0) {
            double remaining = limit - pos;
            if (remaining <= 0.0)   return 0.0;
            if (remaining < buffer) requestedPower *= (remaining / buffer);
        }

        // Left (negative) limit
        if (requestedPower < 0.0) {
            double remaining = pos - (-limit);
            if (remaining <= 0.0)   return 0.0;
            if (remaining < buffer) requestedPower *= (remaining / buffer);
        }

        return requestedPower;
    }

    /**
     * Guarantees the motor receives at least {@link TurretMotorConfig#MIN_MOTOR_POWER}
     * whenever there is meaningful error outside the dead-band.
     * Prevents stalling under friction at very small PID outputs.
     * Has no effect when power is already 0.0 (dead-band or at a limit).
     */
    private double applyMinPower(double power, double error) {
        if (power == 0.0) return 0.0;
        if (Math.abs(error) <= TurretMotorConfig.CENTERING_DEADBAND_PX) return 0.0;

        if (Math.abs(power) < TurretMotorConfig.MIN_MOTOR_POWER) {
            power = Math.copySign(TurretMotorConfig.MIN_MOTOR_POWER, power);
        }
        return power;
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
            double error = computeHorizontalError(detection);
            telemetry.addData("Tag visible",  "YES");
            telemetry.addData("Tag centre X", "%.1f px",  detection.center.x);
            telemetry.addData("Error",        "%.1f px",  error);
            telemetry.addData("Range",        "%.2f in",  detection.ftcPose.range);
            telemetry.addData("Bearing",      "%.1f deg", detection.ftcPose.bearing);
        } else {
            telemetry.addData("Tag visible", "NO — holding");
        }

        telemetry.addLine("────────────────────────────────────────");
        telemetry.addData("Motor power",   "%.3f", turretMotor.getPower());
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
