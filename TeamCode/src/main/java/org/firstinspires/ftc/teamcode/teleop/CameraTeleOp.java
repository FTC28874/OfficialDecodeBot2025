package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.turret.AprilTagCenteringBase;

/**
 * CameraTeleOp
 *
 * Abstract TeleOp base class that owns all turret / vision logic shared
 * between RedTeleOp and BlueTeleOp (and any future alliance variants).
 *
 * Responsibilities
 * ────────────────
 *   • Initialises and stops the AprilTagCenteringBase subsystem.
 *   • Calls update() every loop tick, gating it to gamepad2.right_bumper.
 *   • Caches distanceToGoalInches and turretCentred each tick so subclasses
 *     can use them without re-querying the subsystem.
 *   • Provides turret telemetry lines common to all subclasses.
 *   • Exposes the main loop via runOpMode() using the Template Method pattern:
 *       - onInit()       → subclass init hook  (optional override)
 *       - onStart()      → subclass start hook (optional override)
 *       - onLoop()       → subclass loop hook  (required override — drive,
 *                          shooter, intake, etc. go here)
 *       - onStop()       → subclass stop hook  (optional override)
 *
 * Subclasses must implement
 * ─────────────────────────
 *   protected abstract AprilTagCenteringBase createCenteringSubsystem();
 *       Return the correct centering instance (RedGoalCentering or
 *       BlueGoalCentering). Called once during init.
 *
 * Subclasses may override
 * ────────────────────────
 *   protected void onInit()   — extra hardware init (drivetrain, shooter…)
 *   protected void onStart()  — anything needed just after waitForStart()
 *   protected void onLoop()   — all per-tick robot logic except the turret
 *   protected void onStop()   — extra cleanup on OpMode end
 *
 * Values available to subclasses every tick
 * ──────────────────────────────────────────
 *   distanceToGoalInches  — range to tag in inches (−1 if not visible)
 *   turretCentred         — true when within dead-band
 *   centering             — direct access to the subsystem if needed
 */
public abstract class CameraTeleOp extends LinearOpMode {

    // ------------------------------------------------------------------
    // Vision subsystem — created by the subclass via factory method
    // ------------------------------------------------------------------
    /** The centering subsystem for this alliance. Set during init. */
    protected AprilTagCenteringBase centering;

    // ------------------------------------------------------------------
    // Cached turret state  (refreshed every tick — use freely in onLoop)
    // ------------------------------------------------------------------
    /** Last measured distance to the goal in inches. −1 = tag not visible. */
    protected double  distanceToGoalInches = -1.0;

    /** True when the turret is within the dead-band (effectively centred). */
    protected boolean turretCentred        = false;

    // ======================================================================
    //  Abstract factory — subclass supplies the correct centering instance
    // ======================================================================

    /**
     * Return the alliance-specific centering subsystem.
     * Called once during OpMode init.
     *
     * Examples:
     *   RedTeleOp  → return new RedGoalCentering();
     *   BlueTeleOp → return new BlueGoalCentering();
     */
    protected abstract AprilTagCenteringBase createCenteringSubsystem();

    // ======================================================================
    //  Subclass hooks  (Template Method pattern)
    // ======================================================================

    /** Called after turret subsystem is initialised. Override to init other hardware. */
    protected void onInit()  {}

    /** Called once immediately after waitForStart(). Override if needed. */
    protected void onStart() {}

    /**
     * Called every loop tick after turret update and state cache.
     * Override to add drivetrain, shooter, intake, or any other per-tick logic.
     *
     * distanceToGoalInches and turretCentred are already up to date when
     * this method is called.
     */
    protected void onLoop()  {}

    /** Called after the main loop exits. Override to stop extra subsystems. */
    protected void onStop()  {}

    // ======================================================================
    //  LinearOpMode entry point  (owned here — subclasses do NOT override)
    // ======================================================================

    @Override
    public final void runOpMode() {

        // ── Init ─────────────────────────────────────────────────────
        centering = createCenteringSubsystem();
        centering.init(hardwareMap, telemetry);
        onInit();

        telemetry.addLine(getClass().getSimpleName() + " ready — press START");
        telemetry.addLine("Hold gamepad2.right_bumper to align turret");
        telemetry.update();

        waitForStart();
        onStart();

        // ── Main loop ────────────────────────────────────────────────
        while (opModeIsActive()) {

            // Turret: update with button state
            centering.update(gamepad2.right_bumper);

            // Cache state for subclass use in onLoop()
            distanceToGoalInches = centering.getDistanceInches();
            turretCentred        = centering.isCentred();

            // Subclass logic (drive, shooter, intake, etc.)
            onLoop();

            // Common turret telemetry
            updateTurretTelemetry();

            idle();
        }

        // ── Stop ─────────────────────────────────────────────────────
        centering.stop();
        onStop();
    }

    // ======================================================================
    //  Telemetry
    // ======================================================================

    private void updateTurretTelemetry() {
        telemetry.addLine("── Turret ──────────────────────────────");
        telemetry.addData("Align active",   gamepad2.right_bumper ? "YES" : "no");
        telemetry.addData("Turret centred", turretCentred         ? "YES" : "no");

        if (centering.isTagVisible()) {
            telemetry.addData("Distance to goal", "%.2f in", distanceToGoalInches);
        } else {
            telemetry.addData("Distance to goal", "tag not visible");
        }

        // AprilTagCenteringBase appends its own detail lines (error, power, ticks)
        telemetry.update();
    }
}
