package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.DynamicShooter;
import org.firstinspires.ftc.teamcode.robot.HelperServos;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.turret.AprilTagCenteringBase;

/**
 * NewTeleOp
 *
 * Abstract base TeleOp. Owns the main loop, vision/turret subsystem,
 * drivetrain, shooter, intake, and all shared robot logic.
 *
 * Subclasses (RedTeleOp, BlueTeleOp) only need to implement
 * createCenteringSubsystem() to supply the correct alliance tag tracker.
 *
 * Distance pipeline
 * ─────────────────
 *   AprilTag range (inches) → DynamicShooter.calcTargetRPM / calcHoodPos
 *   Tag not visible → shooter holds last known values (no blind firing)
 *
 * Gamepad map
 * ───────────
 *  Gamepad 1
 *    left_stick        drive (axial + strafe)
 *    right_stick_x     yaw
 *    dpad_up/down      far / close shooter preset
 *    dpad_left/right   mid-mid / mid shooter preset
 *    y                 reset hood to minimum angle
 *
 *  Gamepad 2
 *    right_trigger     HOLD → camera-based turret alignment
 *    right_bumper      toggle shooter on/off
 *                      (also brakes drivetrain while shooter is active)
 *    b                 toggle dynamic shooting mode
 *    dpad_up/down      shooter RPM +/− 100
 *    dpad_right/left   hood angle +/− 0.05
 *    left_bumper       run intake + raise + stopper stop
 *    x                 pass ring / reverse intake (+ left_bumper)
 *    y                 lower intake
 */
public abstract class NewTeleOp extends LinearOpMode {

    // ------------------------------------------------------------------
    // Vision / turret subsystem — supplied by subclass
    // ------------------------------------------------------------------
    protected AprilTagCenteringBase centering;

    // Cached every tick from centering subsystem
    protected double  distanceToGoalInches = -1.0;
    protected boolean turretCentred        = false;

    // ------------------------------------------------------------------
    // Drivetrain
    // ------------------------------------------------------------------
    protected DcMotor driveFL, driveBL, driveFR, driveBR;

    // ------------------------------------------------------------------
    // Shooter
    // ------------------------------------------------------------------
    protected double  shooterTargetRPM = 3000;
    protected double  shooterPower     = 0;
    protected boolean shooterState     = true;

    // ------------------------------------------------------------------
    // Hood
    // ------------------------------------------------------------------
    protected final double minHoodAngle = Shooter.HoodState.DOWN.angle;
    protected final double maxHoodAngle = Shooter.HoodState.UP.angle;
    protected double       curHoodAngle = minHoodAngle;

    // ------------------------------------------------------------------
    // Dynamic shooting
    // ------------------------------------------------------------------
    protected boolean isDynamic = false;

    // ======================================================================
    //  Abstract factory — subclass returns the correct centering instance
    // ======================================================================

    /**
     * Return the alliance-specific centering subsystem.
     *   RedTeleOp  → return new RedGoalCentering();
     *   BlueTeleOp → return new BlueGoalCentering();
     */
    protected abstract AprilTagCenteringBase createCenteringSubsystem();

    // ======================================================================
    //  Main loop
    // ======================================================================

    @Override
    public final void runOpMode() {
        initHardware();

        centering = createCenteringSubsystem();
        centering.init(hardwareMap, telemetry);

        telemetry.addLine(getClass().getSimpleName() + " ready — press START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ── Vision / turret ───────────────────────────────────────
            centering.update(gamepad2.right_trigger > 0.1);
            distanceToGoalInches = centering.getDistanceInches();
            turretCentred        = centering.isCentred();

            // ── Dynamic shooting ──────────────────────────────────────
            if (isDynamic && centering.isTagVisible()) {
                if (distanceToGoalInches >= 130) {
                    shooterTargetRPM = 2900;
                    curHoodAngle     = maxHoodAngle;
                } else {
                    shooterTargetRPM = DynamicShooter.calcTargetRPM(distanceToGoalInches);
                    curHoodAngle     = DynamicShooter.calcHoodPos(distanceToGoalInches);
                }
            }
            if (gamepad2.bWasPressed()) isDynamic = !isDynamic;

            // ── Drivetrain ────────────────────────────────────────────
            // Shooter active → brake all four wheels (defensive lock).
            // Shooter off    → normal mecanum drive.
            if (shooterState) {
                setDrivetrainBrake(true);
                driveFL.setPower(0);
                driveFR.setPower(0);
                driveBL.setPower(0);
                driveBR.setPower(0);
            } else {
                setDrivetrainBrake(false);

                double axial   = -gamepad1.left_stick_y;
                double lateral =  gamepad1.left_stick_x;
                double yaw     =  gamepad1.right_stick_x;

                double powerFL = axial + lateral + yaw;
                double powerFR = axial - lateral - yaw;
                double powerBL = axial - lateral + yaw;
                double powerBR = axial + lateral - yaw;

                double max = Math.max(Math.abs(powerFL), Math.abs(powerFR));
                max = Math.max(max, Math.abs(powerBL));
                max = Math.max(max, Math.abs(powerBR));
                if (max > 1.0) {
                    powerFL /= max; powerFR /= max;
                    powerBL /= max; powerBR /= max;
                }

                driveFL.setPower(powerFL);
                driveFR.setPower(powerFR);
                driveBL.setPower(powerBL);
                driveBR.setPower(powerBR);
            }

            // ── Shooter ───────────────────────────────────────────────
            if (shooterState) {
                shooterPower = Shooter.PIDControl(shooterTargetRPM, Shooter.getCurrentRPM());
                Shooter.setShooterPower(shooterPower);
            } else {
                Shooter.stopShooter();
            }
            if (gamepad2.rightBumperWasPressed()) shooterState = !shooterState;

            if (gamepad2.dpadUpWasPressed())   shooterTargetRPM += 100;
            if (gamepad2.dpadDownWasPressed()) shooterTargetRPM -= 100;

            // ── Hood ──────────────────────────────────────────────────
            Shooter.setShooterPosition(curHoodAngle);
            if (gamepad2.dpadRightWasPressed() && curHoodAngle < maxHoodAngle) curHoodAngle += 0.05;
            if (gamepad2.dpadLeftWasPressed()  && curHoodAngle > minHoodAngle) curHoodAngle -= 0.05;
            if (gamepad1.yWasPressed()) curHoodAngle = minHoodAngle;

            // ── Intake ────────────────────────────────────────────────
            if (gamepad2.left_bumper) {
                Intake.runIntake();
                HelperServos.setStopperStop();
                Intake.raiseIntake();
            } else {
                Intake.stopIntake();
            }
            if (gamepad2.x && !gamepad2.left_bumper) {
                Intake.runIntake();
                HelperServos.setStopperPass();
            }
            if (gamepad2.x && gamepad2.left_bumper) Intake.reverseIntake();
            if (gamepad2.y) Intake.lowerIntake();

            // ── Driver presets ────────────────────────────────────────
            if (gamepad1.dpadUpWasPressed()) {
                shooterTargetRPM = Shooter.ShootPositionState.FAR_RPM.position;
                curHoodAngle     = Shooter.ShootPositionState.FAR_HOOD.position;
            }
            if (gamepad1.dpadDownWasPressed()) {
                shooterTargetRPM = Shooter.ShootPositionState.CLOSE_RPM.position;
                curHoodAngle     = Shooter.ShootPositionState.CLOSE_HOOD.position;
            }
            if (gamepad1.dpadLeftWasPressed()) {
                shooterTargetRPM = Shooter.ShootPositionState.MID_MID_RPM.position;
                curHoodAngle     = Shooter.ShootPositionState.MID_MID_HOOD.position;
            }
            if (gamepad1.dpadRightWasPressed()) {
                shooterTargetRPM = Shooter.ShootPositionState.MID_RPM.position;
                curHoodAngle     = Shooter.ShootPositionState.MID_HOOD.position;
            }

            // ── Telemetry ─────────────────────────────────────────────
            telemetry.addLine("══ " + getClass().getSimpleName() + " ═══════════════════════");
            telemetry.addData("Dynamic mode",  isDynamic    ? "ON"     : "off");
            telemetry.addData("Drive brake",   shooterState ? "LOCKED" : "normal");
            telemetry.addData("Tag visible",   centering.isTagVisible() ? "YES" : "NO");
            if (centering.isTagVisible())
                telemetry.addData("Distance",  "%.2f in", distanceToGoalInches);
            telemetry.addLine("── Shooter ─────────────────────────────");
            telemetry.addData("Shooter ON",    shooterState);
            telemetry.addData("Target RPM",    "%.0f", shooterTargetRPM);
            telemetry.addData("Current RPM",   "%.0f", Shooter.getCurrentRPM());
            telemetry.addData("Hood angle",    "%.3f", curHoodAngle);
            telemetry.update();

            idle();
        }

        // ── Stop ──────────────────────────────────────────────────────
        centering.stop();
        driveFL.setPower(0); driveFR.setPower(0);
        driveBL.setPower(0); driveBR.setPower(0);
        Shooter.stopShooter();
        Intake.stopIntake();
    }

    // ======================================================================
    //  Hardware init — called once before waitForStart
    // ======================================================================

    protected void initHardware() {
        driveFL = hardwareMap.get(DcMotor.class, "driveFL");
        driveBL = hardwareMap.get(DcMotor.class, "driveBL");
        driveFR = hardwareMap.get(DcMotor.class, "driveFR");
        driveBR = hardwareMap.get(DcMotor.class, "driveBR");

        driveFL.setDirection(DcMotor.Direction.REVERSE);
        driveBL.setDirection(DcMotor.Direction.REVERSE);
        driveFR.setDirection(DcMotor.Direction.FORWARD);
        driveBR.setDirection(DcMotor.Direction.FORWARD);

        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Shooter.init(hardwareMap);
        Intake.init(hardwareMap);
        HelperServos.init(hardwareMap);
    }

    // ======================================================================
    //  Drivetrain brake helper
    // ======================================================================

    private void setDrivetrainBrake(boolean brake) {
        DcMotor.ZeroPowerBehavior mode = brake
                ? DcMotor.ZeroPowerBehavior.BRAKE
                : DcMotor.ZeroPowerBehavior.FLOAT;
        driveFL.setZeroPowerBehavior(mode);
        driveFR.setZeroPowerBehavior(mode);
        driveBL.setZeroPowerBehavior(mode);
        driveBR.setZeroPowerBehavior(mode);
    }
}
