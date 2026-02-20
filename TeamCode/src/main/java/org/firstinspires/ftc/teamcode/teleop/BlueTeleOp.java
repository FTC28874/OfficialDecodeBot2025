package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.turret.AprilTagCenteringBase;
import org.firstinspires.ftc.teamcode.turret.BlueGoalCentering;

/**
 * BlueTeleOp
 *
 * Main TeleOp routine for the BLUE alliance.
 * Mirrors RedTeleOp exactly — only the centering subsystem differs.
 * Extends CameraTeleOp which handles all turret / vision logic.
 *
 * Gamepad assignments
 * ───────────────────
 *   Gamepad 1 : Drivetrain  (add in onLoop() when ready)
 *   Gamepad 2 : Operator
 *     right_bumper HOLD    → turret tracks BLUE Goal (Tag 20)
 *     right_bumper RELEASE → turret ramps back to centre
 *
 * Add robot-specific logic by overriding:
 *   onInit()  — extra hardware init
 *   onLoop()  — drive, shooter, intake, etc.
 *   onStop()  — extra cleanup
 *
 * distanceToGoalInches and turretCentred are available every tick.
 */
@TeleOp(name = "Blue TeleOp", group = "Blue Alliance")
public class BlueTeleOp extends CameraTeleOp {

    // ======================================================================
    //  Factory — tells CameraTeleOp which centering subsystem to use
    // ======================================================================

    @Override
    protected AprilTagCenteringBase createCenteringSubsystem() {
        return new BlueGoalCentering();
    }

    // ======================================================================
    //  Subclass hooks
    // ======================================================================

    @Override
    protected void onInit() {
        // TODO: initialise drivetrain, shooter, intake, etc.
    }

    @Override
    protected void onLoop() {
        // TODO: Gamepad 1 drivetrain control
        // TODO: Gamepad 2 other operator functions
        // distanceToGoalInches and turretCentred are ready to use here.
    }

    @Override
    protected void onStop() {
        // TODO: stop drivetrain, shooter, intake, etc.
    }
}
