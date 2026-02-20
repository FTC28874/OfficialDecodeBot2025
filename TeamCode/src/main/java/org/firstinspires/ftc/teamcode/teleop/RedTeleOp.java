package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.turret.AprilTagCenteringBase;
import org.firstinspires.ftc.teamcode.turret.RedGoalCentering;

/**
 * RedTeleOp
 *
 * Main TeleOp routine for the RED alliance.
 * Extends CameraTeleOp which handles all turret / vision logic.
 *
 * Gamepad assignments
 * ───────────────────
 *   Gamepad 1 : Drivetrain  (add in onLoop() when ready)
 *   Gamepad 2 : Operator
 *     right_bumper HOLD    → turret tracks RED Goal (Tag 24)
 *     right_bumper RELEASE → turret ramps back to centre
 *
 * Add robot-specific logic by overriding:
 *   onInit()  — extra hardware init
 *   onLoop()  — drive, shooter, intake, etc.
 *   onStop()  — extra cleanup
 *
 * distanceToGoalInches and turretCentred are available every tick.
 */
@TeleOp(name = "Red TeleOp", group = "Red Alliance")
public class RedTeleOp extends CameraTeleOp {

    // ======================================================================
    //  Factory — tells CameraTeleOp which centering subsystem to use
    // ======================================================================

    @Override
    protected AprilTagCenteringBase createCenteringSubsystem() {
        return new RedGoalCentering();
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
        // Example for mecanum:
        //   double drive  = -gamepad1.left_stick_y;
        //   double strafe =  gamepad1.left_stick_x;
        //   double turn   =  gamepad1.right_stick_x;
        //   driveTrain.move(drive, strafe, turn);

        // TODO: Gamepad 2 other operator functions
        // distanceToGoalInches and turretCentred are ready to use here.
        // Example:
        //   if (gamepad2.a && turretCentred) {
        //       shooter.fire(distanceToGoalInches);
        //   }
    }

    @Override
    protected void onStop() {
        // TODO: stop drivetrain, shooter, intake, etc.
    }
}
