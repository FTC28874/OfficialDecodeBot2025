package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.turret.AprilTagCenteringBase;
import org.firstinspires.ftc.teamcode.turret.RedGoalCentering;

/**
 * RedTeleOp — RED alliance (AprilTag ID 24).
 * All robot logic inherited from NewTeleOp.
 */
@TeleOp(name = "Red TeleOp", group = "Red Alliance")
public class RedTeleOp extends NewTeleOp {
    @Override
    protected AprilTagCenteringBase createCenteringSubsystem() {
        return new RedGoalCentering();
    }
}
