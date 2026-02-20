package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.turret.AprilTagCenteringBase;
import org.firstinspires.ftc.teamcode.turret.BlueGoalCentering;

/**
 * BlueTeleOp — BLUE alliance (AprilTag ID 20).
 * All robot logic inherited from NewTeleOp.
 */
@TeleOp(name = "Blue Alliance", group = "Camera TeleOp")
public class BlueTeleOp extends NewTeleOp {
    @Override
    protected AprilTagCenteringBase createCenteringSubsystem() {
        return new BlueGoalCentering();
    }
}
