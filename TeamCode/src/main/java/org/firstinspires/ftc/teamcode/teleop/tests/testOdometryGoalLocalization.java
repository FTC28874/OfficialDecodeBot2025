package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.Constants;

@TeleOp(name = "Pedro Turret Auto-Aim", group = "Linear OpMode")
public class testOdometryGoalLocalization extends LinearOpMode {
    // 1. Placeholder Goal Coordinates (e.g., Red Goal)
    private final double goal_x = 144.0;
    private final double goal_y = 144.0;

    // 2. Placeholder Start Pose (X, Y, Heading in Radians)
    // Adjust these based on your actual starting position in 2026 matches
    private final Pose startPose = new Pose(88, 9, Math.toRadians(0));
    private HardwareMap hardwareMap;
    private Follower follower;
    private DcMotorEx turretMotor;

    // Simple PID constants - tune these for your specific turret weight
    private double kP = 0.8, kI = 0.0, kD = 0.05;
    private double lastError = 0;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        // CRITICAL: Tell Pedro Pathing where the robot is at the start
        follower.setStartingPose(startPose);

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            // Update odometry pods continuously
            follower.update();
            Pose currentPose = follower.getPose();

            // 3. Calculate target angle based on field position
            double dx = goal_x - currentPose.getX();
            double dy = goal_y - currentPose.getY();
            double absoluteAngleToGoal = Math.atan2(dy, dx);

            // 4. Subtract robot heading to find the turret-relative target
            double targetRelativeAngle = absoluteAngleToGoal - currentPose.getHeading();

            // 5. Normalize angle to ensure the turret takes the shortest path [-PI, PI]
            while (targetRelativeAngle > Math.PI) targetRelativeAngle -= 2 * Math.PI;
            while (targetRelativeAngle < -Math.PI) targetRelativeAngle += 2 * Math.PI;

            // 6. Execute PID Control (Placeholder current angle logic)
            double currentTurretRad = getTurretRadians();
            double error = targetRelativeAngle - currentTurretRad;
            double power = (error * kP) + ((error - lastError) * kD);

            turretMotor.setPower(power);
            lastError = error;

            // Telemetry for debugging in the 2026 season
            telemetry.addData("Robot X", currentPose.getX());
            telemetry.addData("Robot Y", currentPose.getY());
            telemetry.addData("Target Angle (Deg)", Math.toDegrees(targetRelativeAngle));
            telemetry.update();
        }
    }

    private double getTurretRadians() {
        // Map turret encoder ticks to radians here
        // Example: (ticks / ticks_per_rev) * 2 * PI
        return (turretMotor.getCurrentPosition() / 2000.0) * 2 * Math.PI;
    }
}
