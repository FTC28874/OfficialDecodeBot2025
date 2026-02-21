package org.firstinspires.ftc.teamcode.teleop.tests;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.PIDtoPoint.drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.robot.Turret;
@Disabled

@TeleOp(name = "Test Turret 123", group = "Linear OpMode")
public class TestTurret extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    GoBildaPinpointDriver odo = null;

    @Override
    public void runOpMode() {

        Turret.init(hardwareMap);
        Pose2D pos = odo.getPosition();
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        odo.setOffsets(-48, -156); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            double angle = 0.0;
            if (gamepad1.xWasPressed()) {
                Turret.zeroTurret();
            }
            if (gamepad1.y) {
                //urret.update();
            }
            if (gamepad1.b) {
                Turret.setTargetAngle(20.0);
            }
            if (gamepad1.dpad_down) {
                Turret.stop();
            }
            if (gamepad1.a) {
                Turret.setTargetAngle(angle);
            }
//            if (gamepad1.y) {
//                Turret.setTurretAngle(0.0);
//            }
//            if (gamepad1.b) {
//
//                Turret.setTurretAngle(45.0);
//            }
//            if (gamepad1.a) {
//                Turret.aimAtRedGoal(odo.getXOffset(INCH) - 4, odo.getYOffset(INCH), odo.getHeading(DEGREES));
//            }
            odo.update();
            // Telemetry
            telemetry.addData("=== CONTROLS ===", "");
            telemetry.addData("X", "Zero turret (face forward first!)");
            telemetry.addData("A", "Auto-aim at goal");
            telemetry.addData("B", "Stop turret");
            telemetry.addData("Y", "Reset goal position");
            telemetry.addData("DPad", "Set angle (Up=0, Right=90, Down=180, Left=-90)");
            telemetry.addData("LB/RB", "Manual rotate left/right");
            telemetry.addData("LT/RT", "Adjust goal closer/further");
            telemetry.addData("", "");

            telemetry.addData("=== TURRET STATUS ===", "");
            telemetry.addData("Current Angle", "%.1f°", Turret.getCurrentAngle());
            telemetry.addData("Target Angle", "%.1f°", Turret.getTargetAngle());
            telemetry.addData("Error", "%.1f°", Turret.getError());
            telemetry.addData("At Target?", Turret.isAtTarget(3.0) ? "YES" : "NO");
            telemetry.addData("", "");

            telemetry.addData("=== ROBOT STATUS ===", "");
            telemetry.addData("Robot X", "%.1f in",
                    odo.getXOffset(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH));
            telemetry.addData("Robot Y", "%.1f in",
                    odo.getYOffset(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH));
            telemetry.addData("Robot Heading", "%.1f°",
                    odo.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES));
            telemetry.addData("Distance to Goal", "%.1f in", Turret.getDistanceToGoal(odo.getPosition()));

            telemetry.update();

        }

    }

}
