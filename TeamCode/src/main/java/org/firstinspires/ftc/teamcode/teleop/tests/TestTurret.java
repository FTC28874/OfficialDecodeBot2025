package org.firstinspires.ftc.teamcode.teleop.tests;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PIDtoPoint.drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.robot.Turret;

@TeleOp(name = "Test Turret 123", group = "Linear OpMode")
public class TestTurret extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    GoBildaPinpointDriver odo = null;

    @Override
    public void runOpMode() {

        Turret.init(hardwareMap);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        odo.setOffsets(-48, -156); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {

            if (gamepad1.xWasPressed()) {
                Turret.zeroTurret();
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
            telemetry.addData("x: ", odo.getXOffset(INCH));
            telemetry.addData("y: ", odo.getYOffset(INCH));
            telemetry.addData("heading: ", odo.getHeading(DEGREES));
            telemetry.update();

        }

    }

}
