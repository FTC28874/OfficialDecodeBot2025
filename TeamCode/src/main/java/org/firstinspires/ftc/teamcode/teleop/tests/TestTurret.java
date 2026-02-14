package org.firstinspires.ftc.teamcode.teleop.tests;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.PIDtoPoint.drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.robot.Turret;

import java.util.concurrent.TimeUnit;


@TeleOp(name = "Test Turret 123", group = "Linear OpMode")
public class TestTurret extends LinearOpMode {
    private HuskyLens huskyLens;


    private ElapsedTime runtime = new ElapsedTime();
    GoBildaPinpointDriver odo = null;
    private final int READ_PERIOD = 1;
    private DcMotor alignMotor= null;

    @Override
    public void runOpMode() {

        Turret.init(hardwareMap);
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        odo.setOffsets(-48, -156); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        odo.resetPosAndIMU();
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (HuskyLens.Block block : blocks) {

                if (block.id == 1){

                    telemetry.addLine("Red goal (tag id 1) found!");
                    double x = 160 - block.x;
                    telemetry.addData("ID Dist from center: ", x);
                    telemetry.update();

            }
                else if(block.id == 0){
                    telemetry.addLine("No Tag/Unknown tag");
                    telemetry.update();
                }
            telemetry.update();

        }

    }

}}
