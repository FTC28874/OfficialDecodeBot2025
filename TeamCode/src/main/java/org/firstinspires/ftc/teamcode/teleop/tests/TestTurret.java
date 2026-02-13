package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.PIDtoPoint.drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.robot.Turret;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Test Turret 123", group = "Linear OpMode")
public class TestTurret extends LinearOpMode {
    int i = 0;
    private final int READ_PERIOD = 1;
    private static final double MOTOR_SPEED = 0.4;
    private static final double DEADZONE_PX = 10; // stop when |x| <= this

    private HuskyLens huskyLens;
    private DcMotor alignMotor= null;
    private ElapsedTime runtime = new ElapsedTime();
    GoBildaPinpointDriver odo = null;

    @Override
    public void runOpMode() {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        /*
         * This sample rate limits the reads solely to allow a user time to observe
         * what is happening on the Driver Station telemetry.  Typical applications
         * would not likely rate limit.
         */
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        /*
         * Immediately expire so that the first time through we'll do the read.
         */
        rateLimit.expire();

        /*
         * Basic check to see if the device is alive and communicating.  This is not
         * technically necessary here as the HuskyLens class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        /*
         * The device uses the concept of an algorithm to determine what types of
         * objects it will look for and/or what mode it is in.  The algorithm may be
         * selected using the scroll wheel on the device, or via software as shown in
         * the call to selectAlgorithm().
         *
         * The SDK itself does not assume that the user wants a particular algorithm on
         * startup, and hence does not set an algorithm.
         *
         * Users, should, in general, explicitly choose the algorithm they want to use
         * within the OpMode by calling selectAlgorithm() and passing it one of the values
         * found in the enumeration HuskyLens.Algorithm.
         *
         * Other algorithm choices for FTC might be: OBJECT_RECOGNITION, COLOR_RECOGNITION or OBJECT_CLASSIFICATION.
         */
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.update();
        waitForStart();
        alignMotor = hardwareMap.get(DcMotor.class, "turret");
        alignMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        alignMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Flip this if the motor spins the wrong way
        alignMotor.setDirection(DcMotor.Direction.REVERSE);
        alignMotor.setPower(0.2);

        Turret.init(hardwareMap);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        odo.setOffsets(-48, -156); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            HuskyLens.Block[] blocks = huskyLens.blocks();
            for (HuskyLens.Block block : blocks){
                telemetry.addData("Block value: ", block.id);
                if (block.id == 1){

                    telemetry.addLine("Red goal (tag id 1) found!");
                    double x = 160 - block.x;
                    telemetry.addData("ID Dist from center: ", x);
                    if (Math.abs(x) <= DEADZONE_PX) {
                        // Centered
                        alignMotor.setPower(0);
                        telemetry.addLine("Aligned: motor stopped");
                    }
                    else if (x > 0) {
                        // Tag is to the right → move left
                        alignMotor.setPower(-MOTOR_SPEED);
                        telemetry.addLine("Tag right → motor left");
                    }
                    else {
                        // Tag is to the left → move right
                        alignMotor.setPower(MOTOR_SPEED);
                        telemetry.addLine("Tag left → motor right");
                    }

                    telemetry.addData("Tag X (cm)", x);

                }
                else {
                    // No tag detected → stop motor for safety
                    alignMotor.setPower(0);
                    telemetry.addLine("Tag not detected");
                }
                }

            }

//
//            if (blocks.length == 0) {
//                alignMotor.setPower(0);
//
//


            telemetry.update();
        }
//            odo.update();
//            // Telemetry
//
//            telemetry.update();

        }




