package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.PIDtoPoint.drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.Turret;

@TeleOp(name = "Test Turret", group = "Linear OpMode")
public class TestAtanTurret extends LinearOpMode {

    private GoBildaPinpointDriver odo = null;
    private ElapsedTime runtime = new ElapsedTime();
    Pose2D pos = null;
    private double targetTurretAngle = 0.0;
    private int targetTurretTicks = 0;
    private double currentTurretTicks = 0.0;
    private double distToTarget = 0.0;
    private DcMotorEx turret = null;
    boolean turretAimToggle = false;
    private final int DEAD_ZONE = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        Turret.init(hardwareMap);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        turret = hardwareMap.get(DcMotorEx.class, "turret");

        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        runtime.reset();

        odo.resetPosAndIMU();
        waitForStart();
        while (opModeIsActive()) {
            pos = odo.getPosition();
            odo.update();
            targetTurretAngle = Turret.calculateTurretHeading(pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES), 72.0, 72.0);
            targetTurretTicks = Turret.degreesToTicks(targetTurretAngle);
            currentTurretTicks = turret.getCurrentPosition();
            distToTarget = Math.abs(targetTurretTicks - currentTurretTicks);
            double power = distToTarget / 100;
            if (power > 1.0) {
                power = 1.0;
            }
            if (power < 0.0) {
                power = 0.0;
            }

            if (gamepad1.xWasPressed()) {
                turretAimToggle = !turretAimToggle;
            }
            if (turretAimToggle) {
                if (targetTurretAngle > currentTurretTicks) {
                    turret.setDirection(DcMotorSimple.Direction.FORWARD);
                    turret.setPower(power);
                } else if (targetTurretAngle < currentTurretTicks) {
                    turret.setDirection(DcMotorSimple.Direction.REVERSE);
                    turret.setPower(power);
                } else if (distToTarget <= DEAD_ZONE) {
                    turret.setPower(0);
                }
                if (currentTurretTicks >= 500 || currentTurretTicks <= -500) {
                    turret.setPower(0.0);
                }
            } else {
                turret.setPower(0.0);
            }

            telemetry.addData("Target Angle: ", targetTurretAngle);
            telemetry.addData("Target Tick: ", targetTurretTicks);
            telemetry.addData("Turret Aim Toggle", turretAimToggle);
            telemetry.addLine("");
            telemetry.addLine("=========== Positioning ===========");
            telemetry.addData("X Position", pos.getX(DistanceUnit.INCH));
            telemetry.addData("Y Position", pos.getY(DistanceUnit.INCH));
            telemetry.addData("Heading", pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Current Turret Position", turret.getCurrentPosition());
            telemetry.update();

        }
    }
}
