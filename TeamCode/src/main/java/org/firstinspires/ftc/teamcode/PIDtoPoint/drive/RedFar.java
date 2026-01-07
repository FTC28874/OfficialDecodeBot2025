package org.firstinspires.ftc.teamcode.PIDtoPoint.drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name="Far From Red Goal DrivetoPoint", group="Auto")
public class RedFar extends LinearOpMode {

    DcMotor lf, rf, lb, rb;
    GoBildaPinpointDriver odo;
    DriveToPoint nav = new DriveToPoint(this);
    ElapsedTime timer = new ElapsedTime();

    enum State {
        DRIVE_START_TO_SHOOT,
        PAUSE_AFTER_START,
        PAUSE_FOR_SHOOT_1,

        DRIVE_SHOOT_TO_INTAKE_READY_2,
        PAUSE_AFTER_INTAKE_READY_2,
        DRIVE_INTAKE_READY_2_TO_INTAKE_2,
        PAUSE_AFTER_INTAKE_2,
        DRIVEBACK_TO_INTAKE_READY_2,

        DRIVE_INTAKE_2_TO_READY_TO_EMPTY,
        PAUSE_AFTER_READY_TO_EMPTY,
        DRIVE_READY_TO_EMPTY_TO_GATE,
        PAUSE_AFTER_GATE,
        DRIVE_GATE_TO_SHOOT_LINE,
        PAUSE_AFTER_SHOOT_LINE,
        PAUSE_FOR_SHOOT_2,

        DRIVE_SHOOT_TO_INTAKE_READY_1,
        PAUSE_AFTER_INTAKE_READY_1,
        DRIVE_INTAKE_READY_1_TO_INTAKE_1,
        PAUSE_AFTER_INTAKE_1,
        DRIVE_INTAKE_1_TO_SHOOT,
        PAUSE_FOR_SHOOT_3,

        DRIVE_SHOOT_TO_INTAKE_READY_3,
        PAUSE_AFTER_INTAKE_READY_3,
        DRIVE_INTAKE_READY_3_TO_INTAKE_3,
        PAUSE_AFTER_INTAKE_3,
        DRIVE_INTAKE_3_TO_SHOOT,
        PAUSE_FOR_SHOOT_4,

        DRIVE_SHOOT_TO_READY_TO_EMPTY_END,
        DONE
    }

    State state = State.DRIVE_START_TO_SHOOT;

    static Pose2D pose(double xIn, double yIn, double headingDeg) {
        return new Pose2D(
                DistanceUnit.MM,
                xIn * 25.4,
                yIn * 25.4,
                AngleUnit.DEGREES,
                headingDeg
        );
    }

    final Pose2D SHOOT_POSE = pose(88, 95, 0);
    final Pose2D READY_TO_EMPTY = pose(110, 66, 90);
    final Pose2D EMPTY_GATE = pose(120, 66, 90);
    final Pose2D SHOOTING_LINE = pose(88, 66, 90);
    final Pose2D READY_TO_EMPTY_END = pose(110, 66, 90);

    final Pose2D INTAKE_READY_SET_1 = pose(88, 84, 0);
    final Pose2D ACTUALLY_DO_INTAKE_SET_1 = pose(111, 84, 0);

    final Pose2D INTAKE_READY_SET_2 = pose(88, 60, 0);
    final Pose2D ACTUALLY_DO_INTAKE_SET_2 = pose(111, 60, 0);

    final Pose2D INTAKE_READY_SET_3 = pose(88, 36, 0);
    final Pose2D ACTUALLY_DO_INTAKE_SET_3 = pose(111, 36, 0);

    @Override
    public void runOpMode() {

        lf = hardwareMap.get(DcMotor.class, "driveFL");
        rf = hardwareMap.get(DcMotor.class, "driveFR");
        lb = hardwareMap.get(DcMotor.class, "driveBL");
        rb = hardwareMap.get(DcMotor.class, "driveBR");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.resetPosAndIMU();

        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {

            odo.update();
            Pose2D current = odo.getPosition();

            switch (state) {

                case DRIVE_START_TO_SHOOT:
                    telemetry.addData("Now in state", state);
                    if (nav.driveTo(current, SHOOT_POSE, 0.8, 0)) {
                        timer.reset();
                        state = State.PAUSE_AFTER_START;
                    }
                    break;

                case PAUSE_AFTER_START:
                    telemetry.addData("Now in state", state);
                    if (timer.seconds() >= 2.0)
                        state = State.PAUSE_FOR_SHOOT_1;
                    break;

                case PAUSE_FOR_SHOOT_1:
                    telemetry.addData("Now in state", state);
                    if (timer.seconds() >= 2.0)
                        state = State.DRIVE_SHOOT_TO_INTAKE_READY_2;
                    break;

                case DRIVE_SHOOT_TO_INTAKE_READY_2:
                    telemetry.addData("Now in state", state);
                    if (nav.driveTo(current, INTAKE_READY_SET_2, 0.7, 0)) {
                        timer.reset();
                        state = State.PAUSE_AFTER_INTAKE_READY_2;
                    }
                    break;

                case PAUSE_AFTER_INTAKE_READY_2:
                    telemetry.addData("Now in state", state);
                    if (timer.seconds() >= 2.0)
                        state = State.DRIVE_INTAKE_READY_2_TO_INTAKE_2;
                    break;

                case DRIVE_INTAKE_READY_2_TO_INTAKE_2:
                    telemetry.addData("Now in state", state);
                    if (nav.driveTo(current, ACTUALLY_DO_INTAKE_SET_2, 0.5, 0)) {
                        timer.reset();
                        state = State.PAUSE_AFTER_INTAKE_2;
                    }
                    break;

                case PAUSE_AFTER_INTAKE_2:
                    telemetry.addData("Now in state", state);
                    if (timer.seconds() >= 2.0)
                        state = State.DRIVEBACK_TO_INTAKE_READY_2;
                    break;

                case DRIVEBACK_TO_INTAKE_READY_2:
                    telemetry.addData("Now in state", state);
                    if (nav.driveTo(current, INTAKE_READY_SET_2, 0.6, 0)) {
                        timer.reset();
                        state = State.DRIVE_INTAKE_2_TO_READY_TO_EMPTY;
                    }
                    break;

                case DRIVE_INTAKE_2_TO_READY_TO_EMPTY:
                    telemetry.addData("Now in state", state);
                    if (nav.driveTo(current, READY_TO_EMPTY, 0.7, 0)) {
                        timer.reset();
                        state = State.PAUSE_AFTER_READY_TO_EMPTY;
                    }
                    break;

                case PAUSE_AFTER_READY_TO_EMPTY:
                    telemetry.addData("Now in state", state);
                    if (timer.seconds() >= 2.0)
                        state = State.DRIVE_READY_TO_EMPTY_TO_GATE;
                    break;

                case DRIVE_READY_TO_EMPTY_TO_GATE:
                    telemetry.addData("Now in state", state);
                    if (nav.driveTo(current, EMPTY_GATE, 0.6, 0)) {
                        timer.reset();
                        state = State.PAUSE_AFTER_GATE;
                    }
                    break;

                case PAUSE_AFTER_GATE:
                    telemetry.addData("Now in state", state);
                    if (timer.seconds() >= 2.0)
                        state = State.DRIVE_GATE_TO_SHOOT_LINE;
                    break;

                case DRIVE_GATE_TO_SHOOT_LINE:
                    telemetry.addData("Now in state", state);
                    if (nav.driveTo(current, SHOOTING_LINE, 0.7, 0)) {
                        timer.reset();
                        state = State.PAUSE_AFTER_SHOOT_LINE;
                    }
                    break;

                case PAUSE_AFTER_SHOOT_LINE:
                    telemetry.addData("Now in state", state);
                    if (timer.seconds() >= 2.0)
                        state = State.PAUSE_FOR_SHOOT_2;
                    break;

                case PAUSE_FOR_SHOOT_2:
                    telemetry.addData("Now in state", state);
                    if (timer.seconds() >= 2.0)
                        state = State.DRIVE_SHOOT_TO_INTAKE_READY_1;
                    break;

                case DRIVE_SHOOT_TO_INTAKE_READY_1:
                    telemetry.addData("Now in state", state);
                    if (nav.driveTo(current, INTAKE_READY_SET_1, 0.7, 0)) {
                        timer.reset();
                        state = State.PAUSE_AFTER_INTAKE_READY_1;
                    }
                    break;

                case PAUSE_AFTER_INTAKE_READY_1:
                    telemetry.addData("Now in state", state);
                    if (timer.seconds() >= 2.0)
                        state = State.DRIVE_INTAKE_READY_1_TO_INTAKE_1;
                    break;

                case DRIVE_INTAKE_READY_1_TO_INTAKE_1:
                    telemetry.addData("Now in state", state);
                    if (nav.driveTo(current, ACTUALLY_DO_INTAKE_SET_1, 0.5, 0)) {
                        timer.reset();
                        state = State.PAUSE_AFTER_INTAKE_1;
                    }
                    break;

                case PAUSE_AFTER_INTAKE_1:
                    telemetry.addData("Now in state", state);
                    if (timer.seconds() >= 2.0)
                        state = State.DRIVE_INTAKE_1_TO_SHOOT;
                    break;

                case DRIVE_INTAKE_1_TO_SHOOT:
                    telemetry.addData("Now in state", state);
                    if (nav.driveTo(current, SHOOT_POSE, 0.8, 0)) {
                        timer.reset();
                        state = State.PAUSE_FOR_SHOOT_3;
                    }
                    break;

                case PAUSE_FOR_SHOOT_3:
                    telemetry.addData("Now in state", state);
                    if (timer.seconds() >= 2.0)
                        state = State.DRIVE_SHOOT_TO_INTAKE_READY_3;
                    break;

                case DRIVE_SHOOT_TO_INTAKE_READY_3:
                    telemetry.addData("Now in state", state);
                    if (nav.driveTo(current, INTAKE_READY_SET_3, 0.7, 0)) {
                        timer.reset();
                        state = State.PAUSE_AFTER_INTAKE_READY_3;
                    }
                    break;

                case PAUSE_AFTER_INTAKE_READY_3:
                    telemetry.addData("Now in state", state);
                    if (timer.seconds() >= 2.0)
                        state = State.DRIVE_INTAKE_READY_3_TO_INTAKE_3;
                    break;

                case DRIVE_INTAKE_READY_3_TO_INTAKE_3:
                    telemetry.addData("Now in state", state);
                    if (nav.driveTo(current, ACTUALLY_DO_INTAKE_SET_3, 0.5, 0)) {
                        timer.reset();
                        state = State.PAUSE_AFTER_INTAKE_3;
                    }
                    break;

                case PAUSE_AFTER_INTAKE_3:
                    telemetry.addData("Now in state", state);
                    if (timer.seconds() >= 2.0)
                        state = State.DRIVE_INTAKE_3_TO_SHOOT;
                    break;

                case DRIVE_INTAKE_3_TO_SHOOT:
                    telemetry.addData("Now in state", state);
                    if (nav.driveTo(current, SHOOT_POSE, 0.8, 0)) {
                        timer.reset();
                        state = State.PAUSE_FOR_SHOOT_4;
                    }
                    break;

                case PAUSE_FOR_SHOOT_4:
                    telemetry.addData("Now in state", state);
                    if (timer.seconds() >= 2.0)
                        state = State.DRIVE_SHOOT_TO_READY_TO_EMPTY_END;
                    break;

                case DRIVE_SHOOT_TO_READY_TO_EMPTY_END:
                    telemetry.addData("Now in state", state);
                    if (nav.driveTo(current, READY_TO_EMPTY_END, 0.7, 0)) {
                        state = State.DONE;
                    }
                    break;

                case DONE:
                    telemetry.addData("Now in state", state);
                    break;
            }

            lf.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            rf.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            lb.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            rb.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

            telemetry.update();
        }
    }
}
