package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Point;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.Shooter;

@Autonomous(name="Far From Red Goal", group = "Autonomous")
public class autopathingfarfromredgoal extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer;

    /**
     * Enum for autonomous states. 
     * Consolidates 'Drive' and 'Pause' into single logical steps.
     */
    public enum PathState {
        START_TO_SHOOT_1,
        INTAKE_SET_2,
        BACK_TO_READY_SET_2,
        EMPTY_GATE,
        SHOOT_2,
        INTAKE_SET_1,
        SHOOT_3,
        INTAKE_SET_3,
        SHOOT_4,
        FINAL_EMPTY,
        PARK
    }

    private PathState pathState;

    // Field Coordinate Constants
    private final double SHOOT_X = 88.0;
    private final double SHOOT_Y = 95.0;
    private final double READY_X = 88.0;
    private final double INTAKE_X = 118.0;
    private final double GATE_X = 110.0;
    private final double GATE_Y = 66.0;

    // Y-offsets for intake sets
    private final double SET_1_Y = 84.0;
    private final double SET_2_Y = 60.0;
    private final double SET_3_Y = 36.0;

    // Poses built from constants
    private final Pose startPose = new Pose(88, 9, Math.toRadians(90));
    private final Pose shootPose = new Pose(SHOOT_X, SHOOT_Y, Math.toRadians(0));
    private final Pose gatePose  = new Pose(GATE_X, GATE_Y, Math.toRadians(90));
    private final Pose gateEmpty = new Pose(GATE_X + 10, GATE_Y, Math.toRadians(90));
    private final Pose shootLine = new Pose(READY_X, GATE_Y, Math.toRadians(90));

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
    }

    /**
     * Simplifies path building. Using 'follower.getPose()' ensures the 
     * path starts exactly where the robot currently is, preventing jerks.
     */
    private void follow(Pose target, boolean holdEnd) {
        follower.followPath(follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(target)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), target.getHeading())
                .build(), holdEnd);
    }

    private void followCurve(Point control, Pose target, boolean holdEnd) {
        follower.followPath(follower.pathBuilder()
                .addPath(new BezierCurve(new Point(follower.getPose()), control, new Point(target)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), target.getHeading())
                .build(), holdEnd);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case START_TO_SHOOT_1:
                if (!follower.isBusy()) {
                    follow(shootPose, true);
                    setPathState(PathState.INTAKE_SET_2);
                }
                break;

            case INTAKE_SET_2:
                // Wait for shot action (2s) then move to intake
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.0) {
                    // Glide to 'Ready' (holdEnd=false) then drive to 'Do' (holdEnd=true)
                    follower.followPath(follower.pathBuilder()
                            .addPath(new BezierLine(new Point(shootPose), new Point(READY_X, SET_2_Y)))
                            .setLinearHeadingInterpolation(shootPose.getHeading(), 0)
                            .addPath(new BezierLine(new Point(READY_X, SET_2_Y), new Point(INTAKE_X, SET_2_Y)))
                            .setConstantHeadingInterpolation(0)
                            .build(), true);
                    setPathState(PathState.BACK_TO_READY_SET_2);
                }
                break;

            case BACK_TO_READY_SET_2:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.0) {
                    follow(new Pose(READY_X, SET_2_Y, 0), true);
                    setPathState(PathState.EMPTY_GATE);
                }
                break;

            case EMPTY_GATE:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.0) {
                    // Sequence: To Gate -> Empty -> To Shooting Line
                    follower.followPath(follower.pathBuilder()
                            .addPath(new BezierLine(new Point(follower.getPose()), new Point(gatePose)))
                            .setLinearHeadingInterpolation(0, Math.toRadians(90))
                            .addPath(new BezierLine(new Point(gatePose), new Point(gateEmpty)))
                            .setConstantHeadingInterpolation(Math.toRadians(90))
                            .addPath(new BezierLine(new Point(gateEmpty), new Point(shootLine)))
                            .setConstantHeadingInterpolation(Math.toRadians(90))
                            .build(), true);
                    setPathState(PathState.SHOOT_2);
                }
                break;

            case SHOOT_2:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.0) {
                    follow(shootPose, true);
                    setPathState(PathState.INTAKE_SET_1);
                }
                break;

            case INTAKE_SET_1:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.0) {
                    // Example of using a Curve to smooth the turn from Shoot to Intake
                    followCurve(new Point(READY_X + 10, SET_1_Y + 10), new Pose(INTAKE_X, SET_1_Y, 0), true);
                    setPathState(PathState.SHOOT_3);
                }
                break;

            case SHOOT_3:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.0) {
                    follow(shootPose, true);
                    setPathState(PathState.INTAKE_SET_3);
                }
                break;

            case INTAKE_SET_3:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.0) {
                    follow(new Pose(INTAKE_X, SET_3_Y, 0), true);
                    setPathState(PathState.SHOOT_4);
                }
                break;

            case SHOOT_4:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.0) {
                    follow(shootPose, true);
                    setPathState(PathState.FINAL_EMPTY);
                }
                break;

            case FINAL_EMPTY:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2.0) {
                    follow(gatePose, true);
                    setPathState(PathState.PARK);
                }
                break;

            case PARK:
                break;
        }
    }

    @Override
    public void runOpMode() {
        Shooter.init(hardwareMap);
        pathTimer = new Timer();
        actionTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        setPathState(PathState.START_TO_SHOOT_1);

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            autonomousPathUpdate();

            telemetry.addData("State", pathState);
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Busy", follower.isBusy());
            telemetry.update();
        }
    }
}