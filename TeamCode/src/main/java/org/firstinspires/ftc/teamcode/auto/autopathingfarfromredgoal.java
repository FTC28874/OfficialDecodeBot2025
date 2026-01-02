package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Shooter;

@Autonomous(name="Far From Red Goal", group = "Linear OpMode")
public class autopathingfarfromredgoal extends LinearOpMode {

    private double shooterEncSpeed = 1600;
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {

        DRIVE_STARTPOS_TO_SHOOTING_POS,
        PAUSE_2S_AFTER_STARTPOS_TO_SHOOTING_POS,

        PAUSE_FOR_SHOOT_1,

        DRIVE_SHOOTPOS_TO_INTAKE_READY_SET_2_POS,
        PAUSE_2S_AFTER_SHOOTPOS_TO_INTAKE_READY_SET_2_POS,

        DRIVE_INTAKE_READY_POSE_SET_2_TO_ACTUALLY_DO_INTAKE_SET_2,
        PAUSE_2S_AFTER_INTAKE_READY_SET_2,


        DRIVEBACK_INTAKE_READY_SET_2_POS,
        PAUSE_2S_AFTER_DRIVEBACK_INTAKE_READY_SET_2_POS,

        DRIVE_ACTUALLY_DO_INTAKE_SET_2_TO_READY_TO_EMPTY,
        PAUSE_2S_AFTER_ACTUALLY_DO_INTAKE_SET_2,

        DRIVE_READY_TO_EMPTY_TO_EMPTY_GATE,
        PAUSE_2S_AFTER_READY_TO_EMPTY,

        DRIVE_EMPTY_GATE_TO_GO_TO_SHOOTING_LINE,
        PAUSE_2S_AFTER_EMPTY_GATE,

        DRIVE_GO_TO_SHOOTING_LINE_TO_SHOOT_POSE,
        PAUSE_2S_AFTER_GO_TO_SHOOTING_LINE,

        PAUSE_FOR_SHOOT_2,

        DRIVE_SHOOT_POSE_TO_INTAKE_READY_POSE_SET_1,
        PAUSE_2S_AFTER_SHOOT_TO_INTAKE_READY_SET_1,

        DRIVE_INTAKE_READY_POSE_SET_1_TO_ACTUALLY_DO_INTAKE_SET_1,
        PAUSE_2S_AFTER_INTAKE_READY_SET_1,

        DRIVE_ACTUALLY_DO_INTAKE_SET_1_TO_SHOOT_POSE,
        PAUSE_2S_AFTER_ACTUALLY_DO_INTAKE_SET_1,

        PAUSE_FOR_SHOOT_3,

        DRIVE_SHOOT_POSE_TO_INTAKE_READY_POSE_SET_3,
        PAUSE_2S_AFTER_SHOOT_TO_INTAKE_READY_SET_3,

        DRIVE_INTAKE_READY_POSE_SET_3_TO_ACTUALLY_DO_INTAKE_SET_3,
        PAUSE_2S_AFTER_INTAKE_READY_SET_3,

        DRIVE_ACTUALLY_DO_INTAKE_SET_3_TO_SHOOT_POSE,
        PAUSE_2S_AFTER_ACTUALLY_DO_INTAKE_SET_3,

        PAUSE_FOR_SHOOT_4,

        DRIVE_SHOOT_POSE_TO_READY_TO_EMPTY_END,
        PAUSE_2S_AFTER_READY_TO_EMPTY_END
    }

    PathState pathState;
    private final double start_x = 88;
    private final double start_y = 9;
    private final double shoot_x = start_x;
    private final double shoot_y = 95;

    private final double gate_x = 110;
    private final double gate_y = 66;

    private final double ready_intake_x = start_x;
    private final double do_intake_x = ready_intake_x + 30;
    private final double set_1_y = 84;
    private final double set_2_y = set_1_y - 24;
    private final double set_3_y = set_2_y - 24;

    private final Pose startPose = new Pose(start_x, start_y, Math.toRadians(90));
    private final Pose shootPose = new Pose(shoot_x, shoot_y, Math.toRadians(0));
    private final Pose intakeReadyPoseSet2 = new Pose(ready_intake_x, set_2_y, Math.toRadians(0));
    private final Pose actuallyDoIntakeSet2 = new Pose(do_intake_x, set_2_y, Math.toRadians(0));
    private final Pose readyToEmpty = new Pose(gate_x, gate_y, Math.toRadians(90));
    private final Pose emptyGate = new Pose(gate_x + 10, gate_y, Math.toRadians(90));
    private final Pose goToShootingLine = new Pose(ready_intake_x, gate_y, Math.toRadians(90));

    private final Pose intakeReadyPoseSet1 = new Pose(ready_intake_x, set_1_y, Math.toRadians(0));
    private final Pose actuallyDoIntakeSet1 = new Pose(do_intake_x, set_1_y, Math.toRadians(0));

    private final Pose IntakeReadyPoseSet3 = new Pose(ready_intake_x, set_3_y, Math.toRadians(0));
    private final Pose actuallyDoIntakeSet3 = new Pose(do_intake_x, set_3_y, Math.toRadians(0));

    private final Pose readyToEmptyEnd =  new Pose(gate_x, gate_y, Math.toRadians(90));
    private PathChain
            driveStartToShoot,
            driveShootToIntakeReadyPoseSet2,
            driveIntakeReadyPoseSet2ToActuallyDoIntakeSet2,
            driveActuallyDoIntakeSet2toIntakeReadyPoseSet2,
            driveActuallyDoIntakeSet2ToReadyToEmpty,
            driveReadyToEmptyToEmptyGate,
            driveEmptyGateToGoToShootingLine,
            driveGoToShootingLineToShootPose,
            driveShootPosetoIntakeReadyPoseSet1,
            driveIntakeReadyPoseSet1toActuallyDoIntakeSet1,
            driveActuallyDoIntakeSet1toShootPose,
            driveShootPosetoIntakeReadyPoseSet3,
            driveIntakeReadyPoseSet3toActuallyDoIntakeSet3,
            driveActuallyDoIntakeSet3toShootPose,
            driveShootPosetoReadyToEmptyEnd;

    public void buildPaths() {
        driveStartToShoot = driveReadyPose(startPose, shootPose);
        driveShootToIntakeReadyPoseSet2 = driveReadyPose(shootPose, intakeReadyPoseSet2);
        driveIntakeReadyPoseSet2ToActuallyDoIntakeSet2 = driveReadyPose(intakeReadyPoseSet2, actuallyDoIntakeSet2);
        driveActuallyDoIntakeSet2toIntakeReadyPoseSet2 = driveReadyPose(actuallyDoIntakeSet2, intakeReadyPoseSet2);
        driveActuallyDoIntakeSet2ToReadyToEmpty = driveReadyPose(actuallyDoIntakeSet2, readyToEmpty);
        driveReadyToEmptyToEmptyGate = driveReadyPose(readyToEmpty, emptyGate);
        driveEmptyGateToGoToShootingLine = driveReadyPose(emptyGate, goToShootingLine);
        driveGoToShootingLineToShootPose = driveReadyPose(goToShootingLine, shootPose);
        driveShootPosetoIntakeReadyPoseSet1 = driveReadyPose(shootPose, intakeReadyPoseSet1);
        driveIntakeReadyPoseSet1toActuallyDoIntakeSet1 = driveReadyPose(intakeReadyPoseSet1, actuallyDoIntakeSet1);
        driveActuallyDoIntakeSet1toShootPose = driveReadyPose(actuallyDoIntakeSet1, shootPose);
        driveShootPosetoIntakeReadyPoseSet3 = driveReadyPose(shootPose, IntakeReadyPoseSet3);
        driveIntakeReadyPoseSet3toActuallyDoIntakeSet3 = driveReadyPose(IntakeReadyPoseSet3, actuallyDoIntakeSet3);
        driveActuallyDoIntakeSet3toShootPose = driveReadyPose(actuallyDoIntakeSet3, shootPose);
        driveShootPosetoReadyToEmptyEnd = driveReadyPose(shootPose, readyToEmptyEnd);
    }

    public PathChain driveReadyPose(Pose p1, Pose p2) {
        return follower.pathBuilder()
                .addPath(new BezierLine(p1, p2))
                .setLinearHeadingInterpolation(p1.getHeading(), p2.getHeading())
                .build();
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        if (pathTimer != null) pathTimer.resetTimer();
    }

    public void statePathUpdate() {
        switch (pathState) {

            case DRIVE_STARTPOS_TO_SHOOTING_POS:
                follower.followPath(driveStartToShoot, true);
                setPathState(PathState.PAUSE_FOR_SHOOT_1);
                break;

//            case PAUSE_2S_AFTER_STARTPOS_TO_SHOOTING_POS:
//                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
//                    setPathState(PathState.PAUSE_FOR_SHOOT_1);
//                break;

            case PAUSE_FOR_SHOOT_1:
                // runShooterPID();
                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
                    setPathState(PathState.DRIVE_SHOOTPOS_TO_INTAKE_READY_SET_2_POS);
                // Shooter.stopShooter();
                break;

            case DRIVE_SHOOTPOS_TO_INTAKE_READY_SET_2_POS:
                follower.followPath(driveShootToIntakeReadyPoseSet2, true);
                setPathState(PathState.DRIVE_INTAKE_READY_POSE_SET_2_TO_ACTUALLY_DO_INTAKE_SET_2);
                break;
//

            case DRIVE_INTAKE_READY_POSE_SET_2_TO_ACTUALLY_DO_INTAKE_SET_2:
                // Intake.runIntake();
                follower.followPath(driveIntakeReadyPoseSet2ToActuallyDoIntakeSet2, true);
                setPathState(PathState.DRIVEBACK_INTAKE_READY_SET_2_POS);
                break;

//
            case DRIVEBACK_INTAKE_READY_SET_2_POS:
                follower.followPath(driveActuallyDoIntakeSet2toIntakeReadyPoseSet2, true);
                setPathState(PathState.DRIVE_ACTUALLY_DO_INTAKE_SET_2_TO_READY_TO_EMPTY);
                break;
//            case PAUSE_2S_AFTER_DRIVEBACK_INTAKE_READY_SET_2_POS:
//                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
//                    setPathState(PathState.DRIVE_ACTUALLY_DO_INTAKE_SET_2_TO_READY_TO_EMPTY);
//                break;
//
            case DRIVE_ACTUALLY_DO_INTAKE_SET_2_TO_READY_TO_EMPTY:
                follower.followPath(driveActuallyDoIntakeSet2ToReadyToEmpty, true);
                setPathState(PathState.DRIVE_READY_TO_EMPTY_TO_EMPTY_GATE);
                break;

//            case PAUSE_2S_AFTER_ACTUALLY_DO_INTAKE_SET_2:
//                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
//                    setPathState(PathState.DRIVE_READY_TO_EMPTY_TO_EMPTY_GATE);
//                break;

            case DRIVE_READY_TO_EMPTY_TO_EMPTY_GATE:
                follower.followPath(driveReadyToEmptyToEmptyGate, true);
                setPathState(PathState.DRIVE_EMPTY_GATE_TO_GO_TO_SHOOTING_LINE);
                break;
//
//            case PAUSE_2S_AFTER_READY_TO_EMPTY:
//                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
//                    setPathState(PathState.DRIVE_EMPTY_GATE_TO_GO_TO_SHOOTING_LINE);
//                break;

            case DRIVE_EMPTY_GATE_TO_GO_TO_SHOOTING_LINE:
                follower.followPath(driveEmptyGateToGoToShootingLine, true);
                setPathState(PathState.DRIVE_GO_TO_SHOOTING_LINE_TO_SHOOT_POSE);
                break;

//            case PAUSE_2S_AFTER_EMPTY_GATE:
//                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
//                    setPathState(PathState.DRIVE_GO_TO_SHOOTING_LINE_TO_SHOOT_POSE);
//                break;

            case DRIVE_GO_TO_SHOOTING_LINE_TO_SHOOT_POSE:
                follower.followPath(driveGoToShootingLineToShootPose, true);
                setPathState(PathState.PAUSE_FOR_SHOOT_2);
                break;
//
//            case PAUSE_2S_AFTER_GO_TO_SHOOTING_LINE:
//                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
//                    setPathState(PathState.PAUSE_FOR_SHOOT_2);
//                break;

            case PAUSE_FOR_SHOOT_2:
                // runShooterPID();
                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
                    setPathState(PathState.DRIVE_SHOOT_POSE_TO_INTAKE_READY_POSE_SET_1);
                // Shooter.stopShooter();
                break;

            case DRIVE_SHOOT_POSE_TO_INTAKE_READY_POSE_SET_1:
                follower.followPath(driveShootPosetoIntakeReadyPoseSet1, true);
                setPathState(PathState.PAUSE_2S_AFTER_SHOOT_TO_INTAKE_READY_SET_1);
                break;

            case PAUSE_2S_AFTER_SHOOT_TO_INTAKE_READY_SET_1:
                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
                    setPathState(PathState.DRIVE_INTAKE_READY_POSE_SET_1_TO_ACTUALLY_DO_INTAKE_SET_1);
                break;

            case DRIVE_INTAKE_READY_POSE_SET_1_TO_ACTUALLY_DO_INTAKE_SET_1:
                // Intake.runIntake();
                follower.followPath(driveIntakeReadyPoseSet1toActuallyDoIntakeSet1, true);
                setPathState(PathState.PAUSE_2S_AFTER_INTAKE_READY_SET_1);
                break;

            case PAUSE_2S_AFTER_INTAKE_READY_SET_1:
                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
                    setPathState(PathState.DRIVE_ACTUALLY_DO_INTAKE_SET_1_TO_SHOOT_POSE);
                break;

            case DRIVE_ACTUALLY_DO_INTAKE_SET_1_TO_SHOOT_POSE:
                follower.followPath(driveActuallyDoIntakeSet1toShootPose, true);
                setPathState(PathState.PAUSE_2S_AFTER_ACTUALLY_DO_INTAKE_SET_1);
                break;
//
            case PAUSE_2S_AFTER_ACTUALLY_DO_INTAKE_SET_1:
                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
                    setPathState(PathState.PAUSE_FOR_SHOOT_3);
                break;

            case PAUSE_FOR_SHOOT_3:
                // runShooterPID();
                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
                    setPathState(PathState.DRIVE_SHOOT_POSE_TO_INTAKE_READY_POSE_SET_3);
                // Shooter.stopShooter();
                break;

            case DRIVE_SHOOT_POSE_TO_INTAKE_READY_POSE_SET_3:
                follower.followPath(driveShootPosetoIntakeReadyPoseSet3, true);
                setPathState(PathState.PAUSE_2S_AFTER_SHOOT_TO_INTAKE_READY_SET_3);
                break;

            case PAUSE_2S_AFTER_SHOOT_TO_INTAKE_READY_SET_3:
                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
                    setPathState(PathState.DRIVE_INTAKE_READY_POSE_SET_3_TO_ACTUALLY_DO_INTAKE_SET_3);
                break;

            case DRIVE_INTAKE_READY_POSE_SET_3_TO_ACTUALLY_DO_INTAKE_SET_3:
                // Intake.runIntake();
                follower.followPath(driveIntakeReadyPoseSet3toActuallyDoIntakeSet3, true);
                setPathState(PathState.PAUSE_2S_AFTER_INTAKE_READY_SET_3);
                break;

            case PAUSE_2S_AFTER_INTAKE_READY_SET_3:
                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
                    setPathState(PathState.DRIVE_ACTUALLY_DO_INTAKE_SET_3_TO_SHOOT_POSE);
                break;

            case DRIVE_ACTUALLY_DO_INTAKE_SET_3_TO_SHOOT_POSE:
                follower.followPath(driveActuallyDoIntakeSet3toShootPose, true);
                setPathState(PathState.PAUSE_2S_AFTER_ACTUALLY_DO_INTAKE_SET_3);
                break;

            case PAUSE_2S_AFTER_ACTUALLY_DO_INTAKE_SET_3:
                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
                    setPathState(PathState.PAUSE_FOR_SHOOT_4);
                break;

            case PAUSE_FOR_SHOOT_4:
                // runShooterPID();
                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
                    setPathState(PathState.DRIVE_SHOOT_POSE_TO_READY_TO_EMPTY_END);
                // Shooter.stopShooter();
                break;

            case DRIVE_SHOOT_POSE_TO_READY_TO_EMPTY_END:
                follower.followPath(driveShootPosetoReadyToEmptyEnd, true);
                setPathState(PathState.PAUSE_2S_AFTER_READY_TO_EMPTY_END);
                break;

            case PAUSE_2S_AFTER_READY_TO_EMPTY_END:
                break;
        }
    }

    @Override
    public void runOpMode() {

        Shooter.init(hardwareMap);

        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        buildPaths();
        pathState = PathState.DRIVE_STARTPOS_TO_SHOOTING_POS;

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            statePathUpdate();
            telemetry.addData("State", pathState);
            telemetry.addData("X Position", follower.getPose().getX());
            telemetry.addData("Y Position", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }
}
