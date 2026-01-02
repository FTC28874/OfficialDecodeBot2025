package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;


import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.Intake;

@Autonomous
public class autopathingfarfrombluegoal extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer;

    private GoBildaPinpointDriver pinpoint = null;

    public enum PathState {

        DRIVE_STARTPOS_TO_SHOOTING_POS,
        PAUSE_1,

        DRIVE_SHOOTPOS_TO_INTAKE_READY_SET_2_POS,
        PAUSE_2,

        DRIVE_INTAKE_READY_POSE_SET_2_TO_ACTUALLY_DO_INTAKE_SET_2,
        PAUSE_3,

        DRIVE_ACTUALLY_DO_INTAKE_SET_2_TO_READY_TO_EMPTY,
        PAUSE_4,

        DRIVE_READY_TO_EMPTY_TO_EMPTY_GATE,
        PAUSE_5,

        DRIVE_EMPTY_GATE_TO_GO_TO_SHOOTING_LINE,
        PAUSE_6,

        DRIVE_GO_TO_SHOOTING_LINE_TO_SHOOT_POSE,
        PAUSE_7,

        DRIVE_SHOOT_POSE_TO_INTAKE_READY_POSE_SET_1,
        PAUSE_8,

        DRIVE_INTAKE_READY_POSE_SET_1_TO_ACTUALLY_DO_INTAKE_SET_1,
        PAUSE_9,

        DRIVE_ACTUALLY_DO_INTAKE_SET_1_TO_SHOOT_POSE,
        PAUSE_10,

        DRIVE_SHOOT_POSE_TO_INTAKE_READY_POSE_SET_3,
        PAUSE_11,

        DRIVE_INTAKE_READY_POSE_SET_3_TO_ACTUALLY_DO_INTAKE_SET_3,
        PAUSE_12,

        DRIVE_ACTUALLY_DO_INTAKE_SET_3_TO_SHOOT_POSE,
        PAUSE_13,

        DRIVE_SHOOT_POSE_TO_READY_TO_EMPTY_END
    }

    private PathState pathState;

    private final Pose startPose = new Pose(56.8, 7.88, Math.toRadians(90));
    private final Pose shootPose = new Pose(56, 88, Math.toRadians(180));

    private final Pose intakeReadyPoseSet2 = new Pose(55, 54.8, Math.toRadians(180));
    private final Pose actuallyDoIntakeSet2 = new Pose(35, 54.8, Math.toRadians(180));

    private final Pose readyToEmpty = new Pose(22, 69.7, Math.toRadians(90));
    private final Pose emptyGate = new Pose(19, 69.7, Math.toRadians(90));
    private final Pose goToShootingLine = new Pose(55.8, 69.7, Math.toRadians(90));

    private final Pose intakeReadyPoseSet1 = new Pose(41.6, 81, Math.toRadians(180));
    private final Pose actuallyDoIntakeSet1 = new Pose(21.7, 84, Math.toRadians(180));

    private final Pose intakeReadyPoseSet3 = new Pose(41.6, 35.5, Math.toRadians(180));
    private final Pose actuallyDoIntakeSet3 = new Pose(21.7, 35.5, Math.toRadians(180));

    private final Pose readyToEmptyEnd = new Pose(35, 69.7, Math.toRadians(180));

    private PathChain
            driveStartToShoot,
            driveShootToIntakeReadyPoseSet2,
            driveIntakeReadyPoseSet2ToActuallyDoIntakeSet2,
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
        driveStartToShoot = drive(startPose, shootPose);
        driveShootToIntakeReadyPoseSet2 = drive(shootPose, intakeReadyPoseSet2);
        driveIntakeReadyPoseSet2ToActuallyDoIntakeSet2 = drive(intakeReadyPoseSet2, actuallyDoIntakeSet2);
        driveActuallyDoIntakeSet2ToReadyToEmpty = drive(actuallyDoIntakeSet2, readyToEmpty);
        driveReadyToEmptyToEmptyGate = drive(readyToEmpty, emptyGate);
        driveEmptyGateToGoToShootingLine = drive(emptyGate, goToShootingLine);
        driveGoToShootingLineToShootPose = drive(goToShootingLine, shootPose);
        driveShootPosetoIntakeReadyPoseSet1 = drive(shootPose, intakeReadyPoseSet1);
        driveIntakeReadyPoseSet1toActuallyDoIntakeSet1 = drive(intakeReadyPoseSet1, actuallyDoIntakeSet1);
        driveActuallyDoIntakeSet1toShootPose = drive(actuallyDoIntakeSet1, shootPose);
        driveShootPosetoIntakeReadyPoseSet3 = drive(shootPose, intakeReadyPoseSet3);
        driveIntakeReadyPoseSet3toActuallyDoIntakeSet3 = drive(intakeReadyPoseSet3, actuallyDoIntakeSet3);
        driveActuallyDoIntakeSet3toShootPose = drive(actuallyDoIntakeSet3, shootPose);
        driveShootPosetoReadyToEmptyEnd = drive(shootPose, readyToEmptyEnd);
    }

    private PathChain drive(Pose a, Pose b) {
        return follower.pathBuilder()
                .addPath(new BezierLine(a, b))
                .setLinearHeadingInterpolation(a.getHeading(), b.getHeading())
                .build();
    }

    private void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    public void statePathUpdate() {
        switch (pathState) {

            case DRIVE_STARTPOS_TO_SHOOTING_POS:
                follower.followPath(driveStartToShoot, true);
                Intake.raiseIntake();
                setPathState(PathState.PAUSE_1);
                break;

            case PAUSE_1:
                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
                    setPathState(PathState.DRIVE_SHOOTPOS_TO_INTAKE_READY_SET_2_POS);
                break;

            case DRIVE_SHOOTPOS_TO_INTAKE_READY_SET_2_POS:
                follower.followPath(driveShootToIntakeReadyPoseSet2, true);
                Intake.raiseIntake();
                Intake.runIntake();
                setPathState(PathState.PAUSE_2);
                break;

            case PAUSE_2:
                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
                    setPathState(PathState.DRIVE_INTAKE_READY_POSE_SET_2_TO_ACTUALLY_DO_INTAKE_SET_2);
                break;

            case DRIVE_INTAKE_READY_POSE_SET_2_TO_ACTUALLY_DO_INTAKE_SET_2:
                follower.followPath(driveIntakeReadyPoseSet2ToActuallyDoIntakeSet2, true);
                Intake.stopIntake();
                Intake.lowerIntake();
                setPathState(PathState.PAUSE_3);
                break;

            case PAUSE_3:
                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
                    setPathState(PathState.DRIVE_ACTUALLY_DO_INTAKE_SET_2_TO_READY_TO_EMPTY);
                break;

            case DRIVE_ACTUALLY_DO_INTAKE_SET_2_TO_READY_TO_EMPTY:
                follower.followPath(driveActuallyDoIntakeSet2ToReadyToEmpty, true);
                setPathState(PathState.PAUSE_4);
                break;

            case PAUSE_4:
                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
                    setPathState(PathState.DRIVE_READY_TO_EMPTY_TO_EMPTY_GATE);
                break;

            case DRIVE_READY_TO_EMPTY_TO_EMPTY_GATE:
                follower.followPath(driveReadyToEmptyToEmptyGate, true);
                setPathState(PathState.PAUSE_5);
                break;

            case PAUSE_5:
                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
                    setPathState(PathState.DRIVE_EMPTY_GATE_TO_GO_TO_SHOOTING_LINE);
                break;

            case DRIVE_EMPTY_GATE_TO_GO_TO_SHOOTING_LINE:
                follower.followPath(driveEmptyGateToGoToShootingLine, true);
                setPathState(PathState.PAUSE_6);
                break;

            case PAUSE_6:
                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
                    setPathState(PathState.DRIVE_GO_TO_SHOOTING_LINE_TO_SHOOT_POSE);
                break;

            case DRIVE_GO_TO_SHOOTING_LINE_TO_SHOOT_POSE:
                follower.followPath(driveGoToShootingLineToShootPose, true);
                setPathState(PathState.PAUSE_7);
                break;

            case PAUSE_7:
                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
                    setPathState(PathState.DRIVE_SHOOT_POSE_TO_INTAKE_READY_POSE_SET_1);
                break;

            case DRIVE_SHOOT_POSE_TO_INTAKE_READY_POSE_SET_1:
                follower.followPath(driveShootPosetoIntakeReadyPoseSet1, true);
                setPathState(PathState.PAUSE_8);
                break;
//
            case PAUSE_8:
                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
                    setPathState(PathState.DRIVE_INTAKE_READY_POSE_SET_1_TO_ACTUALLY_DO_INTAKE_SET_1);
                break;

            case DRIVE_INTAKE_READY_POSE_SET_1_TO_ACTUALLY_DO_INTAKE_SET_1:
                follower.followPath(driveIntakeReadyPoseSet1toActuallyDoIntakeSet1, true);
                setPathState(PathState.PAUSE_9);
                break;
//
            case PAUSE_9:
                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
                    setPathState(PathState.DRIVE_ACTUALLY_DO_INTAKE_SET_1_TO_SHOOT_POSE);
                break;

            case DRIVE_ACTUALLY_DO_INTAKE_SET_1_TO_SHOOT_POSE:
                follower.followPath(driveActuallyDoIntakeSet1toShootPose, true);
                setPathState(PathState.PAUSE_10);
                break;
//
//            case PAUSE_10:
//                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
//                    setPathState(PathState.DRIVE_SHOOT_POSE_TO_INTAKE_READY_POSE_SET_3);
//                break;
//
//            case DRIVE_SHOOT_POSE_TO_INTAKE_READY_POSE_SET_3:
//                follower.followPath(driveShootPosetoIntakeReadyPoseSet3, true);
//                setPathState(PathState.PAUSE_11);
//                break;
//
//            case PAUSE_11:
//                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
//                    setPathState(PathState.DRIVE_INTAKE_READY_POSE_SET_3_TO_ACTUALLY_DO_INTAKE_SET_3);
//                break;
//
//            case DRIVE_INTAKE_READY_POSE_SET_3_TO_ACTUALLY_DO_INTAKE_SET_3:
//                follower.followPath(driveIntakeReadyPoseSet3toActuallyDoIntakeSet3, true);
//                setPathState(PathState.PAUSE_12);
//                break;
//
//            case PAUSE_12:
//                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
//                    setPathState(PathState.DRIVE_ACTUALLY_DO_INTAKE_SET_3_TO_SHOOT_POSE);
//                break;
//
//            case DRIVE_ACTUALLY_DO_INTAKE_SET_3_TO_SHOOT_POSE:
//                follower.followPath(driveActuallyDoIntakeSet3toShootPose, true);
//                setPathState(PathState.PAUSE_13);
//                break;
//
//            case PAUSE_13:
//                if (pathTimer.getElapsedTimeSeconds() >= 2.0)
//                    setPathState(PathState.DRIVE_SHOOT_POSE_TO_READY_TO_EMPTY_END);
//                break;
//
//            case DRIVE_SHOOT_POSE_TO_READY_TO_EMPTY_END:
//                follower.followPath(driveShootPosetoReadyToEmptyEnd, true);
//                break;
        }
    }

    @Override
    public void runOpMode() {

        Shooter.init(hardwareMap);
        Intake.init(hardwareMap);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pathTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        buildPaths();
        pathState = PathState.DRIVE_STARTPOS_TO_SHOOTING_POS;

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("x:", follower.getPose().getX());
            telemetry.addData("y: ", follower.getPose().getY());
            telemetry.update();

            follower.update();
            statePathUpdate();
        }
    }
}
