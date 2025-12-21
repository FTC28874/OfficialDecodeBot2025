package org.firstinspires.ftc.teamcode.auto.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Shooter;

@Autonomous
public class autopathing extends OpMode {
    private double shooterEncSpeed = 1600;
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {

        DRIVE_STARTPOS_TO_SHOOTING_POS,
        PAUSE_FLYWHEEL_1,

        DRIVE_SHOOTPOS_TO_INTAKE_READY_SET_2_POS,
        DRIVE_INTAKE_READY_POSE_SET_2_TO_ACTUALLY_DO_INTAKE_SET_2,
        DRIVE_ACTUALLY_DO_INTAKE_SET_2_TO_READY_TO_EMPTY,
        DRIVE_READY_TO_EMPTY_TO_EMPTY_GATE,
        DRIVE_EMPTY_GATE_TO_GO_TO_SHOOTING_LINE,
        DRIVE_GO_TO_SHOOTING_LINE_TO_SHOOT_POSE,
        PAUSE_FLYWHEEL_2,

        DRIVE_SHOOT_POSE_TO_INTAKE_READY_POSE_SET_1,
        DRIVE_INTAKE_READY_POSE_SET_1_TO_ACTUALLY_DO_INTAKE_SET_1,
        DRIVE_ACTUALLY_DO_INTAKE_SET_1_TO_SHOOT_POSE,
        PAUSE_FLYWHEEL_3,

        DRIVE_SHOOT_POSE_TO_INTAKE_READY_POSE_SET_3,
        DRIVE_INTAKE_READY_POSE_SET_3_TO_ACTUALLY_DO_INTAKE_SET_3,
        DRIVE_ACTUALLY_DO_INTAKE_SET_3_TO_SHOOT_POSE,
        PAUSE_FLYWHEEL_4,

        DRIVE_SHOOT_POSE_TO_READY_TO_EMPTY_END
    }

    PathState pathState;

    private final Pose startPose = new Pose(56.8, 7.88571428571429, Math.toRadians(90));
    private final Pose shootPose = new Pose(56, 88, Math.toRadians(180));
    private final Pose intakeReadyPoseSet2 = new Pose(41.6, 59.8, Math.toRadians(180));
    private final Pose actuallyDoIntakeSet2 = new Pose(21.7, 59.8, Math.toRadians(180));
    private final Pose readyToEmpty = new Pose(22, 69.7, Math.toRadians(90));
    private final Pose emptyGate = new Pose(15.8, 69.7, Math.toRadians(90));
    private final Pose goToShootingLine = new Pose(55.8, 69.7, Math.toRadians(90));

    private final Pose intakeReadyPoseSet1 = new Pose(41.6, 84, Math.toRadians(180));
    private final Pose actuallyDoIntakeSet1 = new Pose(21.7, 84, Math.toRadians(180));

    private final Pose IntakeReadyPoseSet3 = new Pose(41.6, 35.5, Math.toRadians(180));
    private final Pose actuallyDoIntakeSet3 = new Pose(21.7, 35.5, Math.toRadians(180));

    private final Pose readyToEmptyEnd = new Pose(35, 69.7, Math.toRadians(180));

    private PathChain driveStartToShoot,
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

    public PathChain pathChain = new PathChain();

    public void buildPaths() {

        driveStartToShoot = driveReadyPose(startPose, shootPose);
        driveShootToIntakeReadyPoseSet2 = driveReadyPose(shootPose, intakeReadyPoseSet2);
        driveIntakeReadyPoseSet2ToActuallyDoIntakeSet2 = driveReadyPose(intakeReadyPoseSet2, actuallyDoIntakeSet2);
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

    public PathChain driveReadyPose(Pose poseObj1, Pose poseObj2) {

        pathChain = follower.pathBuilder()
                .addPath(new BezierLine(poseObj1, poseObj2))
                .setLinearHeadingInterpolation(poseObj1.getHeading(), poseObj2.getHeading())
                .build();

        return pathChain;
    }

    public void statePathUpdate() {
        switch (pathState) {

            case DRIVE_STARTPOS_TO_SHOOTING_POS:
                follower.followPath(driveStartToShoot, true);
                setPathState(PathState.PAUSE_FLYWHEEL_1);
                break;

            case PAUSE_FLYWHEEL_1:
                // TODO add flywheel logic
                if (pathTimer.getElapsedTimeSeconds() >= 2.0) {
                    setPathState(PathState.DRIVE_SHOOTPOS_TO_INTAKE_READY_SET_2_POS);
                }
                break;

            case DRIVE_SHOOTPOS_TO_INTAKE_READY_SET_2_POS:
                follower.followPath(driveShootToIntakeReadyPoseSet2, true);
                setPathState(PathState.DRIVE_INTAKE_READY_POSE_SET_2_TO_ACTUALLY_DO_INTAKE_SET_2);
                break;

            case DRIVE_INTAKE_READY_POSE_SET_2_TO_ACTUALLY_DO_INTAKE_SET_2:
                follower.followPath(driveIntakeReadyPoseSet2ToActuallyDoIntakeSet2, true);
                setPathState(PathState.DRIVE_ACTUALLY_DO_INTAKE_SET_2_TO_READY_TO_EMPTY);
                break;

            case DRIVE_ACTUALLY_DO_INTAKE_SET_2_TO_READY_TO_EMPTY:
                follower.followPath(driveActuallyDoIntakeSet2ToReadyToEmpty, true);
                setPathState(PathState.DRIVE_READY_TO_EMPTY_TO_EMPTY_GATE);
                break;

            case DRIVE_READY_TO_EMPTY_TO_EMPTY_GATE:
                follower.followPath(driveReadyToEmptyToEmptyGate, true);
                setPathState(PathState.DRIVE_EMPTY_GATE_TO_GO_TO_SHOOTING_LINE);
                break;

            case DRIVE_EMPTY_GATE_TO_GO_TO_SHOOTING_LINE:
                follower.followPath(driveEmptyGateToGoToShootingLine, true);
                setPathState(PathState.DRIVE_GO_TO_SHOOTING_LINE_TO_SHOOT_POSE);

            case DRIVE_GO_TO_SHOOTING_LINE_TO_SHOOT_POSE:
                follower.followPath(driveGoToShootingLineToShootPose, true);
                setPathState(PathState.PAUSE_FLYWHEEL_2);
                break;

            case PAUSE_FLYWHEEL_2:

                // TODO add flywheel logic
                if (pathTimer.getElapsedTimeSeconds() >= 2.0) {
                    setPathState(PathState.DRIVE_SHOOT_POSE_TO_INTAKE_READY_POSE_SET_1);
                }
                break;

            case DRIVE_SHOOT_POSE_TO_INTAKE_READY_POSE_SET_1:
                follower.followPath(driveShootPosetoIntakeReadyPoseSet1, true);
                setPathState(PathState.DRIVE_INTAKE_READY_POSE_SET_1_TO_ACTUALLY_DO_INTAKE_SET_1);
                break;

            case DRIVE_INTAKE_READY_POSE_SET_1_TO_ACTUALLY_DO_INTAKE_SET_1:
                follower.followPath(driveIntakeReadyPoseSet1toActuallyDoIntakeSet1, true);
                setPathState(PathState.DRIVE_ACTUALLY_DO_INTAKE_SET_1_TO_SHOOT_POSE);
                break;

            case DRIVE_ACTUALLY_DO_INTAKE_SET_1_TO_SHOOT_POSE:
                follower.followPath(driveActuallyDoIntakeSet1toShootPose, true);
                setPathState(PathState.PAUSE_FLYWHEEL_3);
                break;

            case PAUSE_FLYWHEEL_3:

                // TODO add flywheel logic

                if (pathTimer.getElapsedTimeSeconds() >= 2.0) {
                    setPathState(PathState.DRIVE_EMPTY_GATE_TO_GO_TO_SHOOTING_LINE);
                }
                break;

            case DRIVE_SHOOT_POSE_TO_INTAKE_READY_POSE_SET_3:
                follower.followPath(driveShootPosetoIntakeReadyPoseSet3, true);
                setPathState(PathState.DRIVE_EMPTY_GATE_TO_GO_TO_SHOOTING_LINE);
                break;

            case DRIVE_INTAKE_READY_POSE_SET_3_TO_ACTUALLY_DO_INTAKE_SET_3:
                follower.followPath(driveIntakeReadyPoseSet3toActuallyDoIntakeSet3, true);
                setPathState(PathState.DRIVE_EMPTY_GATE_TO_GO_TO_SHOOTING_LINE);
                break;

            case DRIVE_ACTUALLY_DO_INTAKE_SET_3_TO_SHOOT_POSE:
                follower.followPath(driveActuallyDoIntakeSet3toShootPose, true);
                setPathState(PathState.PAUSE_FLYWHEEL_4);
                break;

            case PAUSE_FLYWHEEL_4:

                // TODO add flywheel logic
                if (pathTimer.getElapsedTimeSeconds() >= 2.0) {
                    setPathState(PathState.DRIVE_EMPTY_GATE_TO_GO_TO_SHOOTING_LINE);
                }
                break;
            case DRIVE_SHOOT_POSE_TO_READY_TO_EMPTY_END:
                follower.followPath(driveShootPosetoReadyToEmptyEnd, true);
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_TO_SHOOTING_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path time", pathTimer.getElapsedTimeSeconds());
    }
}
