package org.firstinspires.ftc.teamcode.PedroPathing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;

@Autonomous(name = "pedro far from blue goal", group = "Autonomous")
@Configurable
public class NewPedroFarFromBlueGoal extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private Timer pathTimer;
    private Timer opmodeTimer;

    private static final int INTAKE_SET_2 = 0;
    private static final int TO_SHOOT_POSITION_SET_2 = 1;
    private static final int SHOOT_SET_2 = 2;
    private static final int INTAKE_SET_3 = 3;
    private static final int TO_EMPTY_GATE = 4;
    private static final int THROUGH_EMPTY_GATE = 5;
    private static final int TO_SHOOT_POSITION_SET_3 = 6;
    private static final int SHOOT_SET_3 = 7;
    private static final int INTAKE_SET_1 = 8;
    private static final int TO_SHOOT_POSITION_SET_1 = 9;
    private static final int SHOOT_SET_1 = 10;
    private static final int IDLE = 11;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(INTAKE_SET_2);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Is Busy", follower.isBusy());
        panelsTelemetry.update(telemetry);
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case INTAKE_SET_2:
                follower.followPath(paths.intakeset2, true);
                setPathState(TO_SHOOT_POSITION_SET_2);
                break;

            case TO_SHOOT_POSITION_SET_2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shootpositionset2, true);
                    setPathState(SHOOT_SET_2);
                }
                break;

            case SHOOT_SET_2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.intakeset3, true);
                    setPathState(INTAKE_SET_3);
                }
                break;

            case INTAKE_SET_3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.emptygate, true);
                    setPathState(TO_EMPTY_GATE);
                }
                break;

            case TO_EMPTY_GATE:
                if (!follower.isBusy()) {
                    setPathState(THROUGH_EMPTY_GATE);
                }
                break;

            case THROUGH_EMPTY_GATE:
                follower.followPath(paths.shootpositionset3, true);
                setPathState(TO_SHOOT_POSITION_SET_3);
                break;

            case TO_SHOOT_POSITION_SET_3:
                if (!follower.isBusy()) {
                    setPathState(SHOOT_SET_3);
                }
                break;

            case SHOOT_SET_3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.intakeset1, true);
                    setPathState(INTAKE_SET_1);
                }
                break;

            case INTAKE_SET_1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shootpositionset1, true);
                    setPathState(TO_SHOOT_POSITION_SET_1);
                }
                break;

            case TO_SHOOT_POSITION_SET_1:
                if (!follower.isBusy()) {
                    setPathState(SHOOT_SET_1);
                }
                break;

            case SHOOT_SET_1:
                if (!follower.isBusy()) {
                    setPathState(IDLE);
                }
                break;

            case IDLE:
                break;

            default:
                break;
        }
    }

    public static class Paths {
        public PathChain intakeset2;
        public PathChain shootpositionset2;
        public PathChain intakeset3;
        public PathChain emptygate;
        public PathChain shootpositionset3;
        public PathChain intakeset1;
        public PathChain shootpositionset1;

        public Paths(Follower follower) {
            intakeset2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.260, 8.800),
                                    new Pose(61.890, 62.885),
                                    new Pose(16.700, 59.790)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            shootpositionset2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(16.700, 59.790),
                                    new Pose(62.073, 63.121),
                                    new Pose(56.110, 7.380)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();

            intakeset3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.110, 7.380),
                                    new Pose(67.453, 92.026),
                                    new Pose(16.100, 84.230)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            emptygate = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(16.100, 84.230),
                                    new Pose(35.488, 66.280),
                                    new Pose(15.380, 69.850)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shootpositionset3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(15.380, 69.850),
                                    new Pose(48.268, 65.972),
                                    new Pose(56.010, 7.550)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();

            intakeset1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.010, 7.550),
                                    new Pose(46.480, 40.181),
                                    new Pose(15.800, 35.590)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            shootpositionset1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(15.800, 35.590),
                                    new Pose(56.180, 7.240)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();
        }
    }
}