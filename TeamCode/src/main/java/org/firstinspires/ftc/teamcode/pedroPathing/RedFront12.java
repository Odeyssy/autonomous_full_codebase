package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Red FRONT 12 Artifact (TESTING)", group = "Autonomous")
@Configurable
public class RedFront12 extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;
    private ElapsedTime stateTimer = new ElapsedTime();

    // Systems
    private IntakeSystem intake;
    private LimelightAligner limelightAligner;
    private ShooterSystem shooter;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.5);

        // Starting pose: (123, 123) facing 217 degrees to match ShootPreloaded start
        follower.setStartingPose(new Pose(123.000, 123.000, Math.toRadians(217)));

        // Initialize systems
        intake = new IntakeSystem(hardwareMap);
        limelightAligner = new LimelightAligner(hardwareMap);
        shooter = new ShooterSystem(hardwareMap);
        shooter.startFlywheels();

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Busy", follower.isBusy());
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.ShootPreloaded, true);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    shooter.shoot();      // shoot preloaded
                    stateTimer.reset();
                    pathState = 12;       // new state for timing shooting
                }
                break;
            case 12:
                if (stateTimer.seconds() > 4.0) {  // allow shooting for 4 seconds
                    shooter.stopFeeding();
                    follower.followPath(paths.ToFirst3, true);
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.CollectFirst3, true);
                    intake.runIntake();   // start intake when collecting
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    intake.stopAll();
                    shooter.shoot();      // shoot first 3
                    stateTimer.reset();
                    pathState = 13;       // new state for timing shooting
                }
                break;
            case 13:
                if (stateTimer.seconds() > 4.0) {
                    shooter.stopFeeding();
                    follower.followPath(paths.ToSecond3, true);
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.CollectSecond3, true);
                    intake.runIntake();
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    intake.stopAll();
                    shooter.shoot();      // shoot second 3
                    stateTimer.reset();
                    pathState = 14;       // new state for timing shooting
                }
                break;
            case 14:
                if (stateTimer.seconds() > 4.0) {
                    shooter.stopFeeding();
                    follower.followPath(paths.ToThird3, true);
                    pathState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.CollectThird3, true);
                    intake.runIntake();
                    pathState = 7;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    intake.stopAll();
                    shooter.shoot();      // shoot third 3
                    stateTimer.reset();
                    pathState = 15;
                }
                break;
            case 15:
                if (stateTimer.seconds() > 4.0) {
                    shooter.stopFeeding();
                    follower.followPath(paths.LeavePoints, true);
                    pathState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.setTeleOpDrive(0, 0, 0);
                    pathState = 9; // finished
                }
                break;
        }
    }

    public static class Paths {
        public PathChain ShootPreloaded, ToFirst3, CollectFirst3, ShootFirst3,
                ToSecond3, CollectSecond3, ShootSecond3,
                ToThird3, CollectThird3, ShootThird3, LeavePoints;

        public Paths(Follower follower) {
            ShootPreloaded = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(123.000, 123.000), new Pose(82.000, 81.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(217), Math.toRadians(45)).build();

            ToFirst3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(82.000, 81.000), new Pose(109.835, 83.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180)).build();

            CollectFirst3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(109.835, 83.000), new Pose(128.000, 83.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

            ShootFirst3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(128.000, 83.000), new Pose(82.000, 81.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45)).build();

            ToSecond3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(82.000, 81.000), new Pose(109.304, 59.177)))
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180)).build();

            CollectSecond3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(109.304, 59.177), new Pose(130.647, 58.912)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

            ShootSecond3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(130.647, 58.912), new Pose(82.000, 81.265)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45)).build();

            ToThird3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(82.000, 81.265), new Pose(109.039, 35.205)))
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180)).build();

            CollectThird3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(109.039, 35.205), new Pose(130.912, 34.674)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

            ShootThird3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(130.912, 34.674), new Pose(88.000, 8.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(72)).build();

            LeavePoints = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(88.000, 8.000), new Pose(108.923, 10.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(72), Math.toRadians(180)).build();
        }
    }
}
