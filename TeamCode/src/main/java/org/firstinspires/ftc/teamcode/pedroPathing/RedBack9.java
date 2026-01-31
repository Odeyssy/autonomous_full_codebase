package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Red BACK 9 Artifact (WORKING)", group = "Autonomous")
@Configurable
public class RedBack9 extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;

    // Systems
    private IntakeSystem intake;
    private LimelightAligner limelightAligner;
    private ShooterSystem shooter;
    private ElapsedTime stateTimer = new ElapsedTime();

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.6);

        intake = new IntakeSystem(hardwareMap);
        limelightAligner = new LimelightAligner(hardwareMap);
        shooter = new ShooterSystem(hardwareMap);
        follower.setStartingPose(new Pose(88.0, 8.0, Math.toRadians(90)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Busy", follower.isBusy());
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // START TURN ONLY (90° → 60°)
                shooter.startFlywheels();
                follower.followPath(paths.Path0, true);
                pathState = 1;
                break;

            case 1:
                // WAIT FOR TURN TO FINISH
                if (!follower.isBusy()) {
                    shooter.shoot();
                    stateTimer.reset();
                    pathState = 2;
                }
                break;

            case 2:
                if (stateTimer.seconds() > 5.0) {
                    shooter.stopFeeding();
                    intake.runIntake();
                    follower.followPath(paths.Path1, true);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2, true);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, true);
                    pathState = 52;
                }
                break;

            case 52:
                if (!follower.isBusy()) {
                    follower.setTeleOpDrive(0, 0, 0);
                    stateTimer.reset();
                    pathState = 53;
                }
                break;

            case 53:
                shooter.shoot();
                if (stateTimer.seconds() > 4.0) {
                    shooter.stopFeeding();
                    follower.followPath(paths.Path4, true);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5, true);
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6, true);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    intake.stopAll();
                    shooter.stopAll();
                    pathState = 8;
                }
                break;

            case 8:
                follower.setTeleOpDrive(0, 0, 0);
                break;
        }
    }

    public static class Paths {

        public PathChain Path0, Path1, Path2, Path3, Path4, Path5, Path6;

        public Paths(Follower follower) {
            Path0 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88.0, 8.0),
                            new Pose(88.0, 14)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(65)
                    )
                    .build();

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88.0, 14),
                            new Pose(88.0, 34.939)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(65),
                            Math.toRadians(180)
                    )
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88.0, 34.939),
                            new Pose(135.448, 34.939)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(180)
                    )
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(135.448, 34.939),
                            new Pose(88.0, 8.0)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(60)
                    )
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88.0, 8.0),
                            new Pose(134.746, 15.0)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(60),
                            Math.toRadians(180)
                    )
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(134.746, 15.0),
                            new Pose(88.0, 8.0)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(75)
                    )
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88.0, 8.0),
                            new Pose(109.613, 12.0)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(75),
                            Math.toRadians(180)
                    )
                    .build();
        }
    }
}