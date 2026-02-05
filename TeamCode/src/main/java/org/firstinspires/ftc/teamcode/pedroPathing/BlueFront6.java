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

@Autonomous(name = "Blue FRONT 6 Artifact (TESTING)", group = "Autonomous")
@Configurable
public class BlueFront6 extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;

    private IntakeSystem intake;
    private LimelightAligner limelightAligner;
    private ShooterSystem shooter;
    private ElapsedTime stateTimer = new ElapsedTime();

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.7);

        intake = new IntakeSystem(hardwareMap);
        limelightAligner = new LimelightAligner(hardwareMap);
        shooter = new ShooterSystem(hardwareMap);
        //shooter.frontShooting();

        // Robot starts at (72, 8)
        follower.setStartingPose(new Pose(21, 123, Math.toRadians(142)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Busy", follower.isBusy());
        panelsTelemetry.update(telemetry);
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooter.startFlywheels();
                follower.followPath(paths.ShootPreloaded, true);
                pathState = 66;
                break;

            case 66: //shoot preloaded
                // WAIT FOR TURN TO FINISH
                if (!follower.isBusy()) {
                    shooter.shoot();
                    stateTimer.reset();
                    pathState = 23;
                }
                break;

            case 23:
                if (stateTimer.seconds() > 5.0) {
                    shooter.stopFeeding();
                    intake.runIntake();
                    follower.followPath(paths.FirstIntake, true);
                    pathState = 1;
                }
                break;   //stop shooting

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.CollectIntake, true);
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ShootFirstIntake, true);
                    pathState = 85;
                }
                break;

            case 85: //shoot preloaded
                // WAIT FOR TURN TO FINISH
                if (!follower.isBusy()) {
                    shooter.shoot();
                    stateTimer.reset();
                    pathState = 36;
                }
                break;

            case 36:
                if (stateTimer.seconds() > 5.0) {
                    shooter.stopFeeding();
                    intake.runIntake();
                    follower.followPath(paths.SecondIntake, true);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.CollectSecondIntake, true);
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.LeavePoints, true);
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
        return pathState;
    }

    public static class Paths {
        public PathChain ShootPreloaded, FirstIntake, CollectIntake, ShootFirstIntake, SecondIntake, CollectSecondIntake, LeavePoints;

        public Paths(Follower follower) {
            // FIXED Path 1: Move from start (72, 8) to (56, 8) so it doesn't get stuck
            ShootPreloaded = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(21, 123), new Pose(72, 73)))
                    .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(142))
                    .build();

            FirstIntake = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(72, 73), new Pose(72, 73.001)))
                    .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(0))
                    .build();

            CollectIntake = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(72, 73.001), new Pose(16, 73)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            ShootFirstIntake = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(16, 73), new Pose(69, 73)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(142))
                    .build();

            SecondIntake = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(69, 73), new Pose(69, 60)))
                    .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(0))
                    .build();

            CollectSecondIntake = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(69, 60), new Pose(17, 60)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            LeavePoints = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(17, 60), new Pose(18, 97)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

        }
    }
}