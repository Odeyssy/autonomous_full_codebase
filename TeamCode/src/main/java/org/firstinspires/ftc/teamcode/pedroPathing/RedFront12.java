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

@Autonomous(name = "Red FRONT 6 Artifact (DIRECT DRIVE)", group = "Autonomous")
@Configurable
public class RedFront12 extends OpMode {
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

        // Target RPM for Front Side
        shooter.setTargetVelocity(1550.0);

        // Mirrored Starting Pose: (123, 123) at 38 degrees
        follower.setStartingPose(new Pose(123, 123, Math.toRadians(38)));
        paths = new Paths(follower);

        telemetry.addData("Status", "Red Side Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Disable Pedro PID only during Limelight alignment states
        if (pathState != 66 && pathState != 85) {
            follower.update();
        }

        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooter.startFlywheels();
                follower.followPath(paths.ShootPreloaded, true);
                pathState = 100;
                break;

            case 100: // Wait for drive, then break for Limelight
                if (!follower.isBusy()) {
                    follower.setTeleOpDrive(0,0,0,false);
                    stateTimer.reset();
                    pathState = 66;
                }
                break;

            case 66: // Align Preload
                int alignStatus = limelightAligner.driveToTarget();

                telemetry.addLine("--- RED ALIGN PRELOAD ---");
                telemetry.addData("Status", (alignStatus == 1) ? "LOCKED" : (alignStatus == 0) ? "MOVING" : "SEARCHING");
                telemetry.addData("Time", "%.2f", stateTimer.seconds());

                // 1. If we are aligned, we move to shoot immediately.
                if (alignStatus == 1) {
                    limelightAligner.stopMotors();
                    stateTimer.reset();
                    pathState = 230;
                }
                // 2. Hard timeout: If we haven't aligned in 3 seconds, shoot anyway.
                else if (stateTimer.seconds() > 3.0) {
                    limelightAligner.stopMotors();
                    stateTimer.reset();
                    pathState = 230;
                }
                // 3. Persistent Search: Even if alignStatus is -1 (Target Lost),
                // we STAY in this state until the 3.0s timeout hits.
                // Do NOT add an "else if (alignStatus == -1)" jump here.
                break;

            case 230:
                if (shooter.isAtVelocity()) {
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
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.CollectIntake, true);
                    pathState = 11;
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    pathState = 110;
                }
                break;

            case 110:
                if (stateTimer.seconds() > 0.5) {
                    follower.followPath(paths.ShootFirstIntake, true);
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.setTeleOpDrive(0,0,0,false);
                    stateTimer.reset();
                    pathState = 85;
                }
                break;

            case 85: // Align Volley 2
                int alignStatus2 = limelightAligner.driveToTarget();

                telemetry.addLine("--- RED ALIGN VOLLEY 2 ---");
                telemetry.addData("Status", (alignStatus2 == 1) ? "LOCKED" : (alignStatus2 == 0) ? "MOVING" : "SEARCHING");

                // 1. Check for alignment success
                if (alignStatus2 == 1) {
                    limelightAligner.stopMotors();
                    stateTimer.reset();
                    pathState = 850;
                }
                // 2. Hard timeout for Volley 2
                else if (stateTimer.seconds() > 5.0) {
                    limelightAligner.stopMotors();
                    stateTimer.reset();
                    pathState = 850;
                }
                break;

            case 850:
                if (shooter.isAtVelocity()) {
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
                    pathState = 33;
                }
                break;

            case 33:
                if (!follower.isBusy()) {
                    follower.followPath(paths.LeavePoints, true);
                    pathState = 4;
                }
                break;

            case 4:
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
        telemetry.update();
    }

    public static class Paths {
        public PathChain ShootPreloaded, FirstIntake, CollectIntake, ShootFirstIntake, SecondIntake, CollectSecondIntake, LeavePoints;

        public Paths(Follower follower) {
            ShootPreloaded = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(123, 123), new Pose(72, 73)))
                    .setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(38))
                    .build();

            FirstIntake = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(72, 73), new Pose(72, 73.001)))
                    .setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(180))
                    .build();

            CollectIntake = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(72, 73.001), new Pose(122, 73)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            ShootFirstIntake = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(122, 73), new Pose(75, 73)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(38))
                    .build();

            SecondIntake = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(75, 73), new Pose(75, 60)))
                    .setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(180))
                    .build();

            CollectSecondIntake = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(75, 60), new Pose(119, 60)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            LeavePoints = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(119, 60), new Pose(25, 25)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }
}