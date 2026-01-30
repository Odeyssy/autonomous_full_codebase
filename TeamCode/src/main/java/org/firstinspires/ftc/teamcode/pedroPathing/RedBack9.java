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
    // Timer declaration - MUST BE AT CLASS LEVEL
    private ElapsedTime stateTimer = new ElapsedTime();

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.5);

        // Initialize Systems
        intake = new IntakeSystem(hardwareMap);
        limelightAligner = new LimelightAligner(hardwareMap); // Ensure this matches your constructor
        shooter = new ShooterSystem(hardwareMap);
        shooter.startFlywheels();
        // Robot starts at (88, 8) facing North (90 degrees)
        follower.setStartingPose(new Pose(88.000, 8.000, Math.toRadians(90)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Debugging Info
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("State Time", stateTimer.seconds());
        panelsTelemetry.debug("Intake Velocity", intake.getIntakeVelocity());
        panelsTelemetry.debug("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // INITIAL SHOOTING: Spin flywheels and trigger the feeding mechanism immediately
                shooter.shoot();
                pathState = 100; // Transition to initial shooting timer
                stateTimer.reset();
                break;

            case 100:
                // Run the initial shoot for 2 seconds to clear the pre-loads
                if (stateTimer.seconds() > 5.0) {
                    shooter.stopFeeding();
                    // Now start the actual movement
                    intake.runIntake();
                    follower.followPath(paths.Path1, true);
                    pathState = 1;
                    stateTimer.reset();
                }
                break;

            case 1:
                // Wait for Path 1 to finish
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2, true);
                    pathState = 2;
                    stateTimer.reset();
                }
                break;

            case 2:
                // Move to Second Shooting Position (Path 3)
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, true);
                    pathState = 52;
                    stateTimer.reset();
                }
                break;

            case 52:
                // BRAKE & WAIT for Path 3
                if (!follower.isBusy()) {
                    follower.setTeleOpDrive(0, 0, 0);
                    follower.update();
                    stateTimer.reset();
                    pathState = 53;
                }
                break;

            case 53:
                // SECOND STATIONARY SHOOTING
                shooter.shoot();

                if (stateTimer.seconds() > 4.0) {
                    shooter.stopFeeding();
                    pathState = 3;
                    stateTimer.reset();
                }
                break;

            case 3:
                // Resume Path 4
                follower.followPath(paths.Path4, true);
                pathState = 4;
                stateTimer.reset();
                break;

            case 4:
                if (stateTimer.seconds() > 0.2 && !follower.isBusy()) {
                    follower.followPath(paths.Path5, true);
                    pathState = 5;
                    stateTimer.reset();
                }
                break;

            case 5:
                if (stateTimer.seconds() > 0.2 && !follower.isBusy()) {
                    follower.followPath(paths.Path6, true);
                    pathState = 6;
                    stateTimer.reset();
                }
                break;

            case 6:
                if (stateTimer.seconds() > 0.2 && !follower.isBusy()) {
                    intake.stopAll();
                    pathState = 7;
                    stateTimer.reset();
                }
                break;

            case 7:
                // Terminal State
                follower.setTeleOpDrive(0,0,0);
                intake.stopAll();
                shooter.stopAll();
                break;
        }
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(88.000, 8.000), new Pose(88.000, 34.939)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180)).build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(88.000, 34.939), new Pose(135.448, 34.939)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(135.448, 34.939), new Pose(88.000, 8.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(60)).build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(88.000, 8.000), new Pose(134.746, 12.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(180)).build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(134.746, 12.000), new Pose(88.000, 8.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(75)).build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(88.000, 8.000), new Pose(109.613, 12.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(75), Math.toRadians(180)).build();
        }
    }
}