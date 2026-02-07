package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Red BACK 6 Artifact", group = "Autonomous")
public class RedBack6 extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;

    private IntakeSystem intake;
    private ShooterSystem shooter;
    private ElapsedTime stateTimer = new ElapsedTime();

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);

        intake = new IntakeSystem(hardwareMap);
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
        panelsTelemetry.debug("Shooter Ready", shooter.isAtVelocity());
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            // ===================== DRIVE TO (88, 14) =====================
            case 0:
                shooter.startFlywheels();
                follower.followPath(paths.Path0, true);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    pathState = 2;
                }
                break;

            // ===================== VOLLEY 1 @ (88, 14) =====================
            case 2:
                intake.runIntake();
                shooter.shoot();
                if (stateTimer.seconds() > 7) {
                    shooter.stopFeeding();
                    intake.runIntake(); // Keep intake on for pickup
                    follower.followPath(paths.Path1, true);
                    pathState = 3;
                }
                break;

            // ===================== CYCLE 1: PICK UP (intake stays on) =====================
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2, true);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    intake.stopAll(); // Done picking up
                    follower.followPath(paths.Path3, true);
                    pathState = 52;
                }
                break;

            case 52:
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    pathState = 53;
                }
                break;

            // ===================== VOLLEY 2 @ (88, 12) =====================
            case 53:
                if (shooter.isAtVelocity()) {
                    intake.runIntake();
                    shooter.shoot();
                    stateTimer.reset();
                    pathState = 54;
                } else if (stateTimer.seconds() > 3.0) {
                    intake.runIntake();
                    shooter.shoot();
                    stateTimer.reset();
                    pathState = 54;
                }
                break;

            case 54:
                if (stateTimer.seconds() > 3) {
                    shooter.stopFeeding();
                    intake.runIntake(); // Keep intake on for next pickup
                    follower.followPath(paths.Path4, true);
                    pathState = 5;
                }
                break;

            // ===================== CYCLE 2: PICK UP (intake stays on) =====================
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5, true);
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    intake.stopAll(); // Done picking up
                    follower.followPath(paths.Path6, true);
                    pathState = 67;
                }
                break;

            case 67:
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    pathState = 55;
                }
                break;

            // ===================== VOLLEY 3 @ (88, 14) =====================
            case 55:
                if (shooter.isAtVelocity()) {
                    intake.runIntake();
                    shooter.shoot();
                    stateTimer.reset();
                    pathState = 63;
                } else if (stateTimer.seconds() > 3.0) {
                    intake.runIntake();
                    shooter.shoot();
                    stateTimer.reset();
                    pathState = 63;
                }
                break;

            case 63:
                if (stateTimer.seconds() > 2.5) {
                    shooter.stopFeeding();
                    intake.stopAll();
                    follower.followPath(paths.Path7, true);
                    pathState = 7;
                }
                break;

            // ===================== PARK =====================
            case 7:
                if (!follower.isBusy()) {
                    intake.stopAll();
                    shooter.stopAll();
                    pathState = 8;
                }
                break;

            case 8:
                // Final idle
                break;
        }
        telemetry.update();
    }

    public static class Paths {
        public PathChain Path0, Path1, Path2, Path3, Path4, Path5, Path6, Path7;

        public Paths(Follower follower) {
            Path0 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(88.0, 8.0), new Pose(88.0, 14)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(60))
                    .build();

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(88.0, 14), new Pose(88.0, 40)))
                    .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(180))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(88.0, 40), new Pose(128.0, 34.939)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(128.0, 34.939), new Pose(88.0, 12.0)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(55))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(88.0, 12.0), new Pose(88, 64)))
                    .setLinearHeadingInterpolation(Math.toRadians(55), Math.toRadians(180))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(88, 64), new Pose(125, 64)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(125, 64), new Pose(88, 14)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(55))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(88, 14), new Pose(125, 14)))
                    .setLinearHeadingInterpolation(Math.toRadians(55), Math.toRadians(180))
                    .build();
        }
    }
}