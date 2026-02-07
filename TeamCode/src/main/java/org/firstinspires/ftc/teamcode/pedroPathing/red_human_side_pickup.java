package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.IntakeSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.ShooterSystem;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Red Human Side Pick up", group = "Autonomous")
public class red_human_side_pickup extends OpMode {
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

        follower.setStartingPose(new Pose(88.361, 6.916, Math.toRadians(90)));
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Red Side Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            // ===================== DRIVE TO (88, 15) =====================
            case 0:
                shooter.startFlywheels();
                follower.followPath(paths.PathToShoot, true);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    pathState = 10;
                }
                break;

            // ===================== VOLLEY 1 @ (88, 15) =====================
            case 10:
                if (shooter.isAtVelocity()) {
                    intake.runIntake();
                    shooter.shoot();
                    stateTimer.reset();
                    pathState = 11;
                } else if (stateTimer.seconds() > 3.0) {
                    intake.runIntake();
                    shooter.shoot();
                    stateTimer.reset();
                    pathState = 11;
                }
                break;

            case 11: // Volley 1 duration, then head to pickup with intake ON
                if (stateTimer.seconds() > 5) {
                    shooter.stopFeeding();
                    intake.runIntake(); // Keep intake running for pickup
                    follower.followPath(paths.PathToIntake, true);
                    pathState = 12;
                }
                break;

            // ===================== CYCLE 1: PICK UP & RETURN (intake stays on) =====================
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PathPickUp, true);
                    pathState = 13;
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    intake.stopAll(); // Done picking up, stop intake before returning
                    follower.followPath(paths.PathReturnToShoot, true);
                    pathState = 14;
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    pathState = 30;
                }
                break;

            // ===================== VOLLEY 2 @ (88, 15) =====================
            case 30:
                if (shooter.isAtVelocity()) {
                    intake.runIntake();
                    shooter.shoot();
                    stateTimer.reset();
                    pathState = 31;
                } else if (stateTimer.seconds() > 3.0) {
                    intake.runIntake();
                    shooter.shoot();
                    stateTimer.reset();
                    pathState = 31;
                }
                break;

            case 31: // Volley 2 duration, then head to park pickup with intake ON
                if (stateTimer.seconds() > 5) {
                    shooter.stopFeeding();
                    intake.runIntake(); // Keep intake running for pickup
                    follower.followPath(paths.PathToPark, true);
                    pathState = 32;
                }
                break;

            // ===================== CYCLE 2: PARK AREA & RETURN (intake stays on) =====================
            case 32:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PathParkReturn, true);
                    pathState = 33;
                }
                break;

            case 33:
                if (!follower.isBusy()) {
                    intake.stopAll(); // Done picking up
                    stateTimer.reset();
                    pathState = 50;
                }
                break;

            // ===================== VOLLEY 3 @ (88, 15) =====================
            case 50:
                if (shooter.isAtVelocity()) {
                    intake.runIntake();
                    shooter.shoot();
                    stateTimer.reset();
                    pathState = 51;
                } else if (stateTimer.seconds() > 3.0) {
                    intake.runIntake();
                    shooter.shoot();
                    stateTimer.reset();
                    pathState = 51;
                }
                break;

            case 51:
                if (stateTimer.seconds() > 5) {
                    shooter.stopFeeding();
                    intake.stopAll();
                    follower.followPath(paths.PathFinalPark, true);
                    pathState = 100;
                }
                break;

            // ===================== DONE =====================
            case 100:
                if (!follower.isBusy()) {
                    intake.stopAll();
                    shooter.stopAll();
                    pathState = -1;
                }
                break;
        }
    }

    public static class Paths {
        public PathChain PathToShoot, PathToIntake, PathPickUp, PathReturnToShoot;
        public PathChain PathToPark, PathParkReturn, PathFinalPark;

        public Paths(Follower follower) {
            PathToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(88.361, 6.916), new Pose(88.000, 15.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(60))
                    .build();

            PathToIntake = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(88.000, 15.000), new Pose(104.241, 35.218)))
                    .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(180))
                    .build();

            PathPickUp = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(104.241, 35.218), new Pose(132.284, 34.972)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            PathReturnToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(132.284, 34.972), new Pose(88.000, 15.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(60))
                    .build();

            PathToPark = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(88.000, 15.000), new Pose(135.779, 9.251)))
                    .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(180))
                    .build();

            PathParkReturn = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(135.779, 9.251), new Pose(88.000, 15.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(60))
                    .build();

            PathFinalPark = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(88.000, 15.000), new Pose(88.445, 47.238)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();
        }
    }
}