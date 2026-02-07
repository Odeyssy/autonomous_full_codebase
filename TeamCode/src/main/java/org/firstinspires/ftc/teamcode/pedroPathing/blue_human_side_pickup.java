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

@Autonomous(name = "blue human side pick up", group = "Autonomous")
public class blue_human_side_pickup extends OpMode {
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

        follower.setStartingPose(new Pose(56.000, 8.000, Math.toRadians(90)));
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Blue Side Initialized");
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

    /*
     * STATE MACHINE:
     *  0  -> Start flywheels, drive to (56, 15.222)
     *  1  -> Wait for arrival
     *  10 -> SHOOT VOLLEY 1 @ (56, 15)
     *  11 -> Wait for volley, then drive to intake (41.99, 35.295)
     *  12 -> Wait, then push pickup (41.99, 35.295) -> (11.32, 35.455)
     *  13 -> Wait, then return to (56, 15)
     *  14 -> Wait for arrival
     *  30 -> SHOOT VOLLEY 2 @ (56, 15)
     *  31 -> Wait for volley, then drive to park area (8.01, 8.107)
     *  32 -> Wait, then return to (56, 15)
     *  33 -> Wait for arrival
     *  50 -> SHOOT VOLLEY 3 @ (56, 15)
     *  51 -> Wait for volley, then drive to final park (56.5, 56.59)
     * 100 -> Stop everything
     */
    public void autonomousPathUpdate() {
        switch (pathState) {

            // ===================== DRIVE TO (56, 15) =====================
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

            // ===================== VOLLEY 1 @ (56, 15) =====================
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

            case 11: // Volley done, head to pickup with intake ON
                if (stateTimer.seconds() > 5) {
                    shooter.stopFeeding();
                    intake.runIntake();
                    follower.followPath(paths.PathToIntake, true);
                    pathState = 12;
                }
                break;

            // ===================== CYCLE 1: PICK UP & RETURN (intake on) =====================
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PathPickUp, true);
                    pathState = 13;
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    intake.stopAll();
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

            // ===================== VOLLEY 2 @ (56, 15) =====================
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

            case 31: // Volley done, head to park pickup with intake ON
                if (stateTimer.seconds() > 5) {
                    shooter.stopFeeding();
                    intake.runIntake();
                    follower.followPath(paths.PathToPark, true);
                    pathState = 32;
                }
                break;

            // ===================== CYCLE 2: PARK AREA & RETURN (intake on) =====================
            case 32:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PathParkReturn, true);
                    pathState = 33;
                }
                break;

            case 33:
                if (!follower.isBusy()) {
                    intake.stopAll();
                    stateTimer.reset();
                    pathState = 50;
                }
                break;

            // ===================== VOLLEY 3 @ (56, 15) =====================
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
            // Start (56, 8) -> Shooting pos (56, 15.222)
            PathToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56.000, 8.000), new Pose(56.000, 15.222)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))
                    .build();

            // (56, 15.222) -> Intake area (41.99, 35.295)
            PathToIntake = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56.000, 15.222), new Pose(41.990, 35.295)))
                    .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(0))
                    .build();

            // Push across (41.99, 35.295) -> (11.32, 35.455)
            PathPickUp = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(41.990, 35.295), new Pose(11.320, 35.455)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Return (11.32, 35.455) -> (56, 15)
            PathReturnToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(11.320, 35.455), new Pose(56.000, 15.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(115))
                    .build();

            // (56, 15) -> Park zone (8.01, 8.107)
            PathToPark = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56.000, 15.000), new Pose(8.010, 8.107)))
                    .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(0))
                    .build();

            // Park zone (8.01, 8.107) -> Back to (56, 15)
            PathParkReturn = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(8.010, 8.107), new Pose(56.000, 15.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(115))
                    .build();

            // (56, 15) -> Final park (56.501, 56.59)
            PathFinalPark = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56.000, 15.000), new Pose(56.501, 56.590)))
                    .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(90))
                    .build();
        }
    }
}