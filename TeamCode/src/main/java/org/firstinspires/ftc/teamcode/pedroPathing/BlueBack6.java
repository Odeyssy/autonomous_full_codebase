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

@Autonomous(name = "Blue BACK 6 ARTIFACT", group = "Autonomous")
public class BlueBack6 extends OpMode {
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

    /*
     * STATE MACHINE:
     *  0  -> Start flywheels, drive to (56, 14)
     *  1  -> Wait for arrival
     *  2  -> SHOOT VOLLEY 1 (intake + shoot for 6s)
     *  67 -> Straighten heading
     *  3  -> Drive to (56, 26)
     *  4  -> Push to (8, 25)
     *  52 -> Return to (55.849, 16), wait for arrival
     *  53 -> SHOOT VOLLEY 2 (shoot when ready)
     *  54 -> Wait for volley, head to (44, 54)
     *  22 -> Drive to (9, 52) with intake on
     *  98 -> Return to (56, 14)
     *  520-> Wait for arrival
     *  55 -> SHOOT VOLLEY 3 (shoot when ready)
     *  63 -> Wait for volley
     *  69 -> Drive to park (8, 14)
     *  7  -> Stop everything
     *  8  -> Idle
     */
    public void autonomousPathUpdate() {
        switch (pathState) {

            // ===================== DRIVE TO SHOOTING POSITION =====================
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

            // ===================== VOLLEY 1 @ (56, 14) =====================
            case 2:
                intake.runIntake();
                shooter.shoot();
                if (stateTimer.seconds() > 6.0) {
                    shooter.stopFeeding();
                    intake.runIntake(); // Keep intake on for pickup
                    follower.followPath(paths.PathStraight, true);
                    pathState = 67;
                }
                break;

            // ===================== CYCLE 1: PICK UP (intake stays on) =====================
            case 67:
                if (!follower.isBusy()) {
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

            // ===================== VOLLEY 2 @ (55.849, 16) =====================
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
                if (stateTimer.seconds() > 3.0) {
                    shooter.stopFeeding();
                    intake.runIntake(); // Keep intake on for next pickup
                    follower.followPath(paths.Path4, true);
                    pathState = 22;
                }
                break;

            // ===================== CYCLE 2: PICK UP (intake stays on) =====================
            case 22:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5, true);
                    pathState = 98;
                }
                break;

            case 98:
                if (!follower.isBusy()) {
                    intake.stopAll(); // Done picking up
                    follower.followPath(paths.Path6, true);
                    pathState = 520;
                }
                break;

            case 520:
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    pathState = 55;
                }
                break;

            // ===================== VOLLEY 3 @ (56, 14) =====================
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
                if (stateTimer.seconds() > 3.0) {
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
        public PathChain Path0, Path1, Path2, Path3, Path4, Path5, Path6, Path7, PathStraight;

        public Paths(Follower follower) {
            Path0 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56.0, 8.0), new Pose(56.0, 14)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(123))
                    .build();

            PathStraight = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(56.0, 14.0), new Pose(56, 14.001)))
                    .setLinearHeadingInterpolation(Math.toRadians(123), Math.toRadians(90))
                    .build();

            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(56, 14.001), new Pose(56, 26)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(56, 26), new Pose(6, 25)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(8, 25), new Pose(55.849, 16)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(117))
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(55.849, 9), new Pose(44, 54)))
                    .setLinearHeadingInterpolation(Math.toRadians(117), Math.toRadians(0))
                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(44, 52), new Pose(9, 52)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(9, 54), new Pose(56, 14)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(117))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56, 14), new Pose(8, 14)))
                    .setLinearHeadingInterpolation(Math.toRadians(117), Math.toRadians(0))
                    .build();
        }
    }
}