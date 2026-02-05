//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.TelemetryManager;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@Autonomous(name = "Blue BACK 6 ARTIFACT (WORKING)", group = "Autonomous")
//@Configurable
//public class BlueBack6 extends OpMode {
//    private TelemetryManager panelsTelemetry;
//    public Follower follower;
//    private int pathState = 0;
//    private Paths paths;
//
//    private IntakeSystem intake;
//
//    private LimelightAligner limelightAligner;
//    private ShooterSystem shooter;
//    private ElapsedTime stateTimer = new ElapsedTime();
//
//    @Override
//    public void init() {
//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//        follower = Constants.createFollower(hardwareMap);
//        follower.setMaxPower(1);
//
//        intake = new IntakeSystem(hardwareMap);
//        limelightAligner = new LimelightAligner(hardwareMap);
//        shooter = new ShooterSystem(hardwareMap);
//
//        // Use your class method to set the velocity for back shooting
//        //shooter.backShooting();
//
//        follower.setStartingPose(new Pose(56.000, 8.000, Math.toRadians(90)));
//        paths = new Paths(follower);
//
//        panelsTelemetry.debug("Status", "Initialized");
//        panelsTelemetry.update(telemetry);
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        // Vital: Update the PIDF coefficients if you are tuning via Dashboard
//        shooter.updatePIDF();
//
//        autonomousPathUpdate();
//
//        // Telemetry for debugging PID performance
//        panelsTelemetry.debug("Path State", pathState);
//        panelsTelemetry.debug("Target Vel", shooter.getTargetVelocity());
//        panelsTelemetry.debug("Actual Vel", shooter.getActualVelocity());
//        panelsTelemetry.debug("Shooter Ready", shooter.isAtVelocity());
//        panelsTelemetry.update(telemetry);
//    }
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                shooter.startFlywheels(); // Start spinning early
//                follower.followPath(paths.Path0, true);
//                pathState = 1;
//                break;
//
//            case 1:
//                // Wait for the robot to reach the shooting position AND the motors to be at speed
//                if (!follower.isBusy() && shooter.isAtVelocity()) {
//                    shooter.shoot();
//                    stateTimer.reset();
//                    pathState = 2;
//                }
//                break;
//
//            case 2:
//                // Shooting duration
//                if (stateTimer.seconds() > 5.0) { // Reduced from 5.0 to 2.0 for efficiency
//                    shooter.stopFeeding();
//                    intake.runIntake();
//                    follower.followPath(paths.PathStraight, true);
//                    pathState = 67;
//                }
//                break;
//
//            case 67:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path1, true);
//                    pathState = 3;
//                }
//                break;
//
//            case 3:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path2, true);
//                    pathState = 4;
//                }
//                break;
//
//            case 4:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path3, true);
//                    pathState = 52;
//                }
//                break;
//
//            case 52:
//                if (!follower.isBusy()) {
//                    follower.setTeleOpDrive(0, 0, 0);
//                    stateTimer.reset();
//                    pathState = 53;
//                }
//                break;
//
//            case 53:
//                // Check velocity again before second volley
//                if (shooter.isAtVelocity()) {
//                    shooter.shoot();
//                    if (stateTimer.seconds() > 3.0) {
//                        shooter.stopFeeding();
//                        follower.followPath(paths.Path4, true);
//                        pathState = 22;
//                    }
//                }
//                break;
//
//            case 22:
//                // Logic to transition to picking up or finishing
//                shooter.stopFeeding();
//                intake.runIntake();
//                follower.followPath(paths.Path5, true);
//                pathState = 98;
//                break;
//
//            case 98:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path6, true);
//                    pathState = 7;
//                }
//                break;
//
//            case 7:
//                if (!follower.isBusy()) {
//                    intake.stopAll();
//                    shooter.stopAll();
//                    pathState = 8;
//                }
//                break;
//
//            case 8:
//                follower.setTeleOpDrive(0, 0, 0);
//                break;
//        }
//    }
//
//    // Paths class remains largely the same
//    public static class Paths {
//        public PathChain Path0, Path1, Path2, Path3, Path4, Path5, Path6, PathStraight;
//
//        public Paths(Follower follower) {
//            Path0 = follower.pathBuilder()
//                    .addPath(new BezierLine(new Pose(56.0, 8.0), new Pose(56.0, 14)))
//                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(125))
//                    .build();
//
//            PathStraight = follower.pathBuilder().addPath(
//                            new BezierLine(new Pose(56.0, 14.0), new Pose(54, 14.001)))
//                    .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(90))
//                    .build();
//
//            Path1 = follower.pathBuilder().addPath(
//                            new BezierLine(new Pose(54, 14.001), new Pose(53, 26)))
//                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
//                    .build();
//
//            Path2 = follower.pathBuilder().addPath(
//                            new BezierLine(new Pose(53, 26), new Pose(8, 25)))
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                    .build();
//
//            Path3 = follower.pathBuilder().addPath(
//                            new BezierLine(new Pose(8, 25), new Pose(55.849, 16)))
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(120))
//                    .build();
//
//            Path4 = follower.pathBuilder().addPath(
//                            new BezierLine(new Pose(55.849, 9), new Pose(44, 54)))
//                    .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(0))
//                    .build();
//
//            Path5 = follower.pathBuilder().addPath(
//                            new BezierLine(new Pose(44, 52), new Pose(9, 52)))
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                    .build();
//
//            Path6 = follower.pathBuilder().addPath(
//                            new BezierLine(new Pose(9, 54), new Pose(33, 14)))
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
//                    .build();
//        }
//    }
//}

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

@Autonomous(name = "Blue BACK 6 ARTIFACT (WORKING)", group = "Autonomous")
public class BlueBack6 extends OpMode {
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
        follower.setMaxPower(1);

        intake = new IntakeSystem(hardwareMap);
        limelightAligner = new LimelightAligner(hardwareMap);
        shooter = new ShooterSystem(hardwareMap);

        // Set the target speed specifically for this position
        //shooter.backShooting();

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
        panelsTelemetry.debug("Actual Vel", shooter.getActualVelocity());
        panelsTelemetry.debug("Shooter Ready", shooter.isAtVelocity());
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooter.startFlywheels();
                follower.followPath(paths.Path0, true);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    pathState = 10; // Transition to Limelight Alignment
                }
                break;

            case 10: // Align before first shot
                double alignPower = limelightAligner.calculateAlignPower(0);
                // Exit if aligned (power 0) or timeout (1.5s)
                if (alignPower == 0 || stateTimer.seconds() > 2.5) {
                    follower.setTeleOpDrive(0, 0, 0);
                    if (shooter.isAtVelocity()) {
                        shooter.shoot();
                        stateTimer.reset();
                        pathState = 2;
                    }
                } else {
                    follower.setTeleOpDrive(0, 0, alignPower);
                }
                break;

            case 2:
                if (stateTimer.seconds() > 5) { // Shooting duration
                    shooter.stopFeeding();
                    intake.runIntake();
                    follower.followPath(paths.PathStraight, true);
                    pathState = 67;
                }
                break;

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
                    follower.followPath(paths.Path3, true);
                    pathState = 52;
                }
                break;

            case 52:
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    pathState = 60; // Align before second volley
                }
                break;

            case 60: // Align before second volley
                double alignPower2 = limelightAligner.calculateAlignPower(0);
                if (alignPower2 == 0 || stateTimer.seconds() > 1.5) {
                    follower.setTeleOpDrive(0, 0, 0);
                    pathState = 53;
                } else {
                    follower.setTeleOpDrive(0, 0, alignPower2);
                }
                break;

            case 53:
                if (shooter.isAtVelocity()) {
                    shooter.shoot();
                    stateTimer.reset();
                    pathState = 54;
                }
                break;

            case 54:
                if (stateTimer.seconds() > 3.0) {
                    shooter.stopFeeding();
                    follower.followPath(paths.Path4, true);
                    pathState = 22;
                }
                break;

            case 22:
                shooter.stopFeeding();
                intake.runIntake();
                follower.followPath(paths.Path5, true);
                pathState = 98;
                break;

            case 98:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6, true);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    intake.stopAll();
                    shooter.stopAll();
                    limelightAligner.stop();
                    pathState = 8;
                }
                break;

            case 8:
                follower.setTeleOpDrive(0, 0, 0);
                break;
        }
    }

    public static class Paths {
        public PathChain Path0, Path1, Path2, Path3, Path4, Path5, Path6, PathStraight;

        public Paths(Follower follower) {
            Path0 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56.0, 8.0), new Pose(56.0, 14)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(125))
                    .build();

            PathStraight = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(56.0, 14.0), new Pose(54, 14.001)))
                    .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(90))
                    .build();

            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(54, 14.001), new Pose(53, 26)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(53, 26), new Pose(8, 25)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(8, 25), new Pose(55.849, 16)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(120))
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(55.849, 9), new Pose(44, 54)))
                    .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(0))
                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(44, 52), new Pose(9, 52)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(9, 54), new Pose(33, 14)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();
        }
    }
}