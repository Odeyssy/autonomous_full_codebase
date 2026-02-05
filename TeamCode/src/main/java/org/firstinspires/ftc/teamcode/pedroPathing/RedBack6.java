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
//@Autonomous(name = "Red BACK 6 Artifact (WORKING)", group = "Autonomous")
//@Configurable
//public class RedBack6 extends OpMode {
//    private TelemetryManager panelsTelemetry;
//    public Follower follower;
//    private int pathState = 0;
//    private Paths paths;
//
//    // Systems
//    private IntakeSystem intake;
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
//        follower.setStartingPose(new Pose(88.0, 8.0, Math.toRadians(90)));
//
//        paths = new Paths(follower);
//
//        panelsTelemetry.debug("Status", "Initialized");
//        panelsTelemetry.update(telemetry);
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        autonomousPathUpdate();
//
//        panelsTelemetry.debug("Path State", pathState);
//        panelsTelemetry.debug("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
//        panelsTelemetry.debug("Busy", follower.isBusy());
//        panelsTelemetry.update(telemetry);
//    }
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                // START TURN ONLY (90° → 60°)
//                shooter.startFlywheels();
//                follower.followPath(paths.Path0, true);
//                pathState = 1;
//                break;
//
//            case 1:
//                // WAIT FOR TURN TO FINISH
//                if (!follower.isBusy()) {
//                    shooter.shoot();
//                    stateTimer.reset();
//                    pathState = 2;
//                }
//                break;
//
//            case 2:
//                if (stateTimer.seconds() > 5.0) {
//                    shooter.stopFeeding();
//                    intake.runIntake();
//                    follower.followPath(paths.Path1, true);
//                    pathState = 3;
//                }
//                break;
//
//            case 3:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path2, true);
//                    pathState = 4; //4;
//                }
//                break;
//
//            case 4:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path3, true);
//                    pathState = 52; //52;
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
//                shooter.shoot();
//                if (stateTimer.seconds() > 4.0) {
//                    shooter.stopFeeding();
//                    follower.followPath(paths.Path4, true);
//                    pathState = 7; //5;
//                }
//                break;
//
//            case 5:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path5, true);
//                    pathState = 7; //6;
//                }
//                break;
//
//            case 6:
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
//    public static class Paths {
//
//        public PathChain Path0, Path1, Path2, Path3, Path4, Path5, Path6;
//
//        public Paths(Follower follower) {
//            Path0 = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(88.0, 8.0),
//                            new Pose(88.0, 14)
//                    ))
//                    .setLinearHeadingInterpolation(
//                            Math.toRadians(90),
//                            Math.toRadians(60)
//                    )
//                    .build();
//
//            Path1 = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(88.0, 14),
//                            new Pose(88.0, 25)
//                    ))
//                    .setLinearHeadingInterpolation(
//                            Math.toRadians(60),
//                            Math.toRadians(180)
//                    )
//                    .build();
//
//            Path2 = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(88.0, 25),
//                            new Pose(125.0, 34.939)
//                    ))
//                    .setLinearHeadingInterpolation(
//                            Math.toRadians(180),
//                            Math.toRadians(180)
//                    )
//                    .build();
//
//            Path3 = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(125.0, 34.939),
//                            new Pose(88.0, 12.0)
//                    ))
//                    .setLinearHeadingInterpolation(
//                            Math.toRadians(180),
//                            Math.toRadians(60)
//                    )
//                    .build();
//
//            Path4 = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(88.0, 8.0),
//                            new Pose(134.746, 15.0)
//                    ))
//                    .setLinearHeadingInterpolation(
//                            Math.toRadians(60),
//                            Math.toRadians(180)
//                    )
//                    .build();
//
//            Path5 = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(134.746, 15.0),
//                            new Pose(88.0, 8.0)
//                    ))
//                    .setLinearHeadingInterpolation(
//                            Math.toRadians(180),
//                            Math.toRadians(75)
//                    )
//                    .build();
//
//            Path6 = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(88.0, 8.0),
//                            new Pose(109.613, 12.0)
//                    ))
//                    .setLinearHeadingInterpolation(
//                            Math.toRadians(75),
//                            Math.toRadians(180)
//                    )
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

@Autonomous(name = "Red BACK 6 Artifact (WORKING)", group = "Autonomous")
public class RedBack6 extends OpMode {
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
        follower.setMaxPower(1);

        intake = new IntakeSystem(hardwareMap);
        limelightAligner = new LimelightAligner(hardwareMap);
        shooter = new ShooterSystem(hardwareMap);

        // Ensure starting velocity is set correctly
        //shooter.backShooting();

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
        panelsTelemetry.debug("Actual Vel", shooter.getActualVelocity());
        panelsTelemetry.debug("Shooter Ready", shooter.isAtVelocity());
        panelsTelemetry.update(telemetry);
    }

//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                shooter.startFlywheels();
//                follower.followPath(paths.Path0, true);
//                pathState = 1;
//                break;
//
//            case 1:
//                // Wait for movement to end, then trigger Limelight
//                if (!follower.isBusy()) {
//                    stateTimer.reset();
//                    pathState = 10; // Alignment state
//                }
//                break;
//
//            case 10: // ALIGNMENT BEFORE VOLLEY 1
//                double alignPower = limelightAligner.calculateAlignPower(0);
//
//                // Exit if centered (power 0) or 1.5 second timeout
//                if (alignPower == 0 || stateTimer.seconds() > 4) {
//                    follower.setTeleOpDrive(0, 0, 0);
//                    if (shooter.isAtVelocity()) {
//                        shooter.shoot();
//                        stateTimer.reset();
//                        pathState = 2;
//                    }
//                } else {
//                    telemetry.addData("Align power: ",alignPower);
//                    telemetry.update();
//                    follower.setTeleOpDrive(0, 0, alignPower);
//
//                }
//                break;
//
//            case 2:
//                // Wait for shot to complete (tuned down from 5s to 2.5s)
//                if (stateTimer.seconds() > 5) {
//                    shooter.stopFeeding();
//                    intake.runIntake();
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
//                    stateTimer.reset();
//                    pathState = 60; // Alignment before volley 2
//                }
//                break;
//
//            case 60: // ALIGNMENT BEFORE VOLLEY 2
//                double alignPower2 = limelightAligner.calculateAlignPower(0);
//
//                if (alignPower2 == 0 || stateTimer.seconds() > 1.5) {
//                    follower.setTeleOpDrive(0, 0, 0);
//                    pathState = 53;
//                } else {
//                    telemetry.addData("Align power: ",alignPower2);
//                    telemetry.update();
//                    follower.setTeleOpDrive(0, 0, alignPower2);
//                }
//                break;
//
//            case 53:
//                if (shooter.isAtVelocity()) {
//                    shooter.shoot();
//                    stateTimer.reset();
//                    pathState = 54;
//                }
//                break;
//
//            case 54:
//                if (stateTimer.seconds() > 3) {
//                    shooter.stopFeeding();
//                    follower.followPath(paths.Path4, true);
//                    pathState = 5;
//                }
//                break;
//
//            case 5:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path5, true);
//                    pathState = 6;
//                }
//                break;
//            case 6:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path6, true);
//                    pathState = 67;
//                }
//                break;
//
//
//            case 67:
//                if (!follower.isBusy()) {
//                    stateTimer.reset();
//                    pathState = 61;
//                }
//
//            case 61: // ALIGNMENT BEFORE VOLLEY 3
//
//                double alignPower3 = limelightAligner.calculateAlignPower(0);
//                if (alignPower3 == 0 || stateTimer.seconds() > 1.5) {
//                    follower.setTeleOpDrive(0, 0, 0);
//                    pathState = 55;
//                } else {
//                    telemetry.addData("Align power: ",alignPower3);
//                    telemetry.update();
//                    follower.setTeleOpDrive(0, 0, alignPower3);
//                }
//                break;
//
//            case 55:
//                if (shooter.isAtVelocity()) {
//                    shooter.shoot();
//                    stateTimer.reset();
//                    pathState = 63;
//                }
//                break;
//
//            case 63:
//                if (stateTimer.seconds() > 5.0) {
//                    shooter.stopFeeding();
//                    //follower.followPath(paths.Path4, true);
//                    pathState = 7;
//                }
//                break;
//
//
//
//            case 7:
//                if (!follower.isBusy()) {
//                    intake.stopAll();
//                    shooter.stopAll();
//                    limelightAligner.stop();
//                    pathState = 8;
//                }
//                break;
//
//            case 8:
//                follower.setTeleOpDrive(0, 0, 0);
//                break;
//        }
//    }

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
                    pathState = 10;
                }
                break;

            case 10: // ALIGNMENT BEFORE VOLLEY 1
                double alignPower = limelightAligner.calculateAlignPower(0);
                if (alignPower == 0 || stateTimer.seconds() > 4) {
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
                if (stateTimer.seconds() > 7) {
                    shooter.stopFeeding(); // Closes Gate
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
                    stateTimer.reset();
                    pathState = 60;
                }
                break;

            case 60: // ALIGNMENT BEFORE VOLLEY 2
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
                    shooter.shoot(); // Opens Gate
                    stateTimer.reset();
                    pathState = 54;
                }
                break;

            case 54:
                if (stateTimer.seconds() > 3) {
                    shooter.stopFeeding(); // Closes Gate
                    follower.followPath(paths.Path4, true);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5, true);
                    pathState = 6;
                }
                break; // ADDED BREAK

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6, true);
                    pathState = 67;
                }
                break; // ADDED BREAK

            case 67:
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    pathState = 61;
                }
                break; // ADDED BREAK

            case 61: // ALIGNMENT BEFORE VOLLEY 3
                double alignPower3 = limelightAligner.calculateAlignPower(0);
                if (alignPower3 == 0 || stateTimer.seconds() > 1.5) {
                    follower.setTeleOpDrive(0, 0, 0);
                    pathState = 55;
                } else {
                    follower.setTeleOpDrive(0, 0, alignPower3);
                }
                break;

            case 55:
                if (shooter.isAtVelocity()) {
                    shooter.shoot(); // Opens Gate
                    stateTimer.reset();
                    pathState = 63;
                }
                break;

            case 63:
                if (stateTimer.seconds() > 2.5) {
                    shooter.stopFeeding(); // Closes Gate
                    pathState = 68;
                }
                break;

            case 68:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7, true);
                    pathState = 7;
                }
                break; // ADDED BREAK

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
                    .addPath(new BezierLine(new Pose(88.0, 40), new Pose(125.0, 34.939)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(125.0, 34.939), new Pose(88.0, 12.0)))
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