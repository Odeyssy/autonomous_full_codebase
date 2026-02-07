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

@Autonomous(name = "Blue FRONT 6 Artifact (FULL)", group = "Autonomous")
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

        // Set initial RPM for Front Side
        shooter.setTargetVelocity(1550.0);

        // Starting Pose
        follower.setStartingPose(new Pose(21, 123, Math.toRadians(142)));
        paths = new Paths(follower);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Debugging Telemetry
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

//    public void autonomousPathUpdate() {
//        double currentAlignPower = 0;
//
//        switch (pathState) {
//            case 0: // Move to Preload shooting spot
//                shooter.startFlywheels();
//                follower.followPath(paths.ShootPreloaded, true);
//                pathState = 100;
//                break;
//
//            case 100: // Wait for drive to finish
//                if (!follower.isBusy()) {
//                    stateTimer.reset();
//                    pathState = 66;
//                }
//                break;
//
//            case 66: // Align Preload
//                currentAlignPower = limelightAligner.calculateAlignPower(-99);
//                telemetry.addLine("--- ALIGNING PRELOAD ---");
//
//                if (currentAlignPower == -99) {
//                    follower.setTeleOpDrive(0, 0, 0, false);
//                    if (stateTimer.seconds() > 1.2) { stateTimer.reset(); pathState = 230; }
//                } else if (Math.abs(currentAlignPower) < 0.02 || stateTimer.seconds() > 3.0) {
//                    follower.setTeleOpDrive(0, 0, 0, false);
//                    stateTimer.reset();
//                    pathState = 230;
//                } else {
//                    follower.setTeleOpDrive(0, 0, -currentAlignPower, false);
//                }
//                break;
//
//            case 230: // --- FIRE PRELOADED ---
//                if (shooter.isAtVelocity()) {
//                    shooter.shoot(); // Starts ramp, gate, and servos
//                    stateTimer.reset(); // Start the 3-second dwell clock NOW
//                    pathState = 23;
//                }
//                break;
//
//            case 23: // --- STAY FOR 3 SECONDS ---
//                telemetry.addLine("Shooting Volley 1...");
//                telemetry.addData("Dwell Time", "%.2f / 3.00", stateTimer.seconds());
//
//                if (stateTimer.seconds() > 5.0) {
//                    shooter.stopFeeding();
//                    intake.runIntake();
//                    follower.followPath(paths.FirstIntake, true);
//                    pathState = 1;
//                }
//                break;
//
//            case 1: // Wait for turn, then drive to intake stack
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.CollectIntake, true);
//                    pathState = 11;
//                }
//                break;
//
//            case 11: // Wait for robot to reach intake stack
//                if (!follower.isBusy()) {
//                    stateTimer.reset();
//                    pathState = 110; // Brief pause to ensure intake
//                }
//                break;
//
//            case 110:
//                if (stateTimer.seconds() > 0.5) {
//                    follower.followPath(paths.ShootFirstIntake, true);
//                    pathState = 2;
//                }
//                break;
//
//            case 2: // Wait for drive back to goal
//                if (!follower.isBusy()) {
//                    stateTimer.reset();
//                    pathState = 85;
//                }
//                break;
//
//            case 85: // Align Volley 2
//                currentAlignPower = limelightAligner.calculateAlignPower(-99);
//                telemetry.addLine("--- ALIGNING VOLLEY 2 ---");
//
//                if (currentAlignPower == -99) {
//                    follower.setTeleOpDrive(0, 0, 0, false);
//                    if (stateTimer.seconds() > 1.2) { stateTimer.reset(); pathState = 850; }
//                } else if (Math.abs(currentAlignPower) < 0.02 || stateTimer.seconds() > 3.0) {
//                    follower.setTeleOpDrive(0, 0, 0, false);
//                    stateTimer.reset();
//                    pathState = 850;
//                } else {
//                    follower.setTeleOpDrive(0, 0, -currentAlignPower, false);
//                }
//                break;
//
//            case 850: // --- FIRE VOLLEY 2 ---
//                if (shooter.isAtVelocity()) {
//                    shooter.shoot();
//                    stateTimer.reset(); // Start the 3-second dwell clock NOW
//                    pathState = 36;
//                }
//                break;
//
//            case 36: // --- STAY FOR 3 SECONDS ---
//                telemetry.addLine("Shooting Volley 2...");
//                telemetry.addData("Dwell Time", "%.2f / 3.00", stateTimer.seconds());
//
//                if (stateTimer.seconds() > 5.0) {
//                    shooter.stopFeeding();
//                    intake.runIntake();
//                    follower.followPath(paths.SecondIntake, true);
//                    pathState = 3;
//                }
//                break;
//
//            case 3: // Wait for drive to second intake
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.CollectSecondIntake, true);
//                    pathState = 33;
//                }
//                break;
//
//            case 33: // Wait for collection
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.LeavePoints, true);
//                    pathState = 4;
//                }
//                break;
//
//            case 4: // Final Park
//                if (!follower.isBusy()) {
//                    intake.stopAll();
//                    shooter.stopAll();
//                    pathState = 8;
//                }
//                break;
//
//            case 8: // Done
//                follower.setTeleOpDrive(0, 0, 0);
//                break;
//        }
//        telemetry.update();
//    }

//    public void autonomousPathUpdate() {
//        double currentAlignPower = 0;
//        // Smallest power that actually moves your robot. Adjust 0.05 as needed.
//        final double MIN_ALIGN_POWER = 0.05;
//
//        switch (pathState) {
//            case 0: // Move to Preload shooting spot
//                shooter.startFlywheels();
//                follower.followPath(paths.ShootPreloaded, true);
//                pathState = 100;
//                break;
//
//            case 100: // Wait for drive to finish
//                if (!follower.isBusy()) {
//                    // STOP the follower's PID from holding position so Limelight can take over
//                    follower.setTeleOpDrive(0, 0, 0, false);
//
//                    stateTimer.reset();
//                    pathState = 66;
//                }
//                break;
//
//            case 66: // Align Preload
//                currentAlignPower = limelightAligner.calculateAlignPower(-99);
//                telemetry.addLine("--- ALIGNING PRELOAD ---");
//
//                if (currentAlignPower == -99) {
//                    follower.setTeleOpDrive(0, 0, 0, false);
//                    if (stateTimer.seconds() > 1.2) { stateTimer.reset(); pathState = 230; }
//                } else if (Math.abs(currentAlignPower) < 0.02 || stateTimer.seconds() > 3.0) {
//                    follower.setTeleOpDrive(0, 0, 0, false);
//                    stateTimer.reset();
//                    pathState = 230;
//                } else {
//                    // APPLY DEADZONE OVERRIDE: Ensure the robot actually turns
//                    double finalPower = -currentAlignPower;
//                    if (Math.abs(finalPower) > 0 && Math.abs(finalPower) < MIN_ALIGN_POWER) {
//                        finalPower = Math.signum(finalPower) * MIN_ALIGN_POWER;
//                    }
//                    follower.setTeleOpDrive(0, 0, finalPower, false);
//                }
//                break;
//
//            case 230: // --- FIRE PRELOADED ---
//                if (shooter.isAtVelocity()) {
//                    shooter.shoot();
//                    stateTimer.reset();
//                    pathState = 23;
//                }
//                break;
//
//            case 23: // --- STAY FOR 3 SECONDS ---
//                if (stateTimer.seconds() > 5.0) {
//                    shooter.stopFeeding();
//                    intake.runIntake();
//                    follower.followPath(paths.FirstIntake, true);
//                    pathState = 1;
//                }
//                break;
//
//            case 1:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.CollectIntake, true);
//                    pathState = 11;
//                }
//                break;
//
//            case 11:
//                if (!follower.isBusy()) {
//                    stateTimer.reset();
//                    pathState = 110;
//                }
//                break;
//
//            case 110:
//                if (stateTimer.seconds() > 0.5) {
//                    follower.followPath(paths.ShootFirstIntake, true);
//                    pathState = 2;
//                }
//                break;
//
//            case 2: // Wait for drive back to goal
//                if (!follower.isBusy()) {
//                    // BREAK AGAIN: Pathing is done, let Limelight rotate the robot
//                    follower.setTeleOpDrive(0, 0, 0, false);
//                    stateTimer.reset();
//                    pathState = 85;
//                }
//                break;
//
//            case 85: // Align Volley 2
//                currentAlignPower = limelightAligner.calculateAlignPower(-99);
//                telemetry.addLine("--- ALIGNING VOLLEY 2 ---");
//
//                if (currentAlignPower == -99) {
//                    follower.setTeleOpDrive(0, 0, 0, false);
//                    if (stateTimer.seconds() > 1.2) { stateTimer.reset(); pathState = 850; }
//                } else if (Math.abs(currentAlignPower) < 0.02 || stateTimer.seconds() > 3.0) {
//                    follower.setTeleOpDrive(0, 0, 0, false);
//                    stateTimer.reset();
//                    pathState = 850;
//                } else {
//                    double finalPower = -currentAlignPower;
//                    if (Math.abs(finalPower) > 0 && Math.abs(finalPower) < MIN_ALIGN_POWER) {
//                        finalPower = Math.signum(finalPower) * MIN_ALIGN_POWER;
//                    }
//                    follower.setTeleOpDrive(0, 0, finalPower, false);
//                }
//                break;
//
//            case 850:
//                if (shooter.isAtVelocity()) {
//                    shooter.shoot();
//                    stateTimer.reset();
//                    pathState = 36;
//                }
//                break;
//
//            case 36:
//                if (stateTimer.seconds() > 5.0) {
//                    shooter.stopFeeding();
//                    intake.runIntake();
//                    follower.followPath(paths.SecondIntake, true);
//                    pathState = 3;
//                }
//                break;
//
//            case 3:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.CollectSecondIntake, true);
//                    pathState = 33;
//                }
//                break;
//
//            case 33:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.LeavePoints, true);
//                    pathState = 4;
//                }
//                break;
//
//            case 4:
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
//        telemetry.update();
//    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooter.startFlywheels();
                follower.followPath(paths.ShootPreloaded, true);
                pathState = 100;
                break;

            case 100: // Transition to Align
                if (!follower.isBusy()) {
                    // Ensure Pedro is not trying to drive while we use raw motor power
                    follower.setTeleOpDrive(0,0,0,false);
                    stateTimer.reset();
                    pathState = 66;
                }
                break;

            case 66: // Align Preload
                int alignStatus = limelightAligner.driveToTarget();
                telemetry.addLine("--- DIRECT MOTOR ALIGNMENT ---");

                if (alignStatus == 1 || stateTimer.seconds() > 5) {
                    // SUCCESS or TIMEOUT
                    limelightAligner.stopMotors();
                    stateTimer.reset();
                    pathState = 230;
                } else if (alignStatus == -1 && stateTimer.seconds() > 5) {
                    // LOST TARGET for too long
                    limelightAligner.stopMotors();
                    stateTimer.reset();
                    pathState = 230;
                }
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
                telemetry.addLine("--- DIRECT MOTOR ALIGNMENT 2 ---");

                if (alignStatus2 == 1 || stateTimer.seconds() > 5) {
                    limelightAligner.stopMotors();
                    stateTimer.reset();
                    pathState = 850;
                } else if (alignStatus2 == -1 && stateTimer.seconds() > 5) {
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
                    .addPath(new BezierLine(new Pose(21, 123), new Pose(72, 73)))
                    .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(142))
                    .build();

            FirstIntake = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(72, 73), new Pose(72, 73.001)))
                    .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(0))
                    .build();

            CollectIntake = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(72, 73.001), new Pose(22, 73)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            ShootFirstIntake = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(22, 73), new Pose(69, 73)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(142))
                    .build();

            SecondIntake = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(69, 73), new Pose(69, 60)))
                    .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(0))
                    .build();

            CollectSecondIntake = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(69, 60), new Pose(25, 60)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            LeavePoints = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(25, 60), new Pose(25, 15)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }
}