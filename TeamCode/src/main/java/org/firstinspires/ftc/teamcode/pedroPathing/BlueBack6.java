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

        //shooter.backShooting(); // Set target to 1650

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
        double currentAlignPower = 0;
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

//            case 10: // Align for volley 1
//                currentAlignPower = limelightAligner.calculateAlignPower(-99);
//
//                // Driver Hub Telemetry
//                telemetry.addLine("--- VOLLEY 1 ALIGNMENT ---");
//                telemetry.addData("Status", (currentAlignPower == -99) ? "LOST TARGET" : "LOCKING");
//                telemetry.addData("Power", "%.3f", currentAlignPower);
//                telemetry.addData("Timer", "%.2f s", stateTimer.seconds());
//
//                if (currentAlignPower == -99) {
//                    follower.setTeleOpDrive(0, 0, 0, false);
//                } else if (Math.abs(currentAlignPower) < 0.02 || stateTimer.seconds() > 5) {
//                    follower.setTeleOpDrive(0, 0, 0, false);
//                    if (shooter.isAtVelocity()) {
//                        shooter.shoot();
//                        stateTimer.reset();
//                        pathState = 2;
//                    }
//                } else {
//                    // Ensure Robot-Centric (false) and check rotation direction
//                    follower.setTeleOpDrive(0, 0, -currentAlignPower, false);
//                }
//                break;

            case 10: // Align for volley 1
                currentAlignPower = limelightAligner.calculateAlignPower(-99);

                // Driver Hub Telemetry
                telemetry.addLine("--- VOLLEY 1 ALIGNMENT ---");
                telemetry.addData("Status", (currentAlignPower == -99) ? "LOST TARGET" : "LOCKING");
                telemetry.addData("Power", "%.3f", currentAlignPower);
                telemetry.addData("Timer", "%.2f s", stateTimer.seconds());

                if (currentAlignPower == -99) {
                    // TARGET LOST: Stop movement and wait for target to appear
                    follower.setTeleOpDrive(0, 0, 0, false);

                    // Safety: If we've looked for 1.5s and still see nothing, skip to shooting
                    if (stateTimer.seconds() > 1.5) {
                        pathState = 2; // Or wherever your "failed lock" fallback is
                        stateTimer.reset();
                    }
                }
                else if (Math.abs(currentAlignPower) < 0.02 || stateTimer.seconds() > 5.0) {
                    // TARGET LOCKED: We are within the threshold OR we hit the 5s hard timeout
                    follower.setTeleOpDrive(0, 0, 0, false);

                    if (shooter.isAtVelocity()) {
                        shooter.shoot();
                        stateTimer.reset();
                        pathState = 2;
                    }
                }
                else {
                    // TARGET TRACKING: We see it, now rotate to center it
                    // Note: Using -currentAlignPower based on your previous working code
                    follower.setTeleOpDrive(0, 0, -currentAlignPower, false);
                }
                break;
            case 2:
                if (stateTimer.seconds() > 6.0) { // Volley 1 duration
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
                    pathState = 60;
                }
                break;

//            case 60: // Align before Volley 2
//                currentAlignPower = limelightAligner.calculateAlignPower(-99);
//
//                telemetry.addLine("--- VOLLEY 2 ALIGNMENT ---");
//                telemetry.addData("Power", "%.3f", currentAlignPower);
//
//                if (currentAlignPower == 0 || stateTimer.seconds() > 5) {
//                    follower.setTeleOpDrive(0, 0, 0, false);
//                    pathState = 53;
//                } else if (currentAlignPower == -99) {
//                    follower.setTeleOpDrive(0, 0, 0, false);
//                    if (stateTimer.seconds() > 1.0) pathState = 53;
//                } else {
//                    follower.setTeleOpDrive(0, 0, currentAlignPower, false);
//                }
//                break;

            case 60: // Align before Volley 2
                currentAlignPower = limelightAligner.calculateAlignPower(-99);

                telemetry.addLine("--- VOLLEY 2 ALIGNMENT ---");
                telemetry.addData("Power", "%.3f", currentAlignPower);
                telemetry.addData("Timer", "%.2f", stateTimer.seconds());

                if (currentAlignPower == -99) {
                    // We have no target. Stop and wait for one.
                    follower.setTeleOpDrive(0, 0, 0, false);

                    // If we've looked for over 1.5 seconds and seen nothing, give up and shoot anyway
                    if (stateTimer.seconds() > 1.5) {
                        pathState = 53;
                        stateTimer.reset();
                    }
                }
                else if (currentAlignPower == 0 || stateTimer.seconds() > 5.0) {
                    // We HAVE a target and it's centered (0), OR we hit the hard timeout (5s)
                    follower.setTeleOpDrive(0, 0, 0, false);
                    pathState = 53;
                    stateTimer.reset();
                }
                else {
                    // We HAVE a target and we are actively moving toward it
                    follower.setTeleOpDrive(0, 0, currentAlignPower, false);
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
                    pathState = 61; // Go to Volley 3 alignment
                }
                break;

            case 61: // Align before Volley 3
                if (!follower.isBusy()) {
                    double alignPower3 = limelightAligner.calculateAlignPower(0);
                    if (alignPower3 == 0 || stateTimer.seconds() > 1.5) {
                        follower.setTeleOpDrive(0, 0, 0);
                        stateTimer.reset();
                        pathState = 55;
                    } else {
                        follower.setTeleOpDrive(0, 0, alignPower3);
                    }
                }
                break;

            case 55:
                if (shooter.isAtVelocity()) {
                    shooter.shoot();
                    stateTimer.reset();
                    pathState = 63;
                }
                break;

            case 63:
                if (stateTimer.seconds() > 3.0) { // Volley 3 duration
                    shooter.stopFeeding();
                    pathState = 69;
                }
                break;

            case 69:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7, true);
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
        public PathChain Path0, Path1, Path2, Path3, Path4, Path5, Path6, Path7, PathStraight;

        public Paths(Follower follower) {
            Path0 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56.0, 8.0), new Pose(56.0, 14)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(125))
                    .build();

            PathStraight = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(56.0, 14.0), new Pose(56, 14.001)))
                    .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(90))
                    .build();

            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(56, 14.001), new Pose(56, 26)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(56, 26), new Pose(8, 25)))
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
                            new BezierLine(new Pose(9, 54), new Pose(56, 14)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(120))
                    .build();
            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56, 14), new Pose(8, 14)))
                    .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(0))
                    .build();
        }
    }
}