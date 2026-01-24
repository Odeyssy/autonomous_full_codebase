package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red FRONT 12", group = "Autonomous")
@Configurable // Panels
public class RedFront12 extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        // Updated starting pose as per retrofit requirements
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    /**
     * State machine to sequence through the paths.
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.ShootPreloaded, true);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ToFirst3, true);
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.CollectFirst3, true);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ShootFirst3, true);
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ToSecond3, true);
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.CollectSecond3, true);
                    pathState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ShootSecond3, true);
                    pathState = 7;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ToThird3, true);
                    pathState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.CollectThird3, true);
                    pathState = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ShootThird3, true);
                    pathState = 10;
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.LeavePoints, true);
                    pathState = 11;
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    pathState = 12; // Finished
                }
                break;
        }
    }

    public static class Paths {
        public PathChain ShootPreloaded, ToFirst3, CollectFirst3, ShootFirst3,
                ToSecond3, CollectSecond3, ShootSecond3,
                ToThird3, CollectThird3, ShootThird3, LeavePoints;

        public Paths(Follower follower) {
            ShootPreloaded = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(123.000, 123.000), new Pose(82.000, 81.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(217), Math.toRadians(45)).build();

            ToFirst3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(82.000, 81.000), new Pose(109.835, 83.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180)).build();

            CollectFirst3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(109.835, 83.000), new Pose(128.000, 83.000)))
                    .setTangentHeadingInterpolation().setReversed().build();

            ShootFirst3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(128.000, 83.000), new Pose(82.000, 81.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45)).build();

            ToSecond3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(82.000, 81.000), new Pose(109.304, 59.177)))
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180)).build();

            CollectSecond3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(109.304, 59.177), new Pose(130.647, 58.912)))
                    .setTangentHeadingInterpolation().setReversed().build();

            ShootSecond3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(130.647, 58.912), new Pose(82.000, 81.265)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45)).build();

            ToThird3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(82.000, 81.265), new Pose(109.039, 35.205)))
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180)).build();

            CollectThird3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(109.039, 35.205), new Pose(130.912, 34.674)))
                    .setTangentHeadingInterpolation().setReversed().build();

            ShootThird3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(130.912, 34.674), new Pose(88.000, 8.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(72)).build();

            LeavePoints = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(88.000, 8.000), new Pose(108.923, 10.000)))
                    .setTangentHeadingInterpolation().build();
        }
    }
}