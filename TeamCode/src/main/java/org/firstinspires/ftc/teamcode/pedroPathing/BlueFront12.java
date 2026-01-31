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

@Autonomous(name = "BlueFront12", group = "Autonomous")
@Configurable
public class BlueFront12 extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);

        // Robot starts at (72, 8)
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1, true);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2, true);
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, true);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4, true);
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5, true);
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6, true);
                    pathState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7, true);
                    pathState = 7;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8, true);
                    pathState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9, true);
                    pathState = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path10, true);
                    pathState = 10;
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path11, true);
                    pathState = 11;
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    pathState = 12; // Finished
                }
                break;
        }
        return pathState;
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11;

        public Paths(Follower follower) {
            // FIXED Path 1: Move from start (72, 8) to (56, 8) so it doesn't get stuck
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(72.000, 8.000), new Pose(56.000, 8.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(113))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(56.000, 8.000), new Pose(41.346, 35.718)))
                    .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(0))
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(41.346, 35.718), new Pose(11.796, 35.460)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(11.796, 35.460), new Pose(59.779, 13.605)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(59.779, 13.605), new Pose(40.440, 60.407)))
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(135))
                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(40.440, 60.407), new Pose(13.526, 60.023)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(13.526, 60.023), new Pose(53.579, 89.409)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(53.579, 89.409), new Pose(37.875, 83.650)))
                    .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(37.875, 83.650), new Pose(14.372, 83.977)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(14.372, 83.977), new Pose(53.667, 89.691)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(125))
                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(53.667, 89.691), new Pose(23.992, 100.534)))
                    .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(160))
                    .build();
        }
    }
}