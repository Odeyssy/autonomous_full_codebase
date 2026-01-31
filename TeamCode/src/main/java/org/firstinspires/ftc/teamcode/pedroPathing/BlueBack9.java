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

@Autonomous(name = "BlueBack9", group = "Autonomous")
@Configurable
public class BlueBack9 extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);

        // Set starting pose to match the first point of movement
        follower.setStartingPose(new Pose(56.000, 8.000, Math.toRadians(90)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Check these on your Driver Station!
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Start first movement immediately
                follower.followPath(paths.Path1, true);
                pathState = 1;
                break;
            case 1: // Wait for Path 1, then move to Path 2
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
            case 6: // Final check
                if (!follower.isBusy()) {
                    pathState = 7; // Finished
                }
                break;
        }
    }

    public static class Paths {
        // Reduced to 6 paths since we removed the 0-length one
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6;

        public Paths(Follower follower) {
            // MOVEMENT 1: (56,8) -> (38.151, 35.382)
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(56.000, 8.000), new Pose(38.151, 35.382)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            // MOVEMENT 2
            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(38.151, 35.382), new Pose(20.785, 35.237)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // MOVEMENT 3
            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(20.785, 35.237), new Pose(55.849, 7.789)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            // MOVEMENT 4
            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(55.849, 7.789), new Pose(9.636, 7.108)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            // MOVEMENT 5
            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(9.636, 7.108), new Pose(56.000, 8.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            // MOVEMENT 6
            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(56.000, 8.000), new Pose(20.000, 10.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();
        }
    }
}