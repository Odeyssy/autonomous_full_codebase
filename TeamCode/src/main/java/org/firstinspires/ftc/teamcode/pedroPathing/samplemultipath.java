package org.firstinspires.ftc.teamcode;

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

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable
public class samplemultipath extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        // Match the starting point of Path1 for smooth localization
        follower.setStartingPose(new Pose(87.799, 8.000, Math.toRadians(90)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Debugging values
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    /**
     * State machine to sequence through the 9 paths.
     * Each case triggers a path and waits for completion before moving to the next.
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move to Point 1
                follower.followPath(paths.Path1, true);
                pathState = 1;
                break;
            case 1: // Wait for P1, Move to P2
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2, true);
                    pathState = 2;
                }
                break;
            case 2: // Wait for P2, Move to P3
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, true);
                    pathState = 3;
                }
                break;
            case 3: // Wait for P3, Move to P4
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4, true);
                    pathState = 4;
                }
                break;
            case 4: // Wait for P4, Move to P5
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5, true);
                    pathState = 5;
                }
                break;
            case 5: // Wait for P5, Move to P6
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6, true);
                    pathState = 6;
                }
                break;
            case 6: // Wait for P6, Move to P7
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7, true);
                    pathState = 7;
                }
                break;
            case 7: // Wait for P7, Move to P8
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8, true);
                    pathState = 8;
                }
                break;
            case 8: // Wait for P8, Move to P9
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9, true);
                    pathState = 9;
                }
                break;
            case 9: // Final completion check
                if (!follower.isBusy()) {
                    pathState = 10; // Final state / Idle
                }
                break;
        }
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(87.799, 8.000), new Pose(103.880, 34.735)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180)).build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(103.880, 34.735), new Pose(127.041, 34.610)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(127.041, 34.610), new Pose(81.295, 13.439)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115)).build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(81.295, 13.439), new Pose(104.911, 58.708)))
                    .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180)).build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(104.911, 58.708), new Pose(129.898, 59.163)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(129.898, 59.163), new Pose(78.597, 16.437)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90)).build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(78.597, 16.437), new Pose(91.547, 82.408)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180)).build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(91.547, 82.408), new Pose(120.212, 82.858)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(120.212, 82.858), new Pose(75.690, 18.459)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90)).build();
        }
    }
}