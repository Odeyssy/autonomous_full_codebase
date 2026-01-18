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

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        // This is your starting pose. The first path should begin near here.
        // IMPORTANT: Set this to the start of your first path: new Pose(93.878, 0.796, heading)
        // Set the initial heading as needed, for example 0 radians.
        follower.setStartingPose(new Pose(93.878, 0.796, 0));

        paths = new Paths(follower); // Build paths

        // Initialize the pathState to the first state
        pathState = 0;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading())); // Switched to degrees for readability
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(93.878, 0.796),
                                    new Pose(80.663, 14.785)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Renamed from Path3 in your original to follow sequence
            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(80.663, 14.785),
                                    new Pose(86.122, 25.436)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Renamed from Path2 in your original
            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86.122, 25.436),
                                    new Pose(91.580, 36.088)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Renamed from the first Path5
            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(91.580, 36.088),
                                    new Pose(126.801, 35.624)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            // Renamed from the second Path5
            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(126.801, 35.624),
                                    new Pose(83.624, 10.785)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
        }
    }


    /**
     * This is the state machine for our autonomous path.
     * It follows a sequence of paths defined in the Paths class.
     * @return The next state for the state machine.
     */
    public int autonomousPathUpdate() {
        switch (pathState) {
            // Path 1
            case 0:
                follower.followPath(paths.Path1);
                return 1;
            case 1:
                if (!follower.isBusy()) {
                    return 2;
                }
                break;

            // Path 2
            case 2:
                follower.followPath(paths.Path2);
                return 3;
            case 3:
                if (!follower.isBusy()) {
                    return 4;
                }
                break;

            // Path 3
            case 4:
                follower.followPath(paths.Path3);
                return 5;
            case 5:
                if (!follower.isBusy()) {
                    return 6;
                }
                break;

            // Path 4
            case 6:
                follower.followPath(paths.Path4);
                return 7;
            case 7:
                if (!follower.isBusy()) {
                    return 8;
                }
                break;

            // Path 5
            case 8:
                follower.followPath(paths.Path5);
                return 9;
            case 9:
                if (!follower.isBusy()) {
                    // All paths are complete
                    return 10;
                }
                break;

            // End State
            case 10:
                // Autonomous is finished. Do nothing.
                break;
        }
        // Return the current state by default
        return pathState;
    }
}
