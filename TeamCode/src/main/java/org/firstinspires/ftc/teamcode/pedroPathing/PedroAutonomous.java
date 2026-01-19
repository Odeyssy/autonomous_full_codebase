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
import com.pedropathing.paths.PathBuilder; // Required for explicit typing

@Autonomous(name = "Pedro Points Array Auto", group = "Autonomous")
@Configurable
public class PedroAutonomous extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private PathChain fullAutoChain;

    // 1. Updated with your specific points
    private final Pose[] autoPoints = {
            // Use Math.toRadians() for all heading values!
            new Pose(71.900, 22.816, Math.toRadians(90.0)),
            new Pose(108.0, 36.0, Math.toRadians(90.0)),
            new Pose(72.0, 114.0, Math.toRadians(180.0))
    };
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);

        // Crucial: Set the robot's starting position to the first point in your array
        follower.setStartingPose(autoPoints[0]);

        // Keeps it slow and manageable while you test for oscillations
        follower.setMaxPower(0.3);

        buildPathFromArray();

        pathState = 0;
        panelsTelemetry.debug("Status", "Initialized with 3-Point Array");
        panelsTelemetry.update(telemetry);
    }

    private void buildPathFromArray() {
        // Using PathBuilder explicitly to fix 'var' and 'resolve' errors
        PathBuilder builder = follower.pathBuilder();

        for (int i = 0; i < autoPoints.length - 1; i++) {
            builder.addPath(new BezierLine(autoPoints[i], autoPoints[i+1]))
                    .setLinearHeadingInterpolation(autoPoints[i].getHeading(), autoPoints[i+1].getHeading());
        }

        fullAutoChain = builder.build();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Telemetry for debugging the movement
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading Deg", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Triggers the entire chain sequence
                follower.followPath(fullAutoChain);
                pathState = 1;
                break;
            case 1:
                // Monitors if the robot has finished the entire chain
                if (!follower.isBusy()) {
                    pathState = 2;
                }
                break;
            case 2:
                // Finished - robot will hold position if PID is tuned
                break;
        }
    }
}