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

@Autonomous(name = "Red BACK 9 Artifact", group = "Autonomous")
@Configurable
public class RedBack9 extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        // Starting Pose
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

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
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Shoot Preload
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
                    follower.followPath(paths.LeavePoints, true);
                    pathState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    pathState = 9; // Finished
                }
                break;
        }
    }

    public static class Paths {
        public PathChain ShootPreloaded, ToFirst3, CollectFirst3, ShootFirst3,
                ToSecond3, CollectSecond3, ShootSecond3, LeavePoints;

        public Paths(Follower follower) {
            ShootPreloaded = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(88.000, 8.000), new Pose(88.000, 8.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(72)).build();

            ToFirst3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(88.000, 8.000), new Pose(102.144, 34.409)))
                    .setLinearHeadingInterpolation(Math.toRadians(72), Math.toRadians(180)).build();

            CollectFirst3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(102.144, 34.409), new Pose(133.829, 34.409)))
                    .setTangentHeadingInterpolation().setReversed().build();

            ShootFirst3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(133.829, 34.409), new Pose(88.000, 8.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(72)).build();

            ToSecond3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(88.000, 8.000), new Pose(132.227, 7.851)))
                    .setLinearHeadingInterpolation(Math.toRadians(72), Math.toRadians(180)).build();

            CollectSecond3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(132.227, 7.851), new Pose(135.000, 7.851)))
                    .setTangentHeadingInterpolation().setReversed().build();

            ShootSecond3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(135.000, 7.851), new Pose(88.000, 8.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(72)).build();

            LeavePoints = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(88.000, 8.000), new Pose(108.923, 10.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(72), Math.toRadians(180)).build();
        }
    }
}