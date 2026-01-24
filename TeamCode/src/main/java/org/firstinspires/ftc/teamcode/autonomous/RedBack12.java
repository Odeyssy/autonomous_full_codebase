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

@Autonomous(name = "Red BACK 12 Artifact", group = "Autonomous")
@Configurable
public class RedBack12 extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        // Starting Pose from exported code
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
        panelsTelemetry.debug("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

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
                    follower.followPath(paths.CollectingFirst3, true);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GoBack, true);
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
                    follower.followPath(paths.CollectingSecond3, true);
                    pathState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GoFrontShooting, true);
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
                    follower.followPath(paths.CollectingThird3, true);
                    pathState = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GoFrontShoot, true);
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
        public PathChain ShootPreloaded, ToFirst3, CollectingFirst3, GoBack, ToSecond3,
                CollectingSecond3, GoFrontShooting, ToThird3, CollectingThird3,
                GoFrontShoot, LeavePoints;

        public Paths(Follower follower) {
            ShootPreloaded = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(88.000, 8.000), new Pose(88.000, 8.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(72)).build();

            ToFirst3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(88.000, 8.000), new Pose(102.144, 34.409)))
                    .setLinearHeadingInterpolation(Math.toRadians(72), Math.toRadians(180)).build();

            CollectingFirst3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(102.144, 34.409), new Pose(133.829, 34.409)))
                    .setTangentHeadingInterpolation().setReversed().build();

            GoBack = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(133.829, 34.409), new Pose(88.000, 8.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(72)).build();

            ToSecond3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(88.000, 8.000), new Pose(102.144, 59.177)))
                    .setLinearHeadingInterpolation(Math.toRadians(72), Math.toRadians(180)).setReversed().build();

            CollectingSecond3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(102.144, 59.177), new Pose(133.829, 59.177)))
                    .setTangentHeadingInterpolation().setReversed().build();

            GoFrontShooting = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(133.829, 59.177), new Pose(82.000, 81.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(52)).setReversed().build();

            ToThird3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(82.000, 81.000), new Pose(102.144, 83.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(52), Math.toRadians(180)).setReversed().build();

            CollectingThird3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(102.144, 83.000), new Pose(129.000, 83.000)))
                    .setTangentHeadingInterpolation().setReversed().build();

            GoFrontShoot = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(129.000, 83.000), new Pose(82.000, 81.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(52)).build();

            LeavePoints = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(82.000, 81.000), new Pose(82.481, 30.746)))
                    .setLinearHeadingInterpolation(Math.toRadians(52), Math.toRadians(52)).build();
        }
    }
}
