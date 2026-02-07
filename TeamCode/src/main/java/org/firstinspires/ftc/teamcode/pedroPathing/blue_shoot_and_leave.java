package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.IntakeSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.ShooterSystem;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Blue Simple Shoot & Leave", group = "Autonomous")
public class blue_shoot_and_leave extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;

    private IntakeSystem intake;
    private ShooterSystem shooter;
    private ElapsedTime stateTimer = new ElapsedTime();

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);

        intake = new IntakeSystem(hardwareMap);
        shooter = new ShooterSystem(hardwareMap);

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
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Shooter Ready", shooter.isAtVelocity());
        panelsTelemetry.update(telemetry);
    }

    /*
     *  0  -> Start flywheels, drive to (56.181, 18.294)
     *  1  -> Wait for arrival
     *  10 -> Shoot @ (56, 18)
     *  11 -> Wait for volley, then park to (13.056, 12.962)
     * 100 -> Stop everything
     */
    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                shooter.startFlywheels();
                follower.followPath(paths.PathToShoot, true);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    pathState = 10;
                }
                break;

            // ===================== SHOOT @ (56, 18) =====================
            case 10:
                if (shooter.isAtVelocity()) {
                    intake.runIntake();
                    shooter.shoot();
                    stateTimer.reset();
                    pathState = 11;
                } else if (stateTimer.seconds() > 3.0) {
                    intake.runIntake();
                    shooter.shoot();
                    stateTimer.reset();
                    pathState = 11;
                }
                break;

            case 11:
                if (stateTimer.seconds() > 7.0) {
                    shooter.stopFeeding();
                    intake.stopAll();
                    follower.followPath(paths.PathToPark, true);
                    pathState = 100;
                }
                break;

            // ===================== DONE =====================
            case 100:
                if (!follower.isBusy()) {
                    intake.stopAll();
                    shooter.stopAll();
                    pathState = -1;
                }
                break;
        }
    }

    public static class Paths {
        public PathChain PathToShoot, PathToPark;

        public Paths(Follower follower) {
            // Start (56, 8) -> Shooting pos (56.181, 18.294)
            PathToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56.000, 8.000), new Pose(56.181, 15.294)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(118))
                    .build();

            // (56.181, 18.294) -> Park (13.056, 12.962)
            PathToPark = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56.181, 15.294), new Pose(13.056, 12.962)))
                    .setLinearHeadingInterpolation(Math.toRadians(118), Math.toRadians(0))
                    .build();
        }
    }
}