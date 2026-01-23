package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.robotcore.external.Const;

@TeleOp
public class SampleAutoPath extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        //START POSITION_END_POSITION
        // DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_START_SHOOT,
        SHOOT,
        END



    }

    PathState pathState;

    private final Pose startPose = new Pose(71.8996235884567, 22.8155583437892, Math.toRadians(90.0));
    private final Pose endPose = new Pose(72, 72, Math.toRadians(90.0));

    private PathChain driveStartShoot;

    public void buildPaths(){
        driveStartShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setConstraints(new PathConstraints(0.9, 0.9, Math.toRadians(0.2), Math.toRadians(0.2)))
                .setLinearHeadingInterpolation(startPose.getHeading(),endPose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch (pathState){
            case DRIVE_START_SHOOT:
                follower.followPath(driveStartShoot, true);
                setPathState(PathState.SHOOT);
                break;
            case SHOOT:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.25){
                    pathState = PathState.END;
                    follower.breakFollowing();
                }
                break;
            case END:
                follower.breakFollowing();
                break;

            default:
                telemetry.addLine("Path State: " + pathState);
                telemetry.update();
                break;
        }
    }

    public void setPathState(PathState state){
        pathState = state;
        pathTimer.resetTimer();

    }
    @Override
    public void init(){
        pathState = PathState.DRIVE_START_SHOOT;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.3);

        //follower.setPoseErrorConstraint(1.0); // 1 inch of error is okay

    }
    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }
    @Override
    public void loop(){
        follower.update();
        statePathUpdate();

        telemetry.addLine("Path State: " + pathState);
        telemetry.addLine("x: " + follower.getPose().getX());
        telemetry.addLine("y: " + follower.getPose().getY());
        telemetry.update();

    }
}