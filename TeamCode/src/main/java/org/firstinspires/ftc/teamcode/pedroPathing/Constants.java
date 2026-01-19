package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .translationalPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.05, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.01, 0))
            .centripetalScaling(0.005)
            .mass(12);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5)
            .strafePodX(5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

//    public static PinpointConstants localizerConstants = new PinpointConstants()
//            /* TRANSFORMATION:
//               Move your physical forward offset to the Strafe constant
//               and your physical strafe offset to the Forward constant.
//            */
//            .forwardPodY(0.5)   // This was your physical 'strafe' offset
//            .strafePodX(-5.0)   // This was your physical 'forward' offset
//
//            .distanceUnit(DistanceUnit.INCH)
//            .hardwareMapName("odo")
//            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
//
//            /* DIRECTION CORRECTION:
//               We need to ensure that when you push the bot toward the 'top' of the
//               visualizer, Y increases, and toward the 'left', X decreases.
//            */
//            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
//            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.5)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftRear")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static PathConstraints pathConstraints = new PathConstraints(30.0, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
