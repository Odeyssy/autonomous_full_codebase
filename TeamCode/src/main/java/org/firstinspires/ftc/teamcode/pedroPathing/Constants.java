package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
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
            .translationalPIDFCoefficients(new PIDFCoefficients(0.067, 0, 0.001, 0.025))
            .headingPIDFCoefficients(new PIDFCoefficients(0.6, 0, 0.001, 0.025))
            .forwardZeroPowerAcceleration(-28.07790227626875)
            .lateralZeroPowerAcceleration(-64.65057988635511)
            .mass(12);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(182/2.54)               // Now interpreted as -5 INCHES
            .strafePodX(196/2.54)               // Now interpreted as -5 INCHES
            .distanceUnit(DistanceUnit.MM) // Switch this to INCH
            .hardwareMapName("odo")
            //.encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .customEncoderResolution(2000/(2 * Math.PI * 24))
            /* CALCULATION:
               19.89437 (ticks/mm) * 25.4 (mm/inch) = 505.317 ticks/inch
               Now apply your FORWARD MULTIPLIER to this number.
            */
            //.customEncoderResolution(505.317)

            /* YAW SCALAR:
               Only use this if your HEADING (degrees) is wrong.
               If the TURN TUNER gave you this, plug it in here.
            */

            .yawScalar(0.99703)

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
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftRear")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
           .xVelocity(65.31821856160802)
          .yVelocity(50.43163245493972);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
