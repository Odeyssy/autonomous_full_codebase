//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import java.util.List;
//
//public class LimelightAligner {
//    private Limelight3A limelight;
//
//    // Constants from your original code
//    private static final int TARGET_TAG_ID_BLUE = 20;
//    private static final int TARGET_TAG_ID_RED = 24;
//    private static final double ALIGNMENT_TOLERANCE = 1.0;
//    private static final double ALIGNMENT_TURN_POWER = 0.3;
//    private static final double ALIGNMENT_SLOW_POWER = 0.15;
//    private static final double ALIGNMENT_SLOW_ZONE = 10.0;
//    private static final double X_OFFSET_ADJUSTMENT = 3.0; // Your custom -3 offset
//
//    public LimelightAligner(HardwareMap hardwareMap) {
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(0);
//        limelight.start();
//    }
//
//    /**
//     * Calculates the turn power needed to align with the target AprilTag.
//     * @param manualTurn The current manual stick input (to return if no tag is found)
//     * @return The power value for the 'turn' variable
//     */
//    public double calculateAlignPower(double manualTurn) {
//        LLResult result = limelight.getLatestResult();
//
//        if (result != null && result.isValid()) {
//            List<FiducialResult> tags = result.getFiducialResults();
//            for (FiducialResult tag : tags) {
//                if (tag.getFiducialId() == TARGET_TAG_ID_BLUE || tag.getFiducialId() == TARGET_TAG_ID_RED) {
//                    double xOffset = tag.getTargetXDegrees() - X_OFFSET_ADJUSTMENT;
//
//                    if (Math.abs(xOffset) < ALIGNMENT_TOLERANCE) return 0;
//
//                    // Simple P-Controller: Power = Error * Gain
//                    // Adjust 0.04 to make it more or less aggressive
//                    double kP = 0.04;
//                    double power = xOffset * kP;
//
//                    // Clamp the power so it doesn't spin too fast
//                    return Math.max(-0.4, Math.min(0.4, power));
//                }
//            }
//        }
//        // CRITICAL: Return a special value like -99 if no tag is found
//        // so your OpMode knows the difference between "Centered" and "Lost"
//        return -99;
//    }
//
//    public void stop() {
//        limelight.stop();
//    }
//}

package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;

public class LimelightAligner {
    private Limelight3A limelight;
    private DcMotor leftFront, leftRear, rightFront, rightRear;

    private static final int TARGET_TAG_ID_BLUE = 20;
    private static final int TARGET_TAG_ID_RED = 24;
    private static final double ALIGNMENT_TOLERANCE = 1.0;
    private static final double X_OFFSET_ADJUSTMENT = 2.5;

    private double kP = 0.5;
    private double minPower = 0.08; // The "kick" power

    public LimelightAligner(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        // Set to RUN_WITHOUT_ENCODER for raw voltage control
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * @return 1 for Aligned, 0 for Searching/Moving, -1 for Target Lost
     */
    public int driveToTarget() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<FiducialResult> tags = result.getFiducialResults();
            for (FiducialResult tag : tags) {
                if (tag.getFiducialId() == TARGET_TAG_ID_BLUE || tag.getFiducialId() == TARGET_TAG_ID_RED) {
                    double xOffset = tag.getTargetXDegrees() - X_OFFSET_ADJUSTMENT;

                    if (Math.abs(xOffset) < ALIGNMENT_TOLERANCE) {
                        stopMotors();
                        return 1; // Aligned
                    }

                    double turnPower = xOffset * kP;
                    if (Math.abs(turnPower) < minPower) turnPower = Math.signum(turnPower) * minPower;
                    turnPower = Math.max(-0.4, Math.min(0.4, turnPower));

                    // Direct Tank-style rotation
                    leftFront.setPower(turnPower);
                    leftRear.setPower(turnPower);
                    rightFront.setPower(-turnPower);
                    rightRear.setPower(-turnPower);
                    return 0; // Moving
                }
            }
        }
        stopMotors();
        return -1; // Lost
    }

    public void stopMotors() {
        leftFront.setPower(0); leftRear.setPower(0);
        rightFront.setPower(0); rightRear.setPower(0);
    }
}