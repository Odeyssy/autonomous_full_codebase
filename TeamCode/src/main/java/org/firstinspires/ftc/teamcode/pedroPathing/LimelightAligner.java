package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;

public class LimelightAligner {
    private Limelight3A limelight;

    // Constants from your original code
    private static final int TARGET_TAG_ID_BLUE = 20;
    private static final int TARGET_TAG_ID_RED = 24;
    private static final double ALIGNMENT_TOLERANCE = 1.0;
    private static final double ALIGNMENT_TURN_POWER = 0.3;
    private static final double ALIGNMENT_SLOW_POWER = 0.15;
    private static final double ALIGNMENT_SLOW_ZONE = 10.0;
    private static final double X_OFFSET_ADJUSTMENT = 3.0; // Your custom -3 offset

    public LimelightAligner(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    /**
     * Calculates the turn power needed to align with the target AprilTag.
     * @param manualTurn The current manual stick input (to return if no tag is found)
     * @return The power value for the 'turn' variable
     */
    public double calculateAlignPower(double manualTurn) {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<FiducialResult> tags = result.getFiducialResults();

            for (FiducialResult tag : tags) {
                if (tag.getFiducialId() == TARGET_TAG_ID_BLUE || tag.getFiducialId() == TARGET_TAG_ID_RED) {
                    double xOffset = tag.getTargetXDegrees() - X_OFFSET_ADJUSTMENT;

                    if (Math.abs(xOffset) < ALIGNMENT_TOLERANCE) {
                        return 0; // Centered
                    }

                    // Determine power based on distance
                    double p = (Math.abs(xOffset) < ALIGNMENT_SLOW_ZONE) ? ALIGNMENT_SLOW_POWER : ALIGNMENT_TURN_POWER;

                    // Return positive or negative power based on offset direction
                    return (xOffset > 0) ? -p : p;
                }
            }
        }
        return manualTurn; // If no tag found, don't override the driver
    }

    public void stop() {
        limelight.stop();
    }
}