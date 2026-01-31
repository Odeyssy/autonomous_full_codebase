//package org.firstinspires.ftc.teamcode.pedroPathing;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import java.util.List;
//
//@TeleOp(name = "limelightturntoapriltag", group = "Auto")
//public class lightturntoapriltag extends LinearOpMode {
//
//    // Hardware - 4 Mecanum Wheels
//    private Limelight3A limelight;
//    private DcMotor frontLeft;
//    private DcMotor frontRight;
//    private DcMotor backLeft;
//    private DcMotor backRight;
//
//    // Constants - CUSTOMIZE THESE FOR YOUR ROBOT
//    private static final int TARGET_TAG_ID = 20;      // Change to your tag ID
//    private static final double TOLERANCE = 2.0;      // Degrees - how close is "centered"
//    private static final double TURN_POWER = 0.3;     // Motor power for turning
//    private static final double SLOW_TURN_POWER = 0.15; // Slower when close
//    private static final double SLOW_ZONE = 10.0;     // Start slowing down at 10 degrees
//
//    private ElapsedTime runtime = new ElapsedTime();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        // ====================================================================
//        // INITIALIZATION
//        // ====================================================================
//        telemetry.addData("Status", "Initializing...");
//        telemetry.update();
//
//        // Initialize Limelight
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//
//        // Initialize motors - 4 Mecanum Wheels
//        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
//        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
//        backLeft = hardwareMap.get(DcMotor.class, "leftRear");
//        backRight = hardwareMap.get(DcMotor.class, "rightRear");
//
//        // Set motor directions for mecanum drive
//        frontLeft.setDirection(DcMotor.Direction.FORWARD);
//        frontRight.setDirection(DcMotor.Direction.FORWARD);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);
//        backRight.setDirection(DcMotor.Direction.REVERSE);
//
//        // Set motors to brake when power is zero
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // Start Limelight
//        limelight.pipelineSwitch(0);  // Use your AprilTag pipeline
//        limelight.start();
//
//        telemetry.addData("Status", "Ready - Looking for Tag #" + TARGET_TAG_ID);
//        telemetry.addData("Instructions", "Press PLAY to start");
//        telemetry.update();
//
//        // ====================================================================
//        // WAIT FOR START
//        // ====================================================================
//        waitForStart();
//        runtime.reset();
//
//        // ====================================================================
//        // MAIN LOOP - TURN TO CENTER APRILTAG
//        // ====================================================================
//        while (opModeIsActive()) {
//
//            // Get latest vision data from Limelight
//            LLResult result = limelight.getLatestResult();
//
//            if (result != null && result.isValid()) {
//                List<FiducialResult> tags = result.getFiducialResults();
//
//                boolean tagFound = false;
//
//                // Look through all detected tags
//                for (FiducialResult tag : tags) {
//
//                    // Check if this is the tag we want
//                    if (tag.getFiducialId() == TARGET_TAG_ID) {
//                        tagFound = true;
//
//                        // Get horizontal offset from center
//                        // centerX = 0 means perfectly centered
//                        // centerX > 0 means tag is to the RIGHT
//                        // centerX < 0 means tag is to the LEFT
//                        double centerX = tag.getTargetXDegrees();
//
//                        telemetry.addData("Tag Found", "ID: %d", TARGET_TAG_ID);
//                        telemetry.addData("X Offset", "%.2f degrees", centerX);
//
//                        // ================================================
//                        // CHECK IF CENTERED
//                        // ================================================
//                        if (Math.abs(centerX) < TOLERANCE) {
//                            // Tag is centered! Stop motors
//                            stopMotors();
//                            telemetry.addData("Status", "CENTERED!");
//                            telemetry.addData("Final Offset", "%.2f degrees", centerX);
//
//                        } else {
//                            // Tag not centered - need to turn
//
//                            double turnPower;
//
//                            // Use slower speed when getting close
//                            if (Math.abs(centerX) < SLOW_ZONE) {
//                                turnPower = SLOW_TURN_POWER;
//                                telemetry.addData("Status", "Fine tuning...");
//                            } else {
//                                turnPower = TURN_POWER;
//                                telemetry.addData("Status", "Turning...");
//                            }
//
//                            // Turn based on offset direction
//                            if (centerX > 0) {
//                                // Tag is to the RIGHT - turn RIGHT
//                                turnRight(turnPower);
//                                telemetry.addData("Action", "Turning RIGHT");
//                            } else {
//                                // Tag is to the LEFT - turn LEFT
//                                turnLeft(turnPower);
//                                telemetry.addData("Action", "Turning LEFT");
//                            }
//                        }
//
//                        // Display additional tag info
//                        telemetry.addData("Tag Y Offset", "%.2f degrees", tag.getTargetYDegrees());
//
//                        break;  // Found our tag, stop looking through list
//                    }
//                }
//
//                // ====================================================
//                // TAG NOT FOUND - SEARCH FOR IT
//                // ====================================================
//                if (!tagFound) {
//                    telemetry.addData("Status", "Tag #" + TARGET_TAG_ID + " not found");
//                    telemetry.addData("Action", "Searching...");
//                    // Slowly rotate to search for tag
//                    turnRight(SLOW_TURN_POWER);
//                }
//
//            } else {
//                // No valid vision data from Limelight
//                telemetry.addData("Status", "No vision data");
//                telemetry.addData("Action", "Searching...");
//                // Slowly rotate to search for tag
//                turnRight(SLOW_TURN_POWER);
//            }
//
//            // Display runtime
//            telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
//            telemetry.update();
//        }
//
//        // ====================================================================
//        // CLEANUP
//        // ====================================================================
//        stopMotors();
//        limelight.stop();
//        telemetry.addData("Status", "Stopped");
//        telemetry.update();
//    }
//
//    // ========================================================================
//    // HELPER METHODS FOR MOTOR CONTROL - MECANUM 4WD
//    // ========================================================================
//
//    /**
//     * Turn the robot RIGHT (clockwise)
//     * All wheels turn in same rotational direction
//     */
//    private void turnRight(double power) {
//        frontLeft.setPower(power);      // Front left forward
//        frontRight.setPower(-power);    // Front right backward
//        backLeft.setPower(power);       // Back left forward
//        backRight.setPower(-power);     // Back right backward
//    }
//
//    /**
//     * Turn the robot LEFT (counter-clockwise)
//     * All wheels turn in same rotational direction
//     */
//    private void turnLeft(double power) {
//        frontLeft.setPower(-power);     // Front left backward
//        frontRight.setPower(power);     // Front right forward
//        backLeft.setPower(-power);      // Back left backward
//        backRight.setPower(power);      // Back right forward
//    }
//
//    /**
//     * Stop all motors
//     */
//    private void stopMotors() {
//        frontLeft.setPower(0);
//        frontRight.setPower(0);
//        backLeft.setPower(0);
//        backRight.setPower(0);
//    }
//}

package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@TeleOp(name = "Limelight Verification & Turn", group = "Vision")
public class lightturntoapriltag extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Configuration
    private static final int TARGET_TAG_ID = 20;
    private static final double TOLERANCE = 3;      // Degrees
    private static final double TURN_POWER = 0.25;

    private ElapsedTime runtime = new ElapsedTime();
    private long lastResultTimestamp = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // 1. HARDWARE MAPPING
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backLeft = hardwareMap.get(DcMotor.class, "leftRear");
        backRight = hardwareMap.get(DcMotor.class, "rightRear");

        // 2. MOTOR SETUP
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 3. LIMELIGHT STARTUP
        //limelight.setPollRate(10); // Poll at 10Hz or higher
        limelight.pipelineSwitch(0); // Ensure Pipeline 0 is set to AprilTags in Dashboard
        limelight.start();

        telemetry.addData("Status", "Initialized. Looking for Tag %d", TARGET_TAG_ID);
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // GRAB THE LATEST DATA
            LLResult result = limelight.getLatestResult();

            // --- VERIFICATION TELEMETRY ---
            if (result != null) {
                // Check if the frame is "new" by comparing timestamps
                if (result.getTimestamp() != lastResultTimestamp) {
                    telemetry.addData("Vision Feed", "ACTIVE (Heartbeat detected)");
                    lastResultTimestamp = (long) result.getTimestamp();
                } else {
                    telemetry.addData("Vision Feed", "STALE (No new frames)");
                }

                telemetry.addData("Latency", "Capture: %d ms | Parse: %d ms",
                        (int)result.getCaptureLatency(), (int)result.getParseLatency());
            } else {
                telemetry.addData("Vision Feed", "DISCONNECTED (No LLResult object)");
            }

            // --- TARGETING LOGIC ---
            if (result != null && result.isValid()) {
                List<FiducialResult> tags = result.getFiducialResults();
                boolean targetInView = false;

                for (FiducialResult tag : tags) {
                    if (tag.getFiducialId() == TARGET_TAG_ID) {
                        targetInView = true;
                        double xOffset = tag.getTargetXDegrees();

                        telemetry.addData("Targeting", "Tag %d found at %.2f deg", TARGET_TAG_ID, xOffset);

                        if (Math.abs(xOffset) <= TOLERANCE) {
                            stopMotors();
                            telemetry.addData("Action", "STAY (Centered)");
                        } else if (xOffset > 0) {
                            turnRight(TURN_POWER);
                            telemetry.addData("Action", "TURN RIGHT");
                        } else {
                            turnLeft(TURN_POWER);
                            telemetry.addData("Action", "TURN LEFT");
                        }
                    }
                }

                if (!targetInView) {
                    telemetry.addData("Targeting", "Tag %d NOT found in frame", TARGET_TAG_ID);
                    stopMotors();
                }
            } else {
                stopMotors();
            }

            telemetry.update();
        }

        limelight.stop();
    }

    private void turnRight(double p) {
        frontLeft.setPower(-p);
        backLeft.setPower(-p);
        frontRight.setPower(p);
        backRight.setPower(p);
    }

    private void turnLeft(double p) {
        frontLeft.setPower(p);
        backLeft.setPower(p);
        frontRight.setPower(-p);
        backRight.setPower(-p);
    }

    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}