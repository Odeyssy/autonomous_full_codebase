package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "teleop")
public class teleop extends LinearOpMode {

    private DcMotor frontRight, backRight, frontLeft, backLeft;
    private DcMotor IntakeMotorLeft, IntakeMotorRight;
    private DcMotorEx RampMotor1, RampMotor2;
    private CRServo crServobackL, crServofrontR;
    private Servo GateServo;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    final double TURN_GAIN  = 0.003;
    final double TURN_TOLERANCE_PIXELS = 10.0;
    final double MAX_TURN_SPEED = 0.4;

    // --- Ramp Velocity Variables ---
    private double currentRampTarget = 1600.0;
    private final double VELOCITY_STEP = 50.0;
    private final double MAX_VELOCITY = 2800.0;

    // --- Button State Tracking (Prevent rapid-fire incrementing) ---
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    @Override
    public void runOpMode() {
        // --- HARDWARE MAPPING ---
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeMotorLeft = hardwareMap.get(DcMotor.class, "IntakeMotorLeft");
        IntakeMotorRight = hardwareMap.get(DcMotor.class, "IntakeMotorRight");
        RampMotor1 = hardwareMap.get(DcMotorEx.class, "RampMotor1");
        RampMotor2 = hardwareMap.get(DcMotorEx.class, "RampMotor2");
        crServobackL = hardwareMap.get(CRServo.class, "crServobackL");
        crServofrontR = hardwareMap.get(CRServo.class, "crServofrontR");
        GateServo = hardwareMap.get(Servo.class, "GateServo");


        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        RampMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RampMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RampMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        RampMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        IntakeMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        crServobackL.setDirection(DcMotorSimple.Direction.FORWARD);
        crServofrontR.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            // --- 1. RAMP VELOCITY TUNING (Gamepad 2) ---
            if (gamepad2.dpad_up && !dpadUpPressed) {
                currentRampTarget += VELOCITY_STEP;
            }
            if (gamepad2.dpad_down && !dpadDownPressed) {
                currentRampTarget -= VELOCITY_STEP;
            }
            dpadUpPressed = gamepad2.dpad_up;
            dpadDownPressed = gamepad2.dpad_down;

            // Keep velocity within safe bounds
            currentRampTarget = Range.clip(currentRampTarget, 0, MAX_VELOCITY);

            // Apply constant velocity
            RampMotor1.setVelocity(currentRampTarget);
            RampMotor2.setVelocity(currentRampTarget);

            // --- 2. VISION DETECTION ---
            boolean tagFound = false;
            AprilTagDetection targetTag = null;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            if (!currentDetections.isEmpty()) {
                targetTag = currentDetections.get(0);
                tagFound = true;
            }

            // --- 3. DRIVETRAIN & ALIGNMENT (Gamepad 1) ---
            double forward = -gamepad1.right_stick_y;
            double strafe = gamepad1.right_stick_x;
            double turn;

            if (gamepad1.left_trigger > 0.1 && tagFound) {
                double errorX = 350 - targetTag.center.x;
                turn = (Math.abs(errorX) > TURN_TOLERANCE_PIXELS) ?
                        Range.clip(errorX * TURN_GAIN, -MAX_TURN_SPEED, MAX_TURN_SPEED) : 0;
            } else {
                turn = -gamepad1.left_stick_x;
                if (gamepad1.left_bumper) { forward *= 0.5; strafe *= 0.5; turn *= 0.5; }
            }

            double denominator = Math.max(1, Math.abs(forward) + Math.abs(strafe) + Math.abs(turn));
            frontLeft.setPower((forward + strafe + turn) / denominator);
            backLeft.setPower((forward - strafe + turn) / denominator);
            frontRight.setPower((forward - strafe - turn) / denominator);
            backRight.setPower((forward + strafe - turn) / denominator);

            // --- 4. GAMEPAD 1 SPECIAL BUTTONS ---
            if (gamepad1.b) {
                IntakeMotorLeft.setPower(0);
                IntakeMotorRight.setPower(0);
                crServobackL.setPower(0);
                crServofrontR.setPower(0);
                GateServo.setPosition(0.1);
            }
            else if (gamepad1.x) {
                GateServo.setPosition(0.5);
                IntakeMotorLeft.setPower(1.0);
                IntakeMotorRight.setPower(1.0);
                crServobackL.setPower(-1.0);
                crServofrontR.setPower(-1.0);
            }
            else {
                // --- 5. MANUAL CONTROLS (Gamepad 2) ---
                if (gamepad2.a) GateServo.setPosition(0.5);
                else if (gamepad2.b) GateServo.setPosition(0.1);
                else GateServo.setPosition(0.1);

                double manualIntake = -gamepad2.left_stick_y;
                IntakeMotorLeft.setPower(manualIntake);
                IntakeMotorRight.setPower(manualIntake);
                crServobackL.setPower(-manualIntake);
                crServofrontR.setPower(-manualIntake);

                double manualShuffle = gamepad2.right_stick_y;
                crServobackL.setPower(manualShuffle);
                crServofrontR.setPower(manualShuffle);
            }

            // --- 6. TELEMETRY ---
            telemetry.addData("Target Ramp Velocity", currentRampTarget);
            telemetry.addData("Actual Ramp Vel", RampMotor1.getVelocity());
            telemetry.addData("Shooting", gamepad1.x ? "YES" : "NO");

            telemetry.update();
        }
        visionPortal.close();
    }
}