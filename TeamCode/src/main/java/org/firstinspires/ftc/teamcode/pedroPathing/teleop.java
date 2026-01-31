package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;   // <<< KEPT (only for shooter)
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@TeleOp(name = "teleop")
public class teleop extends LinearOpMode {
    // Drivetrain motors
    private DcMotor rightFront, rightRear, leftFront, leftRear;

    // Intake and shooter motors
    private DcMotorEx IntakeMotor;
    private DcMotorEx ShooterMotor1, ShooterMotor2;
    private DcMotorEx rampmotor;

    // Servos
    private CRServo crServofrontR;
    private CRServo crServobackR;
    private Servo GateServo;

    // Limelight vision
    private Limelight3A limelight;

    // --- April Tag Alignment Constants ---
    private static final int TARGET_TAG_ID_BLUE = 20;
    private static final int TARGET_TAG_ID_RED = 24;
    private static final double ALIGNMENT_TOLERANCE = 1;  // Degrees
    private static final double ALIGNMENT_TURN_POWER = 0.3;
    private static final double ALIGNMENT_SLOW_POWER = 0.15;
    private static final double ALIGNMENT_SLOW_ZONE = 10.0;

    // --- Shooter Velocity Variables ---
    private double currentShooterTarget = 1600.0;   // ticks/sec; tuned by dpad
    private final double VELOCITY_STEP = 50.0;
    private final double MAX_VELOCITY = 2800.0;

    // --- Intake Velocity Variable ---
    private final double INTAKE_VELOCITY = 5000.0;  // ticks/sec

    // --- Ramp Motor Velocity ---
    private final double RAMP_VELOCITY = 850.0;     // ticks/sec

    // --- Button State Tracking ---
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    // --- Shooter PIDF constants ---
    private static final double SHOOTER_TICKS_PER_REV = 28.0;   // <<< SHOOTER ONLY
    private static final double SHOOTER_TOLERANCE = 75.0;       // <<< SHOOTER ONLY

    @Override
    public void runOpMode() {
        // --- HARDWARE MAPPING ---
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        ShooterMotor1 = hardwareMap.get(DcMotorEx.class, "ShooterMotor1");
        ShooterMotor2 = hardwareMap.get(DcMotorEx.class, "ShooterMotor2");
        rampmotor = hardwareMap.get(DcMotorEx.class, "rampmotor");
        rampmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        crServofrontR = hardwareMap.get(CRServo.class, "crServofrontR");
        crServobackR = hardwareMap.get(CRServo.class, "crServobackR");
        GateServo = hardwareMap.get(Servo.class, "GateServo");

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Set motor directions
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);

        // --- SHOOTER PIDF SETUP ONLY ---  // <<< SHOOTER ONLY
        ShooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ShooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        ShooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Configure shooter PIDF (P + default F) [web:15][web:21]
        PIDFCoefficients shooterBase = ShooterMotor1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients shooterPIDF = new PIDFCoefficients(
                15.0,          // P  (tune on bot)
                0.0,           // I
                0.0,           // D
                shooterBase.f  // F from firmware
        );
        ShooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        ShooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

        // Intake and ramp stay simple velocity mode (no custom PIDF)  // <<< CHANGED
        rampmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rampmotor.setDirection(DcMotorSimple.Direction.REVERSE);

        IntakeMotor.setDirection(DcMotor.Direction.FORWARD);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        crServofrontR.setDirection(DcMotorSimple.Direction.REVERSE);
        crServobackR.setDirection(DcMotorSimple.Direction.FORWARD);

        // Start Limelight
        limelight.pipelineSwitch(0);  // Make sure pipeline 0 is set to AprilTags
        limelight.start();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- 1. SHOOTER VELOCITY TUNING (Gamepad 2) ---
            if (gamepad2.dpad_up && !dpadUpPressed) {
                currentShooterTarget += VELOCITY_STEP;
            }
            if (gamepad2.dpad_down && !dpadDownPressed) {
                currentShooterTarget -= VELOCITY_STEP;
            }
            dpadUpPressed = gamepad2.dpad_up;
            dpadDownPressed = gamepad2.dpad_down;

            // Keep velocity within safe bounds
            currentShooterTarget = Range.clip(currentShooterTarget, 0, MAX_VELOCITY);

            // Apply target velocity to shooter motors (ticks/sec)
            ShooterMotor1.setVelocity(currentShooterTarget);
            ShooterMotor2.setVelocity(currentShooterTarget);

            // Compute "at speed" flag for shooter wheels only  // <<< SHOOTER ONLY
            double shooterVel1 = ShooterMotor1.getVelocity();
            double shooterVel2 = ShooterMotor2.getVelocity();
            boolean shooterAtSpeed =
                    Math.abs(shooterVel1 - currentShooterTarget) < SHOOTER_TOLERANCE &&
                            Math.abs(shooterVel2 - currentShooterTarget) < SHOOTER_TOLERANCE;

            // Run ramp motor at base velocity (0 = stopped by default)
            rampmotor.setVelocity(0);

            // --- 2. DRIVETRAIN CONTROL ---
            double forward = gamepad1.right_stick_y;
            double strafe = -gamepad1.right_stick_x;
            double turn = -gamepad1.left_stick_x;

            // Check if alignment mode is active (left bumper held)
            if (gamepad1.left_bumper) {
                // AUTO-ALIGNMENT MODE
                LLResult result = limelight.getLatestResult();
                boolean aligned = false;

                if (result != null && result.isValid()) {
                    List<FiducialResult> tags = result.getFiducialResults();

                    for (FiducialResult tag : tags) {
                        if (tag.getFiducialId() == TARGET_TAG_ID_BLUE || tag.getFiducialId() == TARGET_TAG_ID_RED) {
                            double xOffset = tag.getTargetXDegrees() - 3;

                            telemetry.addData("Alignment", "Tag Found - Offset: %.2f deg", xOffset);

                            if (Math.abs(xOffset) < ALIGNMENT_TOLERANCE) {
                                // CENTERED!
                                turn = 0;
                                aligned = true;
                                telemetry.addData("Status", "ALIGNED!");
                            } else {
                                // Calculate turn power based on distance
                                double turnPower;
                                if (Math.abs(xOffset) < ALIGNMENT_SLOW_ZONE) {
                                    turnPower = ALIGNMENT_SLOW_POWER;
                                    telemetry.addData("Status", "Fine tuning...");
                                } else {
                                    turnPower = ALIGNMENT_TURN_POWER;
                                    telemetry.addData("Status", "Aligning...");
                                }

                                // Override turn input for alignment
                                if (xOffset > 0) {
                                    turn = -turnPower;  // Turn right
                                } else {
                                    turn = turnPower; // Turn left
                                }
                            }
                            break;
                        }
                    }

                    if (!aligned && turn == -gamepad1.left_stick_x) {
                        telemetry.addData("Alignment", "Tag #%d not found");
                    }
                } else {
                    telemetry.addData("Alignment", "No vision data");
                }
            }

            // Calculate motor powers for mecanum drive
            double denominator = Math.max(1, Math.abs(forward) + Math.abs(strafe) + Math.abs(turn));
            leftFront.setPower((forward + strafe + turn) / denominator);
            leftRear.setPower((forward - strafe + turn) / denominator);
            rightFront.setPower((forward - strafe - turn) / denominator);
            rightRear.setPower((forward + strafe - turn) / denominator);

            // --- 3. GAMEPAD 1 SPECIAL BUTTONS ---
            if (gamepad1.b) {
                // STOP ALL
                IntakeMotor.setVelocity(0);
                rampmotor.setVelocity(0);
                crServofrontR.setPower(0);
                crServobackR.setPower(0);
                GateServo.setPosition(0.1);
            } else if (gamepad1.x) {
                // SHOOTING MODE - only feed when shooter wheels at speed  // <<< SHOOTER ONLY
                if (shooterAtSpeed) {
                    rampmotor.setVelocity(-RAMP_VELOCITY);
                    IntakeMotor.setVelocity(INTAKE_VELOCITY);
                    GateServo.setPosition(0.5);
                    crServofrontR.setPower(-1.0);
                    crServobackR.setPower(1.0);
                } else {
                    // spin shooter up but don't feed aggressively
                    rampmotor.setVelocity(0);
                    IntakeMotor.setVelocity(0);
                    GateServo.setPosition(0.1);
                    crServofrontR.setPower(0);
                    crServobackR.setPower(0);
                }
            } else {
                // --- 4. MANUAL CONTROLS (Gamepad 2) ---
                if (gamepad2.a) GateServo.setPosition(0.5);
                else if (gamepad2.b) GateServo.setPosition(0.1);
                else GateServo.setPosition(0.1);

                double manualIntake = -gamepad2.left_stick_y;

                // Manual intake control (simple velocity)
                IntakeMotor.setVelocity(manualIntake * INTAKE_VELOCITY);

                double manualShuffle = gamepad2.right_stick_y;
                rampmotor.setVelocity(manualShuffle * RAMP_VELOCITY);
                crServofrontR.setPower(manualShuffle);
                crServobackR.setPower(-manualShuffle);
            }

            // --- 5. TELEMETRY ---
            telemetry.addData("Target Shooter Vel (t/s)", currentShooterTarget);
            telemetry.addData("Shooter1 Vel (t/s)", shooterVel1);
            telemetry.addData("Shooter2 Vel (t/s)", shooterVel2);
            telemetry.addData("Shooter At Speed", shooterAtSpeed);
            telemetry.addData("Ramp Motor Vel", rampmotor.getVelocity());
            telemetry.addData("Intake Velocity", IntakeMotor.getVelocity());
            telemetry.addData("Shooting", gamepad1.x ? "YES" : "NO");
            telemetry.addData("Alignment Mode", gamepad1.left_bumper ? "ACTIVE" : "OFF");

            telemetry.update();
        }

        // Cleanup
        limelight.stop();
    }
}