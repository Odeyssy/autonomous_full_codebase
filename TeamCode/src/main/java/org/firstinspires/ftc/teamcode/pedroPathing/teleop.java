package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@Config  // Makes variables tunable in Dashboard
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
    private static final double ALIGNMENT_TOLERANCE = 1;
    private static final double ALIGNMENT_TURN_POWER = 0.3;
    private static final double ALIGNMENT_SLOW_POWER = 0.15;
    private static final double ALIGNMENT_SLOW_ZONE = 10.0;

    // === DASHBOARD TUNABLE PID VALUES ===
    public static double SHOOTER_P = 40.0;
    public static double SHOOTER_I = 0.0;
    public static double SHOOTER_D = -4.5;
    public static double SHOOTER_F = 12.5;  // Auto-calculates if 0

    // === DASHBOARD TUNABLE VELOCITIES ===
    public static double SHOOTER_TOLERANCE = 75.0;
    public static double INTAKE_VELOCITY = 5000.0;
    public static double RAMP_VELOCITY = 850.0;

    // --- Shooter Velocity Control (Gamepad 2 D-pad) ---
    private double currentShooterTarget = 1600.0;  // Starting velocity
    private final double VELOCITY_STEP = 50.0;     // Increment per button press
    private final double MIN_VELOCITY = 1400.0;    // Minimum velocity
    private final double MAX_VELOCITY = 2100.0;    // Maximum velocity

    // --- Button State Tracking ---
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    // Dashboard instance
    private FtcDashboard dashboard;

    // Track PID changes
    private double lastP = SHOOTER_P;
    private double lastI = SHOOTER_I;
    private double lastD = SHOOTER_D;
    private double lastF = SHOOTER_F;

    @Override
    public void runOpMode() {
        // --- INITIALIZE DASHBOARD ---
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

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

        // Start Limelight camera stream on Dashboard
     //   dashboard.startCameraStream(limelight, 30);

        // Set motor directions
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);

        // --- SHOOTER PIDF SETUP ---
        ShooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ShooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        ShooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initial PID setup from Dashboard values
        updateShooterPID();

        // Intake and ramp stay simple velocity mode
        rampmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rampmotor.setDirection(DcMotorSimple.Direction.REVERSE);

        IntakeMotor.setDirection(DcMotor.Direction.FORWARD);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        crServofrontR.setDirection(DcMotorSimple.Direction.REVERSE);
        crServobackR.setDirection(DcMotorSimple.Direction.FORWARD);

        // Start Limelight
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData("Status", "Initialized - Dashboard Connected!");
        telemetry.addData("Info", "Gamepad2 D-pad Up/Down adjusts shooter speed");
        telemetry.addData("Range", "%.0f - %.0f t/s (50 t/s steps)", MIN_VELOCITY, MAX_VELOCITY);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- CHECK FOR PID CHANGES ---
            if (pidValuesChanged()) {
                updateShooterPID();
            }

            // --- SHOOTER VELOCITY ADJUSTMENT (Gamepad 2 D-pad) ---
            if (gamepad2.dpad_up && !dpadUpPressed) {
                currentShooterTarget += VELOCITY_STEP;
                currentShooterTarget = Range.clip(currentShooterTarget, MIN_VELOCITY, MAX_VELOCITY);
            }
            if (gamepad2.dpad_down && !dpadDownPressed) {
                currentShooterTarget -= VELOCITY_STEP;
                currentShooterTarget = Range.clip(currentShooterTarget, MIN_VELOCITY, MAX_VELOCITY);
            }
            dpadUpPressed = gamepad2.dpad_up;
            dpadDownPressed = gamepad2.dpad_down;

            // Apply target velocity to shooter motors
            ShooterMotor1.setVelocity(currentShooterTarget);
            ShooterMotor2.setVelocity(currentShooterTarget);

            // Compute "at speed" flag using Dashboard tolerance
            double shooterVel1 = ShooterMotor1.getVelocity();
            double shooterVel2 = ShooterMotor2.getVelocity();
            boolean shooterAtSpeed =
                    Math.abs(shooterVel1 - currentShooterTarget) < SHOOTER_TOLERANCE &&
                            Math.abs(shooterVel2 - currentShooterTarget) < SHOOTER_TOLERANCE;

            // Run ramp motor at base velocity
            rampmotor.setVelocity(0);

            // --- DRIVETRAIN CONTROL ---
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
                            double xOffset = tag.getTargetXDegrees() - 2;

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
                                    turn = -turnPower;
                                } else {
                                    turn = turnPower;
                                }
                            }
                            break;
                        }
                    }

                    if (!aligned && turn == -gamepad1.left_stick_x) {
                        telemetry.addData("Alignment", "Tag not found");
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

            // --- GAMEPAD 1 SPECIAL BUTTONS ---
            if (gamepad1.b) {
                // STOP ALL
                IntakeMotor.setVelocity(0);
                rampmotor.setVelocity(0);
                crServofrontR.setPower(0);
                crServobackR.setPower(0);
                GateServo.setPosition(0.1);
            } else if (gamepad1.x) {
                // SHOOTING MODE - uses Dashboard velocities
                if (shooterAtSpeed) {
                    rampmotor.setVelocity(-RAMP_VELOCITY);
                    IntakeMotor.setVelocity(INTAKE_VELOCITY);
                    GateServo.setPosition(0.5);
                    crServofrontR.setPower(-1.0);
                    crServobackR.setPower(1.0);
                } else {
                    rampmotor.setVelocity(0);
                    IntakeMotor.setVelocity(0);
                    GateServo.setPosition(0.1);
                    crServofrontR.setPower(0);
                    crServobackR.setPower(0);
                }
            } else {
                // --- MANUAL CONTROLS (Gamepad 2) ---
                if (gamepad2.a) GateServo.setPosition(0.5);
                else if (gamepad2.b) GateServo.setPosition(0.1);
                else GateServo.setPosition(0.1);

                double manualIntake = -gamepad2.left_stick_y;
                IntakeMotor.setVelocity(manualIntake * INTAKE_VELOCITY);

                double manualShuffle = gamepad2.right_stick_y;
                rampmotor.setVelocity(manualShuffle * RAMP_VELOCITY);
                crServofrontR.setPower(manualShuffle);
                crServobackR.setPower(-manualShuffle);
            }

            // --- ENHANCED TELEMETRY ---
            telemetry.addData("=== PID VALUES ===", "");
            telemetry.addData("P", "%.2f", SHOOTER_P);
            telemetry.addData("I", "%.2f", SHOOTER_I);
            telemetry.addData("D", "%.2f", SHOOTER_D);
            telemetry.addData("F", "%.2f", SHOOTER_F);
            telemetry.addData("=== SHOOTER ===", "");
            telemetry.addData("Target Vel (t/s)", "%.0f", currentShooterTarget);
            telemetry.addData("Motor1 Vel (t/s)", "%.0f", shooterVel1);
            telemetry.addData("Motor2 Vel (t/s)", "%.0f", shooterVel2);
            telemetry.addData("Error1", "%.0f", Math.abs(shooterVel1 - currentShooterTarget));
            telemetry.addData("Error2", "%.0f", Math.abs(shooterVel2 - currentShooterTarget));
            telemetry.addData("At Speed", shooterAtSpeed ? "✓ YES" : "✗ NO");
            telemetry.addData("=== CONTROLS ===", "");
            telemetry.addData("GP2 D-pad Up", "Increase Speed (+50)");
            telemetry.addData("GP2 D-pad Down", "Decrease Speed (-50)");
            telemetry.addData("=== OTHER ===", "");
            telemetry.addData("Ramp Vel", "%.0f", rampmotor.getVelocity());
            telemetry.addData("Intake Vel", "%.0f", IntakeMotor.getVelocity());
            telemetry.addData("Shooting", gamepad1.x ? "YES" : "NO");
            telemetry.addData("Alignment", gamepad1.left_bumper ? "ACTIVE" : "OFF");

            telemetry.update();
        }

        // Cleanup
        limelight.stop();
    }

    // === HELPER METHODS FOR PID TUNING ===

    /**
     * Checks if any PID values changed in Dashboard
     */
    private boolean pidValuesChanged() {
        return lastP != SHOOTER_P ||
                lastI != SHOOTER_I ||
                lastD != SHOOTER_D ||
                lastF != SHOOTER_F;
    }

    /**
     * Updates shooter motor PID coefficients from Dashboard values
     */
    private void updateShooterPID() {
        // Auto-calculate F if set to 0
        double feedforward = SHOOTER_F;
        if (feedforward == 0.0) {
            feedforward = ShooterMotor1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f;
        }

        PIDFCoefficients newPIDF = new PIDFCoefficients(
                SHOOTER_P,
                SHOOTER_I,
                SHOOTER_D,
                feedforward
        );

        ShooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);
        ShooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);

        // Update tracking variables
        lastP = SHOOTER_P;
        lastI = SHOOTER_I;
        lastD = SHOOTER_D;
        lastF = SHOOTER_F;

        telemetry.addData("PID Updated!", "P=%.1f I=%.1f D=%.1f", SHOOTER_P, SHOOTER_I, SHOOTER_D);
    }
}