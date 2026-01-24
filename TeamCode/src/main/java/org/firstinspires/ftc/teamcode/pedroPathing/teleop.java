package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name = "teleop")
public class teleop extends LinearOpMode {
    // Drivetrain motors
    private DcMotor rightFront, rightRear, leftFront, leftRear;

    // Intake and shooter motors
    private DcMotorEx IntakeMotor;
    private DcMotorEx ShooterMotor1, ShooterMotor2;
    private DcMotorEx rampmotor;  // New ramp motor

    // Servos
    private CRServo crServofrontR;
    private Servo GateServo;

    // --- Shooter Velocity Variables ---
    private double currentShooterTarget = 1600.0;
    private final double VELOCITY_STEP = 50.0;
    private final double MAX_VELOCITY = 2800.0;

    // --- Intake Velocity Variable ---
    private final double INTAKE_VELOCITY = 2800.0;

    // --- Ramp Motor Velocity ---
    private final double RAMP_VELOCITY = 1500.0;

    // --- Button State Tracking ---
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

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
        crServofrontR = hardwareMap.get(CRServo.class, "crServofrontR");
        GateServo = hardwareMap.get(Servo.class, "GateServo");

        // Set motor directions
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);

        ShooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        ShooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        rampmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rampmotor.setDirection(DcMotorSimple.Direction.FORWARD);

        IntakeMotor.setDirection(DcMotor.Direction.FORWARD);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        crServofrontR.setDirection(DcMotorSimple.Direction.REVERSE);

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

            // Apply constant velocity to shooter motors
            ShooterMotor1.setVelocity(currentShooterTarget);
            ShooterMotor2.setVelocity(currentShooterTarget);

            // Run ramp motor at constant velocity
            rampmotor.setVelocity(RAMP_VELOCITY);

            // --- 2. DRIVETRAIN CONTROL (Gamepad 1) ---
            double forward = -gamepad1.right_stick_y;
            double strafe = gamepad1.right_stick_x;
            double turn = -gamepad1.left_stick_x;

            // Slow mode with left bumper
            if (gamepad1.left_bumper) {
                forward *= 0.5;
                strafe *= 0.5;
                turn *= 0.5;
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
                GateServo.setPosition(0.1);
            }
            else if (gamepad1.x) {
                // SHOOTING MODE
                GateServo.setPosition(0.5);
                rampmotor.setVelocity(RAMP_VELOCITY);
                IntakeMotor.setVelocity(INTAKE_VELOCITY);
                crServofrontR.setPower(-1.0);
            }
            else {
                // --- 4. MANUAL CONTROLS (Gamepad 2) ---
                if (gamepad2.a) GateServo.setPosition(0.5);
                else if (gamepad2.b) GateServo.setPosition(0.1);
                else GateServo.setPosition(0.1);

                double manualIntake = -gamepad2.left_stick_y;

                // Manual intake control
                IntakeMotor.setVelocity(manualIntake * INTAKE_VELOCITY);
                rampmotor.setVelocity(-manualIntake * RAMP_VELOCITY);
                crServofrontR.setPower(-manualIntake);

                double manualShuffle = gamepad2.right_stick_y;
                rampmotor.setVelocity(manualShuffle * RAMP_VELOCITY);
                crServofrontR.setPower(manualShuffle);
            }

            // --- 5. TELEMETRY ---
            telemetry.addData("Target Shooter Velocity", currentShooterTarget);
            telemetry.addData("Actual Shooter Vel", ShooterMotor1.getVelocity());
            telemetry.addData("Ramp Motor Vel", rampmotor.getVelocity());
            telemetry.addData("Intake Velocity", IntakeMotor.getVelocity());
            telemetry.addData("Shooting", gamepad1.x ? "YES" : "NO");
            telemetry.addData("Slow Mode", gamepad1.left_bumper ? "ON" : "OFF");

            telemetry.update();
        }
    }
}