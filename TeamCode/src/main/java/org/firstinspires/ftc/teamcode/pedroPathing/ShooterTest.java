
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config

   // Makes variables tunable in Dashboard
@TeleOp(name = "Shooter Test Only")

public class ShooterTest extends LinearOpMode {

    // Shooter motors
    private DcMotorEx ShooterMotor1;
    private DcMotorEx ShooterMotor2;

    // === DASHBOARD TUNABLE PID VALUES ===
    public static double SHOOTER_P = 15.0;
    public static double SHOOTER_I = 0.0;
    public static double SHOOTER_D = 0.0;
    public static double SHOOTER_F = 0.0;  // Auto-calculates if 0

    // === DASHBOARD TUNABLE VELOCITIES ===
    public static double SHOOTER_TARGET_VEL = 1600.0;
    public static double SHOOTER_TOLERANCE = 75.0;

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
        ShooterMotor1 = hardwareMap.get(DcMotorEx.class, "ShooterMotor1");
        ShooterMotor2 = hardwareMap.get(DcMotorEx.class, "ShooterMotor2");

        // --- SHOOTER SETUP ---
        ShooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ShooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        ShooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initial PID setup from Dashboard values
        updateShooterPID();

        telemetry.addData("Status", "Shooter Test - Dashboard Connected!");
        telemetry.addData("Info", "Tune PID at http://192.168.43.1:8080/dash");
        telemetry.addData("Controls", "A = Start Shooter | B = Stop Shooter");
        telemetry.update();

        waitForStart();

        boolean shooterRunning = false;

        while (opModeIsActive()) {

            // --- CHECK FOR PID CHANGES ---
            if (pidValuesChanged()) {
                updateShooterPID();
            }

            // --- SIMPLE CONTROLS ---
            if (gamepad1.a) {
                // Start shooter
                shooterRunning = true;
            } else if (gamepad1.b) {
                // Stop shooter
                shooterRunning = false;
            }

            // Apply velocity
            if (shooterRunning) {
                ShooterMotor1.setVelocity(SHOOTER_TARGET_VEL);
                ShooterMotor2.setVelocity(SHOOTER_TARGET_VEL);
            } else {
                ShooterMotor1.setVelocity(0);
                ShooterMotor2.setVelocity(0);
            }

            // Get current velocities
            double shooterVel1 = ShooterMotor1.getVelocity();
            double shooterVel2 = ShooterMotor2.getVelocity();

            // Check if at speed
            boolean shooterAtSpeed =
                    Math.abs(shooterVel1 - SHOOTER_TARGET_VEL) < SHOOTER_TOLERANCE &&
                            Math.abs(shooterVel2 - SHOOTER_TARGET_VEL) < SHOOTER_TOLERANCE;

            // --- TELEMETRY ---
            telemetry.addData("=== CONTROLS ===", "");
            telemetry.addData("A Button", "Start Shooter");
            telemetry.addData("B Button", "Stop Shooter");
            telemetry.addData("", "");
            telemetry.addData("=== PID VALUES ===", "");
            telemetry.addData("P", "%.2f", SHOOTER_P);
            telemetry.addData("I", "%.2f", SHOOTER_I);
            telemetry.addData("D", "%.2f", SHOOTER_D);
            telemetry.addData("F", "%.2f", SHOOTER_F);
            telemetry.addData("", "");
            telemetry.addData("=== SHOOTER STATUS ===", "");
            telemetry.addData("Running", shooterRunning ? "YES" : "NO");
            telemetry.addData("Target Vel (t/s)", "%.0f", SHOOTER_TARGET_VEL);
            telemetry.addData("Motor1 Vel (t/s)", "%.0f", shooterVel1);
            telemetry.addData("Motor2 Vel (t/s)", "%.0f", shooterVel2);
            telemetry.addData("Error1", "%.0f", Math.abs(shooterVel1 - SHOOTER_TARGET_VEL));
            telemetry.addData("Error2", "%.0f", Math.abs(shooterVel2 - SHOOTER_TARGET_VEL));
            telemetry.addData("At Speed", shooterAtSpeed ? "✓ YES" : "✗ NO");

            telemetry.update();
        }

        // Stop motors on exit
        ShooterMotor1.setVelocity(0);
        ShooterMotor2.setVelocity(0);
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