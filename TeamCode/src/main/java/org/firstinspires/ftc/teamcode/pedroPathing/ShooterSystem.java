package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config; // Added for Dashboard
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.PIDFCoefficients; // Added for PIDF

@Config // Allows you to tune these values live in FTC Dashboard
public class ShooterSystem {
    // Hardware
    private DcMotorEx shooterMotor1, shooterMotor2;
    private DcMotorEx rampMotor;
    private CRServo crServoFrontR;
    private CRServo crServobackR;
    private Servo gateServo;

    // --- PIDF Coefficients (Public & Static for Dashboard) ---
    public static double P = 40.0;
    public static double I = 0.0;

    public static double D = -4.5;
    public static double F = 12.5;  // Auto-calculates if 0

    // Constants from your TeleOp
    public static double currentShooterTarget = 1650.0;
    private final double VELOCITY_STEP = 50.0;
    private final double MAX_VELOCITY = 2800.0;
    private final double RAMP_VELOCITY = 850.0;

    // Servo Positions
    public static double GATE_OPEN = 0.5;
    public static double GATE_CLOSED = 0.1;

    public ShooterSystem(HardwareMap hardwareMap) {
        // Initialize Hardware
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "ShooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "ShooterMotor2");
        rampMotor = hardwareMap.get(DcMotorEx.class, "rampmotor");
        crServoFrontR = hardwareMap.get(CRServo.class, "crServofrontR");
        crServobackR = hardwareMap.get(CRServo.class, "crServobackR");
        gateServo = hardwareMap.get(Servo.class, "GateServo");

        // Motor Configuration
        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Apply PIDF Coefficients to the internal motor controller
        updatePIDF();

        rampMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rampMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rampMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        crServoFrontR.setDirection(DcMotorSimple.Direction.REVERSE);
        crServobackR.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Updates the motor controller with current PIDF values.
     * Can be called repeatedly if tuning live.
     */
    public void updatePIDF() {
        shooterMotor1.setVelocityPIDFCoefficients(P, I, D, F);
        shooterMotor2.setVelocityPIDFCoefficients(P, I, D, F);
    }

    public void setPIDF(double p, double i, double d, double f) {
        this.P = p; this.I = i; this.D = d; this.F = f;
        shooterMotor1.setVelocityPIDFCoefficients(P, I, D, F);
        shooterMotor2.setVelocityPIDFCoefficients(P, I, D, F);
    }

    /**
     * Spins the flywheels up to the current target velocity.
     */
    public void startFlywheels() {
        shooterMotor1.setVelocity(currentShooterTarget);
        shooterMotor2.setVelocity(currentShooterTarget);
    }

    /**
     * Activates the feeding mechanism (Ramp, Gate, and Internal Servos) to fire.
     */
    public void shoot() {
        startFlywheels(); // Ensure wheels are spinning
        rampMotor.setVelocity(-RAMP_VELOCITY);
        gateServo.setPosition(GATE_OPEN);
        crServoFrontR.setPower(-1.0);
        crServobackR.setPower(1.0);
    }

    /**
     * Stops everything associated with the shooting mechanism.
     */
    public void stopAll() {
        shooterMotor1.setVelocity(0);
        shooterMotor2.setVelocity(0);
        rampMotor.setVelocity(0);
        crServoFrontR.setPower(0);
        crServobackR.setPower(0);
        gateServo.setPosition(GATE_CLOSED);
    }

    public void stopFeeding() {
        gateServo.setPosition(GATE_CLOSED);
        rampMotor.setVelocity(0);
        crServoFrontR.setPower(0);
        crServobackR.setPower(0);
    }

    /**
     * Manually adjust the shooter velocity (useful for TeleOp tuning).
     */
    public void adjustVelocity(boolean increase) {
        if (increase) currentShooterTarget += VELOCITY_STEP;
        else currentShooterTarget -= VELOCITY_STEP;
        currentShooterTarget = Range.clip(currentShooterTarget, 0, MAX_VELOCITY);
    }

    // --- Getters ---
    public double getTargetVelocity() { return currentShooterTarget; }
    public double getActualVelocity() { return shooterMotor1.getVelocity(); }
    public boolean isAtVelocity() {
        return Math.abs(getActualVelocity() - currentShooterTarget) < 50;
    }
}