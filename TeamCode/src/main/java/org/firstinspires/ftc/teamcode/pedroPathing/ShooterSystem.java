package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class ShooterSystem {
    // Hardware
    private DcMotorEx shooterMotor1, shooterMotor2;
    private DcMotorEx rampMotor;
    private CRServo crServoFrontR;
    private CRServo crServobackR;
    private Servo gateServo;

    // Constants from your TeleOp
    private double currentShooterTarget = 1650.0;
    private final double VELOCITY_STEP = 50.0;
    private final double MAX_VELOCITY = 2800.0;
    private final double RAMP_VELOCITY = 850.0;

    // Servo Positions
    private final double GATE_OPEN = 0.5;
    private final double GATE_CLOSED = 0.1;

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

        rampMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rampMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rampMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        crServoFrontR.setDirection(DcMotorSimple.Direction.REVERSE);
        crServobackR.setDirection(DcMotorSimple.Direction.FORWARD);
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