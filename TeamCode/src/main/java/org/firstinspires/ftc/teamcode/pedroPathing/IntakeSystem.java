package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSystem {

    // Hardware components
    private DcMotorEx intakeMotor;
    private DcMotorEx rampMotor;
    private CRServo crServo;

    // Velocity constants
    private double intakeVelocity;
    private double rampVelocity;

    /**
     * Constructor with default velocities
     */
    public IntakeSystem(HardwareMap hardwareMap) {
        this(hardwareMap, 2800.0, 1500.0);
    }

    /**
     * Constructor with custom velocities
     */
    public IntakeSystem(HardwareMap hardwareMap, double intakeVelocity, double rampVelocity) {
        this.intakeVelocity = intakeVelocity;
        this.rampVelocity = rampVelocity;

        // Initialize hardware
        intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        rampMotor = hardwareMap.get(DcMotorEx.class, "rampmotor");
        crServo = hardwareMap.get(CRServo.class, "crServofrontR");

        // Set motor modes and directions
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rampMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rampMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        crServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Run intake at full speed
     */
    public void runIntake() {
        intakeMotor.setVelocity(intakeVelocity);
        rampMotor.setVelocity(rampVelocity);
        crServo.setPower(-1.0);
    }

    /**
     * Stop all intake motors and servos
     */
    public void stopAll() {
        intakeMotor.setVelocity(0);
        rampMotor.setVelocity(0);
        crServo.setPower(0);
    }

    // Getters for telemetry
    public double getIntakeVelocity() {
        return intakeMotor.getVelocity();
    }

    public double getRampVelocity() {
        return rampMotor.getVelocity();
    }

    public double getCRServoPower() {
        return crServo.getPower();
    }

    // Setters for customization
    public void setIntakeVelocity(double velocity) {
        this.intakeVelocity = velocity;
    }

    public void setRampVelocity(double velocity) {
        this.rampVelocity = velocity;
    }
}
//