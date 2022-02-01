package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Gearbox {
    private CANSparkMax[] controllers;
    private MotorControllerGroup motorControllerGroup;
    private Encoder encoder;

    public Gearbox(Encoder encoder, CANSparkMax... controllers) {
        this.controllers = controllers;
        this.motorControllerGroup = new MotorControllerGroup(controllers);
        this.encoder = encoder;
        // TODO: these values are for a specific robot and specefic encoder
        // that may not be the same for all robots. Change them to reference
        // a value in Constants.java - Egan
        this.encoder.setDistancePerPulse((0.1524 * Math.PI) / 2048.0);
        this.resetEncoder();
    }

    public MotorControllerGroup getMotorControllerGroup() {
        return this.motorControllerGroup;
    }

    /**
    * Comment me! - Egan
    */
    public void setPower(double rate) {
        if (rate < -1.0) {
            rate = -1.0;
        } else if (rate > 1.0) {
            rate = 1.0;
        }
        this.motorControllerGroup.set(rate);
    }

    public void setRampRate(final double rate) {
        for(CANSparkMax controller: this.controllers) {
            controller.setOpenLoopRampRate(rate);
        }
    }

    public void setBrakeMode() {
        for(CANSparkMax controller: this.controllers) {
            controller.setIdleMode(IdleMode.kBrake);
        }
    }

    public void setCoastMode() {
        for(CANSparkMax controller: this.controllers) {
            controller.setIdleMode(IdleMode.kCoast);
        }
    }

    public double getEncoderRate() {
        return this.encoder.getRate();
    }
    
    /**
     * Comment me! - Egan
     */
    public void setInverted(boolean isInverted) {
        this.motorControllerGroup.setInverted(isInverted);
        this.encoder.setReverseDirection(isInverted);
    }

    public void resetEncoder() {
        this.encoder.reset();
    }

    public void setVoltage(double outputVoltage) {
        this.motorControllerGroup.setVoltage(outputVoltage);
    }

    /**
     * Comment me! - Egan
     * TODO: units?
     */
    public double getEncoderDistance() {
        return this.encoder.getDistance();
    }
}