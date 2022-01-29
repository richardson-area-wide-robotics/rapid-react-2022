package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Gearbox {
    private CANSparkMax[] controllers;
    private MotorControllerGroup motorControllerGroup;

    public Gearbox(CANSparkMax... controllers) {
        this.controllers = controllers;
        this.motorControllerGroup = new MotorControllerGroup(controllers);
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
    
    /**
     * Comment me! - Egan
     */
    public void setInverted(boolean isInverted) {
        this.motorControllerGroup.setInverted(isInverted);
    }
}