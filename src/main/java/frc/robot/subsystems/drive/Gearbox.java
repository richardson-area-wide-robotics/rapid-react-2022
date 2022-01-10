package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class Gearbox {
    private final CANSparkMax frontController;
    private final CANSparkMax backController;
    private SpeedControllerGroup gearbox;

    public Gearbox(final CANSparkMax frontController, final CANSparkMax backController, SpeedControllerGroup gearbox) {
        this.frontController = frontController;
        this.backController = backController;
        this.gearbox = gearbox;
    }

    public void setSpeed(double rate) {
        if (rate < -1.0) {
            rate = -1.0;
        } else if (rate > 1.0) {
            rate = 1.0;
        }
        this.gearbox.set(rate);
    }

    public void setRampRate(final double rate) {
        this.frontController.setOpenLoopRampRate(rate);
        this.backController.setOpenLoopRampRate(rate);
    }

    public CANSparkMax getBackController() {
        return this.backController;
    }

    public CANSparkMax getFrontController() {
        return this.frontController;
    }

    public void setBrakeMode() {
        this.frontController.setIdleMode(IdleMode.kBrake);
        this.backController.setIdleMode(IdleMode.kBrake);
    }

    public void setCoastMode() {
        this.frontController.setIdleMode(IdleMode.kCoast);
        this.backController.setIdleMode(IdleMode.kCoast);
    }
}