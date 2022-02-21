package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hangar extends SubsystemBase {
    
    private CANSparkMax elevatorMotor;
    private SparkMaxPIDController elevatorPIDController;
    private DoubleSolenoid leftMidPneumatic;
    private DoubleSolenoid rightMidPneumatic;
    private DoubleSolenoid leftHighPneumatic;
    private DoubleSolenoid rightHighPneumatic;

    public Hangar() {
        elevatorMotor = new CANSparkMax(0, null);
        elevatorPIDController = elevatorMotor.getPIDController();
        leftMidPneumatic = new DoubleSolenoid(null, 0, 0);
        rightMidPneumatic = new DoubleSolenoid(null, 0, 0);
        leftHighPneumatic = new DoubleSolenoid(null, 0, 0);
        rightHighPneumatic = new DoubleSolenoid(0, null, 0, 0);
    }

    /**
     * sends elevatorPower to elevatorMotor
     * positive power --> go up
     * negative power --> go down
     * @param elevatorPower between -1 and 1
     */
    public void setElevatorPower(double elevatorPower) {
        elevatorMotor.set(elevatorPower);
    }

    /**
     * directions TBD
     * opens or closes solenoids depending on direction
     * @param midOnOrOff (which direction are we rotating the hooks)
     */
    public void rotateHooksMid(boolean midOnOrOff) {
        if (midOnOrOff == true) {
            leftMidPneumatic.set(DoubleSolenoid.Value.kForward);
            rightMidPneumatic.set(DoubleSolenoid.Value.kForward);
        }
        else {
            leftMidPneumatic.set(DoubleSolenoid.Value.kReverse);
            rightMidPneumatic.set(DoubleSolenoid.Value.kReverse);
        }
    }

    /**
     * directions TBD
     * opens or closes solenoids depending on direction
     * @param highOnOrOff (which direction are we rotating the hooks)
     */
    public void rotateHooksHigh(boolean highOnOrOff) {
        if (highOnOrOff == true) {
            leftHighPneumatic.set(DoubleSolenoid.Value.kForward);
            rightHighPneumatic.set(DoubleSolenoid.Value.kForward);
        }
        else {
            leftHighPneumatic.set(DoubleSolenoid.Value.kReverse);
            rightHighPneumatic.set(DoubleSolenoid.Value.kReverse);
        }
    }

    /**
     * sensors not figured out yet
     * @return whether the hooks are secured on the mid bar
     */
    public boolean isGrabbingMidBar() {
        return false;
    }

    /**
     * sensors not figured out yet
     * @return whether the hooks are secured on the high bar
     */
    public boolean isGrabbingHighBar() {
        return false;
    }

    /**
     * sensors not figured out yet
     * @return whether the robot is fully on the ground
     */
    public boolean isRobotOnGround() {
        return false;
    }

    /**
     * @param elevatorPosition inches or cms from lowest position
     */
    public void setElevatorPosition(double elevatorPosition) {
        elevatorPIDController.setReference(elevatorPosition, CANSparkMax.ControlType.kPosition);
    }
}
