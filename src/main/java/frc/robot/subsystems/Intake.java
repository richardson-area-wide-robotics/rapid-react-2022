package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    
    private CANSparkMax leftArmMotor;
    private CANSparkMax rightArmMotor;
    private CANSparkMax armLeaderMotor;
    private SparkMaxPIDController armPIDController;
    private final static double kPArm = 0.0;
    private final static double kIArm = 0.0;
    private final static double kDArm = 0.0;
    private final static double kIzArm = 0.0;
    private final static double kFFArm = 0.0;
    private final static double kMaxOutputArm = 0.0;
    private final static double kMinOutputArm = 0.0;
    private CANSparkMax intakeMotor;
    private DigitalInput armUpperLimitSwitch;
    private DigitalInput armLowerLimitSwitch;

    public Intake() {
        leftArmMotor = new CANSparkMax(0, null);
        rightArmMotor = new CANSparkMax(0, null);
        leftArmMotor.setInverted(true);
        rightArmMotor.follow(leftArmMotor);
        armLeaderMotor = leftArmMotor;
        armPIDController = armLeaderMotor.getPIDController();
        leftArmMotor.getEncoder().setPositionConversionFactor(Constants.ARM_POSITION_CONVERISON_FACTOR);
        rightArmMotor.getEncoder().setPositionConversionFactor(Constants.ARM_POSITION_CONVERISON_FACTOR);
        armPIDController.setP(kPArm);
        armPIDController.setI(kIArm);
        armPIDController.setD(kDArm);
        armPIDController.setIZone(kIzArm);
        armPIDController.setFF(kFFArm);
        armPIDController.setOutputRange(kMinOutputArm, kMaxOutputArm);
        intakeMotor = new CANSparkMax(0, null);
        armUpperLimitSwitch = new DigitalInput(0);
        armLowerLimitSwitch = new DigitalInput(0);
    } 

    /**
     * @return whether power of intakeMotor is not 0
     */
    public boolean isIntakeRunning() {
        if (intakeMotor.get() != 0) {
            return true;
        }
        else {
            return false;
        }
    }

    /**
     * @return current position of arm from encoder(degrees from lowest position)
     */
    public double getArmPosition() {
        return armLeaderMotor.getEncoder().getPosition();
    }

    public int isControllingCargo() {
        return 0;
    }

    /**
     * @param armPower between -1 and 1
     */
    public void setArmPower(double armPower) {
        armLeaderMotor.set(armPower);
    }

    /**
     * @param armPosition between 0 and 50(degrees arm will go relative to lowest position)
     */
    public void setArmPosition(double armPosition) {
        armPIDController.setReference(armPosition, CANSparkMax.ControlType.kPosition);
    }

    /**
     * @param intakePower between -1 and 1
     * sends intakePower to intakeMotor
     * positive power --> intake
     * negative power --> outtake
     */
    public void setIntakePower(double intakePower) {
        intakeMotor.set(intakePower);
    }

    /**
     * @return whether limit switch is touching, or "true"
     */
    public boolean getUpperLimitSwitchStatus() {
        return armUpperLimitSwitch.get();
    }

    /**
     * @return whether limit switch is touching, or "true"
     */
    public boolean getLowerLimitSwitchStatus() {
        return armLowerLimitSwitch.get();
    }
}
