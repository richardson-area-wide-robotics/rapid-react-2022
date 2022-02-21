package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

  private CANSparkMax intakeMotor;

  private final double REVERSE_SPEED = 0.5;
  private final double FORWARD_SPEED = 0.1;
  private final double RAMPRATE = 0.1;

  public Intake(int intakeMotorCANID, Boolean invertIntakeMotor) {
    this.intakeMotor = new CANSparkMax(intakeMotorCANID, MotorType.kBrushless);
    this.intakeMotor.setSmartCurrentLimit(30); // 30 for a Neo550
    this.intakeMotor.setInverted(invertIntakeMotor);
    this.intakeMotor.setOpenLoopRampRate(RAMPRATE);
  }

  public void idle() {
    this.intakeMotor.set(0);
  }

  public void gather() // yo dawg I hear you like intakes
      {
    this.intakeMotor.set(FORWARD_SPEED);
  }

  public void outtake() {
    this.intakeMotor.set(REVERSE_SPEED);
  }
}
