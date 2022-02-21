package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake {
        
        private CANSparkMax intakeMotor;
        
        
        final private double REVERSE_SPEED = 0;
        final private double FORWARD_SPEED = 0;
        final private double RAMPRATE =0; 
        
        public Intake(int intakeMotorCANID,  Boolean invertIntakeMotor )
        {
            this.intakeMotor = new CANSparkMax(intakeMotorCANID, MotorType.kBrushless);
            this.intakeMotor.setSmartCurrentLimit(30);// 30 for a Neo550
            this.intakeMotor.setInverted(invertIntakeMotor);
            this.intakeMotor.setOpenLoopRampRate(RAMPRATE);
           
        }
        public void idle()
        {
            this.intakeMotor.set(0);
        }

        public void intake() // yo dawg I hear you like intakes
        {
            this.intakeMotor.set(FORWARD_SPEED);
        }
        
        public void outtake()
        {
            this.intakeMotor.set(REVERSE_SPEED);
        }
        
}
