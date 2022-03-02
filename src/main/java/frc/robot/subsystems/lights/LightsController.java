package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.sensors.TOFSensor;

public class LightsController extends SubsystemBase {
  private Lights lights;
  private TOFSensor tofSensor;
  // private Limelight limelight;
  boolean isIdle = true;

  public LightsController(Lights lights, TOFSensor tofSensor) {
    this.lights = lights;
    this.tofSensor = new TOFSensor(1);
    // this.limelight = limelight;
    this.lights.idleAnimation(3);
  }

  /*public void checkTargetLock() {
      if (limelight.hasValidTarget() && !isIdle) {
          lights.allLimeGreen();
          isIdle = true;
      } else if(!limelight.hasValidTarget() && isIdle){
          lights.idleAnimation(3);
          isIdle = false;
      }
  }*/
}
