//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.ISubsystem;

public class TramperSubsystem extends SubsystemBase  implements ISubsystem{
  public void updateDashboard() {

  }
  /** Creates a new ExtensionSubsystem. */
  public TramperSubsystem() {

    initialize();
  }

  public void initialize() {
    RobotContainer.subsystems.add(this);
  }
  // public void setTestVoltage(double _volts){
    
  // }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
