//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.ISubsystem;

public class ClimberSubsystem extends SubsystemBase  implements ISubsystem{
  //TalonFX m_motor;

  public void updateDashboard() {

  }

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
 //   m_motor = new TalonFX(20, k.ROBORIO_CAN_IDS.NAME);
//    m_motor.setNeutralMode(NeutralModeValue.Brake);

    initialize();
  }
  private void initialize(){
    RobotContainer.subsystems.add(this);
  }
  // public void setTestVoltage(double _volts){
    
  // }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
