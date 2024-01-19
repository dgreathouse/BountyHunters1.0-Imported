//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.ICommand;
import frc.robot.lib.ISubsystem;

public class ShooterSubsystem extends SubsystemBase implements ISubsystem {
  TalonFX m_leftMotor;
  TalonFX m_rightMotor;
  public void updateDashboard() {
    if(this.getCurrentCommand() != null){
      ((ICommand)this.getCurrentCommand()).updateDashboard();
      SmartDashboard.putString("ShooterSubsystem", this.getCurrentCommand().getName());
    }
  }
public void spin(double _speed) { // dogs
      m_leftMotor.set
  }
public void setAngle(double kitty) {
      
  }

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
m_leftMotor = new TalonFX(0);
m_rightMotor = new TalonFX(0);
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
