//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.ISubsystem;
import frc.robot.lib.k;


public class IntakeSubsystem extends SubsystemBase implements ISubsystem {
  CANSparkFlex m_spinMotor;

  public void updateDashboard() {

  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    initialize();
  }

  public void initialize() {
    RobotContainer.subsystems.add(this);
    m_spinMotor = new CANSparkFlex(k.ROBORIO_CAN_IDS.INTAKE_SPIN, MotorType.kBrushless);
    
  }
  public void spin(double _speed){
    m_spinMotor.setVoltage(_speed * k.ROBOT.BATTERY_MAX_VOLTS);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
