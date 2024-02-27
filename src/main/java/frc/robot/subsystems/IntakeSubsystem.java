//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.GD;
import frc.robot.lib.ISubsystem;
import frc.robot.lib.k;


public class IntakeSubsystem extends SubsystemBase implements ISubsystem {
  TalonFX m_leftMotor;
  //TalonFX m_rightMotor;
  VoltageOut leftVoltageOut = new VoltageOut(0);
  //VoltageOut rightVoltageOut = new VoltageOut(0);
  public void updateDashboard() {

  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    initialize();
  }

  public void initialize() {
    RobotContainer.subsystems.add(this);
    m_leftMotor = new TalonFX(k.ROBORIO_CAN_IDS.INTAKE_LEFT_SPIN);
    //m_rightMotor = new TalonFX(k.ROBORIO_CAN_IDS.INTAKE_RIGHT_SPIN);
  }
  public void spinOn(){
    GD.G_Intake_Speed = 0.5;
  }
  public void spinOff(){
    GD.G_Intake_Speed = 0.0;
  }
  public void spin(double _speed){
    m_leftMotor.setControl(leftVoltageOut.withOutput(_speed*k.ROBOT.BATTERY_MAX_VOLTS).withEnableFOC(true));
   // m_rightMotor.setControl(rightVoltageOut.withOutput(-_speed*k.ROBOT.BATTERY_MAX_VOLTS).withEnableFOC(true));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
