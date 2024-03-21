// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.AmpState;
import frc.robot.lib.GD;
import frc.robot.lib.k;

public class AmpSubsystem extends SubsystemBase {
  CANSparkMax m_motor;
  /** Creates a new AmpSubsystem. */
  public AmpSubsystem() {
    initialize();
  }
  private void initialize(){
    m_motor = new CANSparkMax(k.ROBORIO_CAN_IDS.AMP, MotorType.kBrushless);
    m_motor.setIdleMode(IdleMode.kBrake);
  }
  public void setAmpUp( ){
    GD.G_AmpState = AmpState.UP;
  }
  public void setAmpDown( ){
    GD.G_AmpState = AmpState.DOWN;
  }
  @Override
  public void periodic() {
    double ampVolts = -.0;
    if(GD.G_AmpState == AmpState.UP){
      ampVolts = 0;
    }
    m_motor.setVoltage(ampVolts);
    SmartDashboard.putNumber("AmpVolts", ampVolts);
  }
}
