//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.FlipperStates;
import frc.robot.lib.GD;
import frc.robot.lib.ISubsystem;
import frc.robot.lib.NoteState;
import frc.robot.lib.k;

public class ShooterSubsystem extends SubsystemBase implements ISubsystem {

  TalonFX m_leftMotor; // Declare a TalonFX motor controller class and call it m_leftMotor;
  TalonFX m_rightMotor; // Declare a TalonFX motor controller class and call it m_rightMotor;

  Servo m_leftServo;
  Servo m_rightServo;

  VoltageOut m_spinVoltageOut = new VoltageOut(0);

  public void updateDashboard() {

  }

  /** Contructor that creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    initialize();
  }

  public void initialize() {
    RobotContainer.subsystems.add(this);
    m_leftMotor = new TalonFX(k.ROBORIO_CAN_IDS.SHOOTER_LEFT, k.ROBORIO_CAN_IDS.NAME);
    m_rightMotor = new TalonFX(k.ROBORIO_CAN_IDS.SHOOTER_RIGHT, k.ROBORIO_CAN_IDS.NAME);
    m_leftMotor.setNeutralMode(NeutralModeValue.Brake);
    m_rightMotor.setNeutralMode(NeutralModeValue.Brake);
    SmartDashboard.putNumber("Shot Speed", 0);
    m_leftServo = new Servo(1);
    m_rightServo = new Servo(2);
  }

  /**
   * Spin the spinners
   * 
   * @param _speed +/- 1.0
   */
  public void spin(double _speed) {
    // Set the left and right motor voltage.
    m_leftMotor.setControl(m_spinVoltageOut.withEnableFOC(true).withOutput(_speed * k.ROBOT.BATTERY_MAX_VOLTS));
    m_rightMotor.setControl(m_spinVoltageOut.withEnableFOC(true).withOutput(-_speed * k.ROBOT.BATTERY_MAX_VOLTS));
  }

  public void setFlippersRetracted() {
    GD.G_FlipperState = FlipperStates.BACK;
  }

  public void setFlipperExtended() {
    GD.G_FlipperState = FlipperStates.SHOOT;
    GD.G_NoteState = NoteState.OUT;
  }
  public void setShooterOnHigh(){
    GD.G_ShooterSpeed = 0.68;
  }
  public void setShooterOnLow(){
    GD.G_ShooterSpeed = 0.35;
  }
  public void setShooterOff(){
    GD.G_ShooterSpeed = 0.0;
  }

  private void retractFlippers() {
    m_leftServo.set(.6);
    m_rightServo.set(.325);
  }

  private void extendFlippers() {
    m_leftServo.set(0.2);
    m_rightServo.set(0.69);
  }
  private void preloadFlippers() {
    m_leftServo.set(0.41);
    m_rightServo.set(0.50);
  }
  @Override
  public void periodic() {
    spin(GD.G_ShooterSpeed);

    if(GD.G_FlipperState == FlipperStates.BACK){
      retractFlippers();
    }else if(GD.G_FlipperState == FlipperStates.PRELOAD){
      preloadFlippers();
    }else if(GD.G_FlipperState == FlipperStates.SHOOT){
      if(GD.G_ShooterSpeed > 0.05){
        extendFlippers();
      }
    }
    if(GD.G_FlipperState == FlipperStates.BACK){
     // RobotContainer.m_LEDs.setRGBColor(0, 100, 0);
    } else {
     // RobotContainer.m_LEDs.setAllianceColor();
    }

  }
}
