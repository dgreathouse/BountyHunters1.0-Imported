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
import frc.robot.lib.ShooterState;
import frc.robot.lib.k;

public class ShooterSubsystem extends SubsystemBase implements ISubsystem {

  TalonFX m_leftMotor; // Declare a TalonFX motor controller class and call it m_leftMotor;
  TalonFX m_rightMotor; // Declare a TalonFX motor controller class and call it m_rightMotor;

  Servo m_leftServo;
  Servo m_rightServo;

  VoltageOut m_spinVoltageOut = new VoltageOut(0);

  public void updateDashboard() {
    SmartDashboard.putString("Shooter State", GD.G_ShooterState.toString());
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
    m_leftServo = new Servo(0);
    m_rightServo = new Servo(1);

    GD.G_ShooterState = ShooterState.OFF;
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
  }
  public void setFlipperPreload() {
    GD.G_FlipperState = FlipperStates.PRELOAD;
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

  public void setShooterFeed(){
    GD.G_ClimberVoltageMode = false;
    GD.G_ClimberPosition = -10;
    GD.G_ShooterState = ShooterState.FEED;
  }
  public void setShooterOff(){
    GD.G_ClimberVoltageMode = false;
    GD.G_ShooterState = ShooterState.OFF;
  }

  @Override
  public void periodic() {
    // Look at the Shooter State
    // Look at the Shooter Speed
    // Look 
    // Handle Shooter speed and shooter angle
    switch (GD.G_ShooterState) {
      case PODIUM:
        
        spin(k.SHOOTER.SPIN_SPEED_HIGH_LONG);
        break;
      case SPEAKER:
        spin(k.SHOOTER.SPIN_SPEED_HIGH_SHORT);
        break;
      case LEFT:
        spin(k.SHOOTER.SPIN_SPEED_LOW);
        break;
        case RIGHT:
        spin(k.SHOOTER.SPIN_SPEED_HIGH_LONG);
        break;
      case OFF:
        spin(k.SHOOTER.SPIN_SPEED_OFF);
        break;
        case FEED:
        spin(k.SHOOTER.SPIN_SPEED_LOW);
        break;
      default:
        break;
    }
    // Handle flippers
    if(GD.G_FlipperState == FlipperStates.BACK){
      retractFlippers();
    }else if(GD.G_FlipperState == FlipperStates.PRELOAD){
      preloadFlippers();
    }else if(GD.G_FlipperState == FlipperStates.SHOOT){
      if(GD.G_ShooterState == ShooterState.PODIUM || GD.G_ShooterState == ShooterState.SPEAKER || GD.G_ShooterState == ShooterState.LEFT || GD.G_ShooterState == ShooterState.FEED){
        extendFlippers();
        GD.G_NoteState = NoteState.OUT;
      }
    }
    

  }
}
