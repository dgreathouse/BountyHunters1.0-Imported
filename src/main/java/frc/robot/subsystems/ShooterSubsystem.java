//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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
  
  PneumaticsControlModule m_pcm;
  DoubleSolenoid m_lifterSolenoid;
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
    m_leftServo = new Servo(0);
    m_rightServo = new Servo(1);

    m_pcm = new PneumaticsControlModule(k.ROBORIO_CAN_IDS.PCM);
    m_lifterSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
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

  public void setShooterOnHighLong(){
    GD.G_ShooterState = ShooterState.HIGH_LONG;
  }
  public void setShooterOnHighShort(){
    GD.G_ShooterState = ShooterState.HIGH_SHORT;
  }
  public void setShooterOnLow(){
    GD.G_ShooterState = ShooterState.LOW;
  }
  public void setShooterOff(){
    GD.G_ShooterState = ShooterState.OFF;
  }
  private void raiseShooter(){
    m_lifterSolenoid.set(Value.kForward);
  }
  private void lowerShooter(){
    m_lifterSolenoid.set(Value.kReverse);
  }
  @Override
  public void periodic() {
    // Handle Shooter speed and shooter angle
    switch (GD.G_ShooterState) {
      case HIGH_LONG:
        spin(k.SHOOTER.SPIN_SPEED_HIGH_LONG);
        lowerShooter();
        break;
      case HIGH_SHORT:
        spin(k.SHOOTER.SPIN_SPEED_HIGH_SHORT);
        raiseShooter();
        break;
      case LOW:
        spin(k.SHOOTER.SPIN_SPEED_LOW);
        raiseShooter();
        break;
      case OFF:
        spin(k.SHOOTER.SPIN_SPEED_OFF);
        lowerShooter();
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
      if(GD.G_ShooterState == ShooterState.HIGH_LONG || GD.G_ShooterState == ShooterState.HIGH_SHORT || GD.G_ShooterState == ShooterState.LOW){
        extendFlippers();
        GD.G_NoteState = NoteState.OUT;
      }
    }
    

  }
}
