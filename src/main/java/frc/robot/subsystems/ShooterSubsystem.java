//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
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

 // double m_allianceSign = 1.0;
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
   
   //m_allianceSign = GD.G_Alliance == Alliance.Red ? -1.0 : 1.0;
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
    GD.G_ShooterState = ShooterState.FEED;
  }
  public void setShooterOff(){
    GD.G_ShooterState = ShooterState.OFF;
  }
  public void setShooterPodium(){ // triangle
    GD.G_RobotTargetAngle.setTargetAngle(-25 * GD.G_AllianceSign);
    GD.G_ShooterState = ShooterState.PODIUM;
  }
  public void setShooterStraight(){ // cross
    GD.G_RobotTargetAngle.setTargetAngle(0);
    GD.G_ShooterState = ShooterState.STRAIGHT;
  }
  public void setShooterRight(){ // circle blue
    if(GD.G_Alliance == Alliance.Blue){
      GD.G_RobotTargetAngle.setTargetAngle(-45);
    }else {
      GD.G_RobotTargetAngle.setTargetAngle(-25);
    }
    GD.G_ShooterState = ShooterState.RIGHT;
  }
  public void setShooterLeft(){ // square blue
    if(GD.G_Alliance == Alliance.Blue){
      GD.G_RobotTargetAngle.setTargetAngle(25);
    }else {
      GD.G_RobotTargetAngle.setTargetAngle(45);
    }
    GD.G_ShooterState = ShooterState.LEFT;
  }

  @Override
  public void periodic() {
    // Handle Shooter Speed
    switch (GD.G_ShooterState) {
      case PODIUM:
        spin(k.SHOOTER.SPIN_SPEED_HIGH_LONG);
        break;
      case STRAIGHT:
        spin(k.SHOOTER.SPIN_SPEED_HIGH_LONG);
        break;
      case RIGHT:
        spin(k.SHOOTER.SPIN_SPEED_HIGH_LONG);
        break;
      case LEFT:
        spin(k.SHOOTER.SPIN_SPEED_HIGH_LONG);
        break;
      case FEED:
        spin(k.SHOOTER.SPIN_SPEED_LOW);
        break;
      case OFF:
        spin(k.SHOOTER.SPIN_SPEED_OFF);
        break;
      default:
        break;
    }
    // Handle flippers
    switch (GD.G_FlipperState) {
      case BACK:
        retractFlippers();
      break;
      case PRELOAD:
        preloadFlippers();
      break;
      case SHOOT:
        if(GD.G_ShooterState != ShooterState.OFF){
          extendFlippers();
          GD.G_NoteState = NoteState.OUT;
        }
      break;
      default:
        retractFlippers();
      break;
    }
    if (GD.G_IntakeFlashing == false && (GD.G_ShooterState == ShooterState.LEFT || GD.G_ShooterState == ShooterState.RIGHT)) {
        GD.G_ShooterFlashing = true;
        RobotContainer.m_LEDs.setMulticolorBlinky(0.1, 100,100, 0);
    } else if (GD.G_IntakeFlashing == false && GD.G_ShooterState == ShooterState.FEED) {
        GD.G_ShooterFlashing = true;
        RobotContainer.m_LEDs.setMulticolorBlinky(0.3, 100, 100, 0);
    } else if (GD.G_IntakeFlashing == false && GD.G_ShooterState == ShooterState.OFF) {
      GD.G_ShooterFlashing = false;
      RobotContainer.m_LEDs.setAllianceColor();
    }
  }
}
