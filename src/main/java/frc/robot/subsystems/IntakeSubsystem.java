//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.FlipperStates;
import frc.robot.lib.GD;
import frc.robot.lib.ISubsystem;
import frc.robot.lib.NoteState;
import frc.robot.lib.k;


public class IntakeSubsystem extends SubsystemBase implements ISubsystem {
  TalonFX m_leftMotor;
  VoltageOut leftVoltageOut = new VoltageOut(0);
  double m_previousCurrent = 0;
  Timer m_timer = new Timer();

  public void updateDashboard() {
    if(GD.G_NoteState == NoteState.IN || GD.G_NoteState == NoteState.INNOFLASH){
      SmartDashboard.putBoolean("Note State", true);
    }else {
      SmartDashboard.putBoolean("Note State", false);
    }
    SmartDashboard.putNumber("Intake Current", m_leftMotor.getStatorCurrent().getValueAsDouble());
  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    initialize();
  }

  public void initialize() {
    RobotContainer.subsystems.add(this);
    m_leftMotor = new TalonFX(k.ROBORIO_CAN_IDS.INTAKE_LEFT_SPIN);
  }
  public void spinOn(){
    GD.G_Intake_Speed = k.INTAKE.SPIN_SPEED_DEFAULT_VOLT;
  }
  public void spinOff(){
    GD.G_Intake_Speed = 0.0;
  }
  public void spinReverse(){
    GD.G_Intake_Speed = -k.INTAKE.SPIN_SPEED_DEFAULT_VOLT;
  }
  public void spin(double _volts){
    double volts = _volts;
    if(GD.G_FlipperState != FlipperStates.BACK && _volts > 0){
      volts = 0.0;
    }else {
      volts = _volts;
    }
    m_leftMotor.setControl(leftVoltageOut.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void periodic() {
    spin(GD.G_Intake_Speed);
    double statorCurrent = m_leftMotor.getStatorCurrent().getValueAsDouble();

    if(GD.G_Intake_Speed > 0){
      if(statorCurrent > k.INTAKE.NOTE_CURRENT){
       GD.G_NoteState = NoteState.IN;
      }
    }
    if(GD.G_NoteState == NoteState.IN && !m_timer.hasElapsed(2) && GD.G_ShooterFlashing == false){
      RobotContainer.m_LEDs.setMulticolorBlinky(0.1, 0, 100, 0);
      m_timer.start();
      GD.G_IntakeFlashing = true;
    }else if (GD.G_ShooterFlashing == false) {
       RobotContainer.m_LEDs.setAllianceColor();
       GD.G_IntakeFlashing = false;
    }
    if (m_timer.hasElapsed(2)) {
      GD.G_NoteState = NoteState.INNOFLASH;
      GD.G_IntakeFlashing = false;
      m_timer.stop();
      m_timer.reset();
    }
    SmartDashboard.putBoolean("Functional LEDs", RobotContainer.m_LEDs.functionalLEDs);
  }
}
