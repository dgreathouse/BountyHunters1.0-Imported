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
  VoltageOut m_leftVoltageOut = new VoltageOut(0);
  double m_previousCurrent = 0;
  Timer m_timer = new Timer();
  int m_noteBlinkCnt = 0;
  int m_noteBlinkEndCnt = 0;
  boolean m_latch = false;

  public void updateDashboard() {
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

  public void spinOn() {
    GD.G_Intake_Speed = k.INTAKE.SPIN_SPEED_DEFAULT_VOLT;
  }

  public void spinOff() {
    GD.G_Intake_Speed = 0.0;
  }

  public void spinReverse() {
    GD.G_Intake_Speed = -k.INTAKE.SPIN_SPEED_DEFAULT_VOLT;
  }

  public void spin(double _volts) {
    double volts = _volts;
    if (GD.G_FlipperState != FlipperStates.BACK && _volts > 0) {
      volts = 0.0;
    } else {
      volts = _volts;
    }
    m_leftMotor.setControl(m_leftVoltageOut.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void periodic() {
    spin(GD.G_Intake_Speed);
    double statorCurrent = m_leftMotor.getStatorCurrent().getValueAsDouble();
    if (GD.G_Intake_Speed > 0) {
      if (statorCurrent > k.INTAKE.NOTE_CURRENT && !m_latch) {
        GD.G_NoteState = NoteState.IN;
      }
    }
    if (GD.G_NoteState == NoteState.IN) {
      m_noteBlinkEndCnt++;
      if(m_noteBlinkEndCnt > 0 && m_noteBlinkEndCnt < 10){
        RobotContainer.m_LEDs.setRGBColor(0, 0, 0);
      }else if(m_noteBlinkEndCnt >= 10 && m_noteBlinkEndCnt < 20){
        RobotContainer.m_LEDs.setRGBColor(0, 100, 0);
      }else if(m_noteBlinkEndCnt >= 20 && m_noteBlinkEndCnt < 30){
        RobotContainer.m_LEDs.setRGBColor(0, 0, 0);
      }else if(m_noteBlinkEndCnt >= 30 && m_noteBlinkEndCnt < 40){
        RobotContainer.m_LEDs.setRGBColor(0, 100, 0);
      }else if(m_noteBlinkEndCnt >= 40 && m_noteBlinkEndCnt < 50){
        RobotContainer.m_LEDs.setRGBColor(0, 0, 0);
      }else if(m_noteBlinkEndCnt >= 50 && m_noteBlinkEndCnt < 60){
        RobotContainer.m_LEDs.setRGBColor(0, 100, 0);
      }else if(m_noteBlinkEndCnt >= 60 && m_noteBlinkEndCnt < 70){
        RobotContainer.m_LEDs.setRGBColor(0, 0, 0);
      }else if(m_noteBlinkEndCnt >= 70 && m_noteBlinkEndCnt < 80){
        RobotContainer.m_LEDs.setRGBColor(0, 100, 0);
      }else if(m_noteBlinkEndCnt >= 80 && m_noteBlinkEndCnt < 90){
        RobotContainer.m_LEDs.setRGBColor(0, 0, 0);
      }else if(m_noteBlinkEndCnt >= 100 ){
        m_noteBlinkEndCnt = 0;
        GD.G_NoteState = NoteState.OUT;
        RobotContainer.m_LEDs.setAllianceColor();
      }
    }
  }
}
