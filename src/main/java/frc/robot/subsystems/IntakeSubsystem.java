//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.ISubsystem;
import frc.robot.lib.k;

public class IntakeSubsystem extends SubsystemBase implements ISubsystem {
  CANSparkMax m_rotateMotor;
  PIDController m_rotatePID;
  //TalonFX m_spinMotor;
  VoltageOut m_voltageOutSpin;
  public void updateDashboard() {

  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    initialize();
  }

  public void initialize() {
    RobotContainer.subsystems.add(this);
    m_rotateMotor = new CANSparkMax(k.ROBORIO_CAN_IDS.INTAKE_ROTATE, MotorType.kBrushless);
    m_rotateMotor.setIdleMode(IdleMode.kBrake);
    m_rotatePID = new PIDController(1, 0, 0);
    m_rotatePID.setIntegratorRange(-1, 1);
    m_rotatePID.setTolerance(0.01);
    m_rotatePID.reset();
   // m_spinMotor = new TalonFX(k.ROBORIO_CAN_IDS.INTAKE_SPIN);
    m_voltageOutSpin = new VoltageOut(0);
        SmartDashboard.putBoolean("Intake Rotate Enable", false);
    SmartDashboard.putNumber("Intake Test Volts", 0.0);
    
  }
  public void spin(double _speed){
   // m_spinMotor.setControl(m_voltageOutSpin.withEnableFOC(true).withOutput(_speed * k.ROBOT.BATTERY_MAX_VOLTS));
  }
  public void rotate(double _angle){
    if(SmartDashboard.getBoolean("Intake Rotate Enable", false)){
      double volts = SmartDashboard.getNumber("Intake Test Volts", 0.0);
      m_rotateMotor.setVoltage(volts);
    }else {
      //m_rotateMotor.setVoltage(pid + ff);
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
