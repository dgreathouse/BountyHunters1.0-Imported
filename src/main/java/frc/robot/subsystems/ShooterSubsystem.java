//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
 import com.revrobotics.CANSparkMax;
 import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.ICommand;
import frc.robot.lib.ISubsystem;
import frc.robot.lib.k;

public class ShooterSubsystem extends SubsystemBase implements ISubsystem {
  TalonFX m_leftMotor;
  TalonFX m_rightMotor;
  CANSparkMax m_rotateMotor;
  VoltageOut m_spinVoltageOut = new VoltageOut(0);
  
  PIDController m_spinPID = new PIDController(0.1,0.1,0);
  PIDController m_rotatePID = new PIDController(0.01, 0, 0);
  ArmFeedforward m_armFeedForward = new ArmFeedforward(0.05, 0.4, 0.0);
  double m_spinSpeed_rps = 0;
  public void updateDashboard() {
    if(this.getCurrentCommand() != null){
      ((ICommand)this.getCurrentCommand()).updateDashboard();
      SmartDashboard.putString("ShooterSubsystem", this.getCurrentCommand().getName());
    }
    SmartDashboard.putNumber("Shooter Angle", getRotateAngle());
    SmartDashboard.putNumber("Shooter Speed", m_spinSpeed_rps);
    SmartDashboard.putNumber("Shooter Velocity", getShooterVelocity());
  }

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    initialize();
  }

  public void initialize() {

    RobotContainer.subsystems.add(this);
    m_leftMotor = new TalonFX(k.ROBORIO_CAN_IDS.SHOOTER_LEFT,k.ROBORIO_CAN_IDS.NAME);
    m_rightMotor = new TalonFX(k.ROBORIO_CAN_IDS.SHOOTER_RIGHT,k.ROBORIO_CAN_IDS.NAME);
    m_rotateMotor = new CANSparkMax(k.ROBORIO_CAN_IDS.SHOOTER_ROTATE, MotorType.kBrushless);
    m_spinPID.setIntegratorRange(-20, 20);
    m_spinPID.setIZone(50);
  }
  /** Spin the spinners
   * 
   * @param _speed +/- 1.0
   */
  public void spin(double _speed){
    // Create a class/module scoped variable to the requested Spin Velocity
    m_spinSpeed_rps = _speed * k.SHOOTER.SPIN_VELOCITY_MAX_ROT_PER_SEC;
    // Calculate a PID value that is the difference/error between what we want in Velocity and the motor velocity currently is.
    double pid = m_spinPID.calculate(getShooterVelocity(), m_spinSpeed_rps);
    // Calculate the voltage to apply to the motor to reach the velocity we want by adding the PID and requested velocity.
    double m_spinSpeed_v = ((pid + m_spinSpeed_rps) / k.SHOOTER.SPIN_VELOCITY_MAX_ROT_PER_SEC) * k.ROBOT.BATTERY_MAX_VOLTS;
    // Set the left and right motor voltage.
    m_leftMotor.setControl(m_spinVoltageOut.withEnableFOC(true).withOutput(m_spinSpeed_v));
    m_rightMotor.setControl(m_spinVoltageOut.withEnableFOC(true).withOutput(-m_spinSpeed_v));
  }
  public double getShooterVelocity(){
    return (m_leftMotor.getVelocity().getValueAsDouble() - m_rightMotor.getVelocity().getValueAsDouble())/2.0;
  }
  /** Rotate the shooter to an angle
   * 
   * @param _angle Degrees
   */
  public void rotate(double _angle){
     double pid = m_rotatePID.calculate(getRotateAngle(), _angle);
     MathUtil.clamp(pid, -2, 2);
     m_rotateMotor.setVoltage(pid*k.ROBOT.BATTERY_MAX_VOLTS);
  }
  public double getRotateAngle(){
    return m_rotateMotor.getEncoder().getPosition() / k.SHOOTER.ROTATE_GEAR_RATIO * 360.0;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
