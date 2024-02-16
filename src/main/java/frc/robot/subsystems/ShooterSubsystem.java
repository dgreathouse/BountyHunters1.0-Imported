//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.GD;
import frc.robot.lib.ISubsystem;
import frc.robot.lib.k;

public class ShooterSubsystem extends SubsystemBase implements ISubsystem {
  
  TalonFX m_leftMotor;        // Declare a TalonFX motor controller class and call it m_leftMotor;
  TalonFX m_rightMotor;       // Declare a TalonFX motor controller class and call it m_rightMotor;
  CANSparkMax m_rotateMotor;  // Declare a CANSparkMax motor controller class and call it m_rotateMotor;
  Servo m_leftServo;
  Servo m_rightServo;
  // Declare and initialize a VoltageOut class and call the instance m_spinVoltageOut
  VoltageOut m_spinVoltageOut = new VoltageOut(0);
  // Create the contraints our PID controller must follow 
  // Velocity and Acceleration contraints must be in Rad/Sec and Rad/Sec^2
  TrapezoidProfile.Constraints m_rotateContraints = new TrapezoidProfile.Constraints(0.5236, 1.05);
  // Create a profiled PID controller with the contraints 
  // kP and kI should be in Volt/Rad
  ProfiledPIDController m_rotatePID = new ProfiledPIDController(1.15, 0.08, 0, m_rotateContraints);
  // Create a ArmFeedforward object to control the rotate motor.
  // ks in volts, kg in volts kv in volt seconds per radian and ka in volt seconds² per radian
  ArmFeedforward m_armFeedForward = new ArmFeedforward(0.0, 0.4, 1.18, 0.0);

  double m_requestedAngle_deg = 0; // Angle data to store the requested Angle the shooter should rotate to.
  double m_requestedVelocity_rps = 0;

  double m_spinSpeed_v = 0;

  /** Any dashboard telemetry should be set here in this method
   * 
   */
  public void updateDashboard() {

    SmartDashboard.putNumber("Shooter Actual Angle Deg",getActualAngle());
    SmartDashboard.putNumber("Shooter Requested Angle Deg", GD.G_ShooterAngle);
    SmartDashboard.putNumber("Shooter Actual Velocity RPS", getSpinnerActualVelocity());
    SmartDashboard.putData(m_rotatePID);
  }

  /** Contructor that creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    initialize();
  }

  public void initialize() {

    RobotContainer.subsystems.add(this);
    m_leftMotor = new TalonFX(k.ROBORIO_CAN_IDS.SHOOTER_LEFT,k.ROBORIO_CAN_IDS.NAME);
    m_rightMotor = new TalonFX(k.ROBORIO_CAN_IDS.SHOOTER_RIGHT,k.ROBORIO_CAN_IDS.NAME);
    m_rotateMotor = new CANSparkMax(k.ROBORIO_CAN_IDS.SHOOTER_ROTATE, MotorType.kBrushless);
    m_leftServo = new Servo(1);
    m_rightServo = new Servo(2);
    m_rotatePID.setIntegratorRange(-1, 1);
    m_rotatePID.setTolerance(0.5);

    // Smartdashboard test variables
    SmartDashboard.putBoolean("Shooter Rotate Enable", false);
    SmartDashboard.putNumber("Shooter Test Volts", 0.0);
    

  }
  /** Spin the spinners
   * 
   * @param _speed +/- 1.0
   */
  public void spin(double _speed){
    m_spinSpeed_v = _speed * k.ROBOT.BATTERY_MAX_VOLTS;
    // Set the left and right motor voltage.
    m_leftMotor.setControl(m_spinVoltageOut.withEnableFOC(true).withOutput(m_spinSpeed_v));
    m_rightMotor.setControl(m_spinVoltageOut.withEnableFOC(true).withOutput(-m_spinSpeed_v));
  }

  /** Rotate the shooter to an angle
   * 
   * @param _angle Degrees
   */
  public void rotate(double _angle){
    // calculate the PID value based on the actual angle in degrees and the requested goal to achieve
    double pid = m_rotatePID.calculate(Math.toRadians(getActualAngle()), Math.toRadians(_angle));
    // calculate the FeedForward value based on the actual angle and the desired velocity the PID wants.
    double ff = m_armFeedForward.calculate(Math.toRadians(getActualAngle()), m_rotatePID.getSetpoint().velocity);
    // Limit the amount the PID can contribute to the output since the FeedForward should do most of the work
    pid = MathUtil.clamp(pid, -2, 2);
    // Set the actual voltage to the motor by combining the PID and Feedforward values
    if(SmartDashboard.getBoolean("Shooter Rotate Enable", false)){
      double volts = SmartDashboard.getNumber("Shooter Test Volts", 0.0);
      m_rotateMotor.setVoltage(volts);
    }else {
      m_rotateMotor.setVoltage(pid + ff);
    }
    
  }
  public void ShootNote(boolean _shoot){
    if(_shoot){
      m_leftServo.set(0.4);
      m_rightServo.set(0.49);
    }else {
      m_leftServo.set(.6);
      m_rightServo.set(.29);
    }
  }

  public double getActualAngle(){
    double angle_deg = m_rotateMotor.getEncoder().getPosition() / k.SHOOTER.ROTATE_GEAR_RATIO * 360.0 + k.SHOOTER.ROTATE_OFFSET_ANGLE_DEG;
    return angle_deg;
  }

  public double getRequestedAngle_deg() {
    return m_requestedAngle_deg;
  }

  public void setRequestedAngle_deg(double _requestedAngle_deg) {
    this.m_requestedAngle_deg = _requestedAngle_deg;
  }

  public double getRequestedVelocity_rps() {
    return m_requestedVelocity_rps;
  }

  public void setRequestedVelocity_rps(double _requestedVelocity_rps) {
    this.m_requestedVelocity_rps = _requestedVelocity_rps;
  }
  public double getSpinnerActualVelocity(){
    double motorVelocity = (m_leftMotor.getVelocity().getValueAsDouble() - m_rightMotor.getVelocity().getValueAsDouble())/2.0;

    return motorVelocity;
  }
  public double getRotateActualVelocity(){
    return 0.0;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
