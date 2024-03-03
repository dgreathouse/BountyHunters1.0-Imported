//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.Rev2mDistanceSensor.Port;
// import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
// import com.revrobotics.Rev2mDistanceSensor.Unit;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.Rev2mDistanceSensor;
// import com.revrobotics.CANSparkBase.IdleMode;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.GD;
import frc.robot.lib.ISubsystem;
import frc.robot.lib.k;

public class ShooterSubsystem extends SubsystemBase implements ISubsystem {

  TalonFX m_leftMotor; // Declare a TalonFX motor controller class and call it m_leftMotor;
  TalonFX m_rightMotor; // Declare a TalonFX motor controller class and call it m_rightMotor;
 // CANSparkMax m_rotateMotor; // Declare a CANSparkMax motor controller class and call it m_rotateMotor;
  Servo m_leftServo;
  Servo m_rightServo;
  //Rev2mDistanceSensor m_distanceSensor;
  // Declare and initialize a VoltageOut class and call the instance m_spinVoltageOut
  VoltageOut m_spinVoltageOut = new VoltageOut(0);
  //PIDController m_rotatePID = new PIDController(k.SHOOTER.ROTATE_PID_KP, k.SHOOTER.ROTATE_PID_KI, 0);
  // double m_Rotate_PID_volts = 0;
  // double m_Rotate_FF_volts = 0;
  double[] m_distanceAverage = new double[8];
  int m_distanceAvgIndex = 0;
  double m_distance = 0;
  /**
   * Any dashboard telemetry should be set here in this method
   * 
   */
  public void updateDashboard() {

 //   SmartDashboard.putNumber("Shooter Actual Angle Deg", getActualAngle());
 //   SmartDashboard.putNumber("Shooter Requested Angle Deg", GD.G_ShooterAngle);
    //SmartDashboard.putBoolean("Shooter In Range", isRotateInRange());
    
  //  SmartDashboard.putNumber("Shooter PID", m_Rotate_PID_volts);
   // SmartDashboard.putNumber("Shooter Feedforward", m_Rotate_FF_volts);
    //SmartDashboard.putNumber("Shooter Distance", m_distanceSensor.getRange());
   // SmartDashboard.putNumber("Shooter Distance Avg", getDistance());
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
    // m_rotateMotor = new CANSparkMax(k.ROBORIO_CAN_IDS.SHOOTER_ROTATE, MotorType.kBrushless);
    // m_rotateMotor.setIdleMode(IdleMode.kBrake);
    m_leftServo = new Servo(1);
    m_rightServo = new Servo(2);
    // m_rotatePID.setIntegratorRange(-1, 1);
    // m_rotatePID.setTolerance(0.01);
    // m_rotatePID.reset();
    
    //m_distanceSensor = new Rev2mDistanceSensor(Port.kMXP, Unit.kMillimeters, RangeProfile.kHighAccuracy);
    //m_distanceSensor.setAutomaticMode(true);
   // m_distanceSensor.setEnabled(true);
    // for(int i = 0; i < 8; i++){
    //   m_distanceAverage[i] = k.SHOOTER.ROTATE_DS_OFFSET_DISTANCE_MM;
    // }
    // Smartdashboard test variables
    // SmartDashboard.putBoolean("Shooter Rotate Enable", false);
    SmartDashboard.putNumber("Shooter Set Speed", 0.0);

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
  // public void rotateDis(double _mm){
  //   double mm = k.SHOOTER.ROTATE_DS_OFFSET_DISTANCE_MM + _mm;
  // //  m_Rotate_PID_volts = m_rotatePID.calculate(getDistance(), mm );
  //   // Deal with a linear relation from resting spot to Max distance. 
  //   m_Rotate_FF_volts = k.SHOOTER.ROTATE_DS_FEEDFORWARD_KG * ((k.SHOOTER.ROTATE_DS_MAX + k.SHOOTER.ROTATE_DS_OFFSET_DISTANCE_MM - mm)/k.SHOOTER.ROTATE_DS_MAX); // This is close enough to Cos. PID will deal with the rest.
  //   m_Rotate_PID_volts = MathUtil.clamp(m_Rotate_PID_volts, -3, 3);
  //   //m_rotateMotor.setVoltage(m_Rotate_PID_volts + m_Rotate_FF_volts);
  // }
  /**
   * Rotate the shooter to an angle
   * 
   * @param _angle Degrees
   */
  public void rotate(double _angle) {

    // if (_angle < k.SHOOTER.ROTATE_OFFSET_ANGLE_DEG) {
    //   _angle = k.SHOOTER.ROTATE_OFFSET_ANGLE_DEG;
    // }
    // // calculate the PID value based on the actual angle in degrees and the requested goal to achieve
    // m_Rotate_PID_volts = m_rotatePID.calculate(Math.toRadians(getActualAngle()), Math.toRadians(_angle));
    // // calculate the FeedForward value based on the actual angle and gravity
    // m_Rotate_FF_volts = Math.cos(Math.toRadians(getActualAngle()) * k.SHOOTER.ROTATE_FEEDFORWARD_KG);
    // // Limit the amount the PID can contribute to the output since the FeedForward should do most of the work
    // m_Rotate_PID_volts = MathUtil.clamp(m_Rotate_PID_volts, -2, 2);

    
    // if(SmartDashboard.getBoolean("Shooter Rotate Enable", false)){
    // double volts = SmartDashboard.getNumber("Shooter Test Volts", 0.0);
    // m_rotateMotor.setVoltage(volts);
    // }else {
    // Set the actual voltage to the motor by combining the PID and Feedforward values
  //  m_rotateMotor.setVoltage(m_Rotate_PID_volts + m_Rotate_FF_volts);

    // }

  }
  // public double getDistance(){
  //   return m_distance;
  // }
  // public void setDistance(){
  //   double range = m_distanceSensor.GetRange();
  //   if(range > 0){
  //     m_distanceAverage[m_distanceAvgIndex] = range;
  //     m_distanceAvgIndex++;
  //     m_distanceAvgIndex = m_distanceAvgIndex >= 8 ? 0 : m_distanceAvgIndex;
  //     m_distance = 0;
  //     for(int i = 0; i < 8; i++){
  //       m_distance += m_distanceAverage[i];
  //     }
  //     m_distance = m_distance/   8.0;
  //   }
    
  // }
  // private boolean isRotateInRange(){
  //   double tolerance = m_rotatePID.getPositionTolerance();
  //   double diff = getActualAngle() - GD.G_ShooterAngle;
  //   if(Math.abs(diff) < Math.toDegrees(tolerance)){
  //     return true;
  //   }
  //   return false;
  // }
  public void setFlippersRetracted() {
    GD.G_ShooterIsFlipperRetracted = true;
  }

  public void setFlipperExtended() {
    GD.G_ShooterIsFlipperRetracted = false;
  }

  private void retractFlippers() {

    m_leftServo.set(.6);
    m_rightServo.set(.29);
  }

  private void extendFlippers() {

    m_leftServo.set(0.2);
    m_rightServo.set(0.69);

  }

  public void ShootNote(boolean _shoot) {
    if (_shoot) {
      m_leftServo.set(0.2);
      m_rightServo.set(0.69);
    } else {
      m_leftServo.set(.6);
      m_rightServo.set(.29);
    }
  }

  public double getActualAngle() {
    // double angle_deg = m_rotateMotor.getEncoder().getPosition() / k.SHOOTER.ROTATE_GEAR_RATIO * 360.0
    //     + k.SHOOTER.ROTATE_OFFSET_ANGLE_DEG;
    // return angle_deg;
    return 0.0;
  }

  @Override
  public void periodic() {
   // setDistance();
    spin(SmartDashboard.getNumber("Shooter Set Speed", 0.0));
   // rotateDis(10);
    if (GD.G_ShooterSpeed > 0.1) {
      if (GD.G_ShooterIsFlipperRetracted) {
        retractFlippers();
      } else {
        extendFlippers();
      }
    }
    // This method will be called once per scheduler run
  }
}
