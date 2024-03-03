//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.EDriveMode;
import frc.robot.lib.GD;
import frc.robot.lib.ISubsystem;
import frc.robot.lib.k;
import frc.robot.lib.Swerve.SwerveDrive;

public class DrivetrainSubsystem extends SubsystemBase implements ISubsystem{
  public SwerveDrive m_robotDrive;
  public EDriveMode m_driveMode = EDriveMode.ANGLE_FIELD_CENTRIC;
  
  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
     m_robotDrive = new SwerveDrive();
     
    initialize(); 
  }
  public void initialize(){
    RobotContainer.subsystems.add(this);
  }

  public void driveStopMotion(){
    m_robotDrive.driveStopMotion();
  }
  public void driveRobotCentric(double _driveAngle, double _robotAngle, double _speed){
    double y = Math.sin(Units.degreesToRadians(_driveAngle)) * _speed;
    double x = Math.cos(Units.degreesToRadians(_driveAngle)) * _speed;
  }
  public void driveRobotCentric(ChassisSpeeds _speeds){
    m_robotDrive.driveRobotCentric(_speeds);
  }

  public void driveFieldCentric(ChassisSpeeds _speeds){
    m_robotDrive.driveFieldCentric(_speeds);
  }

  public void driveAngleFieldCentric(double _x, double _y, Rotation2d _targetAngle){
    m_robotDrive.driveAngleFieldCentric(_x, _y, _targetAngle);
  }

  public void drivePolarFieldCentric(double _driveAngle_deg, double _robotAngle_deg, double _speed, boolean _toNote){
    if(_toNote){
      double yaw = RobotContainer.m_vision.getNoteYaw();
      if(yaw < 90){
        _driveAngle_deg += yaw;
        _robotAngle_deg += yaw;
      }
    }
    double y = Math.sin(Units.degreesToRadians(_driveAngle_deg)) * _speed;
    double x = Math.cos(Units.degreesToRadians(_driveAngle_deg)) * _speed;
    driveAngleFieldCentric(x, y, new Rotation2d(Math.toRadians(_robotAngle_deg)));
  }

  public void changeDriveMode(){
    switch(m_driveMode){
      case FIELD_CENTRIC:
        m_driveMode = EDriveMode.ANGLE_FIELD_CENTRIC;
      break;
      case ANGLE_FIELD_CENTRIC:
        m_driveMode = EDriveMode.ROBOT_CENTRIC;
      break;
      case ROBOT_CENTRIC:
        m_driveMode = EDriveMode.FIELD_CENTRIC;
      break;
      default:
        m_driveMode = EDriveMode.ROBOT_CENTRIC;
      break;
    }
  }

  public EDriveMode getDriveMode(){
    return m_driveMode;
  }

  public double getRobotAngle(){
    return m_robotDrive.getRobotYaw();
  }

  public void resetYaw(){
    m_robotDrive.resetYaw();
  }

  public void setLastTargetAngle(Rotation2d _targetAngle){
    GD.G_RobotTargetAngle.setTargetAngle(_targetAngle.getDegrees());
  }
  public void resetPose(){
    m_robotDrive.resetPose();
  }

  public void updateDashboard(){
    SmartDashboard.putString(k.DRIVE.T_DRIVER_MODE, m_driveMode.toString());
    SmartDashboard.putString("Robot Target Angle", GD.G_RobotTargetAngle.getTargetAngle().toString());
    SmartDashboard.putNumber("Robot Angle", getRobotAngle());
    SmartDashboard.putNumber("TargetYaw", RobotContainer.m_vision.getNoteYaw());
    //m_robotDrive.updateDashboard();
  }
  @Override
  public void periodic() {

    RobotContainer.m_vision.findNote();

  }
}
