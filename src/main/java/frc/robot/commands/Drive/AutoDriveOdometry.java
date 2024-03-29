// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.GD;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoDriveOdometry extends Command {
  DrivetrainSubsystem m_drive;
  Pose2d m_poseDesired;
  PIDController m_drivePID;
  double m_currentSpeed = 0;
  double m_driveSpeed;
  boolean m_isFinished = false;
  Timer m_timer = new Timer();
  double m_rampUpTime = 0.5;

  /**
   * 
   * @param _drive  The drive subsystem for requirements
   * @param _pose   the desired Pose2d to drive to
   * @param _speed  The speed in MPS to drive at.
   */
  public AutoDriveOdometry(DrivetrainSubsystem _drive, Pose2d _pose, double _speed) {
    m_drive = _drive;
    m_poseDesired = _pose;
    m_driveSpeed = _speed;
    addRequirements(m_drive);// here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // The drive PID based on distance to target and output of MPS.
    m_drivePID = new PIDController(3.0, 0.750, 0.0);
    m_drivePID.setTolerance(0.05,0.2);// 0.05M or 2 inches
    
    m_timer.start();
    m_poseDesired = new Pose2d(m_poseDesired.getX(), m_poseDesired.getY() * GD.G_AllianceSign, new Rotation2d(m_poseDesired.getRotation().getRadians()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d trajectory = m_poseDesired.relativeTo(GD.G_RobotPose);               // Get a Trajectory to the desired Pose relative to the current pose.
    
    double robotAngle = m_poseDesired.getRotation().getDegrees();               // The angle of the robot from the desired pose angle
    double targetAngle = trajectory.getTranslation().getAngle().getDegrees();   // The drive angle to the new pose.
    double targetDistance = m_poseDesired.getTranslation().getDistance(GD.G_RobotPose.getTranslation());   // The drive distance to the new pose.
    double speed = m_drivePID.calculate(0,targetDistance);          // Speed from PID based on 0 target and a changing distance as the robot moves at a target angle towards the destination. Output is speed MPS targetDistance is in Meters
                                                                                // targetDistance is the new setpoint since we are moving to 0 for the target distance. This seams a little reversed but should work.
                                                                           
    speed = rampUpValue(speed, m_rampUpTime);                                   // Ramp up the speed so a sudden step in voltage does not happen
    speed = MathUtil.clamp(speed, -m_driveSpeed, m_driveSpeed);                 // Clamp the speed to the maximum requested
    m_drive.drivePolarFieldCentric(targetAngle, robotAngle, speed, true, true); // Drive at a angle and speed and let the SwerveDrive move to the a robot angle.
    
    
  }

  private double rampUpValue(double _val, double rampTime_sec){
    double currentTime_sec = m_timer.get();                                        
    if(currentTime_sec < rampTime_sec){
      _val = _val * currentTime_sec / rampTime_sec;
    }
    return _val;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_drivePID.atSetpoint()){
      m_isFinished = true;
    }
    return m_isFinished;
  }
}
