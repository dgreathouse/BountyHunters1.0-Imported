//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.lib.GD;
import frc.robot.lib.RobotMode;
import frc.robot.lib.k;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DrivetrainDefaultCommand extends Command {

  private DrivetrainSubsystem m_drive;
  ChassisSpeeds m_speeds = new ChassisSpeeds();
  SlewRateLimiter m_stickLimiterLX = new SlewRateLimiter(6);
  SlewRateLimiter m_stickLimiterLY = new SlewRateLimiter(6);
  SlewRateLimiter m_stickLimiterRX = new SlewRateLimiter(6);
  double m_vYaw = 0;
  /** Creates a new DrivetrainDefaultCommand. */
  public DrivetrainDefaultCommand(DrivetrainSubsystem _drivetrain) {
    m_drive = _drivetrain;
    this.setName("DriveDefaultCommand");
    addRequirements(m_drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Set local variables to game thumbstick axis values
    double leftYRaw = -RobotContainer.s_driverController.getLeftY();
    double leftXRaw = -RobotContainer.s_driverController.getLeftX();
    double rightXRaw = -RobotContainer.s_driverController.getRightX();
    double rightYRaw = -RobotContainer.s_driverController.getRightY();

    // Limit the inputs for a deadband related to the joystick
    double leftYFiltered = MathUtil.applyDeadband(leftYRaw, 0.08, 1.0);
    double leftXFiltered = MathUtil.applyDeadband(leftXRaw, 0.08, 1.0);
    // double rightYFiltered = MathUtil.applyDeadband(rightYRaw, 0.15, 1.0);
    double rightXFiltered = MathUtil.applyDeadband(rightXRaw, 0.15, 1.0);

    leftXFiltered = m_stickLimiterLX.calculate(leftXFiltered);
    leftYFiltered = m_stickLimiterLY.calculate(leftYFiltered);
    rightXFiltered = m_stickLimiterRX.calculate(rightXFiltered);

    // Set the class variable ChassisSpeeds to the local variables in their
    // appropriate units.
    m_speeds.vxMetersPerSecond = leftYFiltered * k.DRIVE.MAX_VELOCITY_MeterPerSec;
    m_speeds.vyMetersPerSecond = leftXFiltered * k.DRIVE.MAX_VELOCITY_MeterPerSec;
    m_speeds.omegaRadiansPerSecond = rightXFiltered * k.DRIVE.MAX_ANGULAR_VELOCITY_RadianPerSec;

    // If correct button is pressed, set the robot angle, shooter angle and shooter
    // speed.
    setShotData(rightXRaw, rightYRaw);

    m_vYaw = RobotContainer.m_vision.getNoteYaw();
    
    if (GD.G_RobotMode != RobotMode.AUTONOMOUS_PERIODIC) {
      // Call the appropriate drive mode. Selected by the driver controller Options button.
      
      // if (m_vYaw < 90 && RobotContainer.s_operatorController.square().getAsBoolean()) {
      //   m_drive.drivePolarFieldCentric(GD.G_RobotTargetAngle.getTargetAngle().getDegrees(), GD.G_RobotTargetAngle.getTargetAngle().getDegrees(), 3, true);
        
      //} else {
        switch (m_drive.getDriveMode()) {
          case FIELD_CENTRIC:
            m_drive.driveFieldCentric(m_speeds);
            break;
          case ROBOT_CENTRIC:
            m_drive.driveRobotCentric(m_speeds);
            break;
          case ANGLE_FIELD_CENTRIC:
            m_drive.driveAngleFieldCentric(m_speeds.vxMetersPerSecond, m_speeds.vyMetersPerSecond, GD.G_RobotTargetAngle.getTargetAngle());
            break;
          default:
            break;
        }
     // }
    } else {
      m_drive.driveStopMotion();
    }
  }

  /**
   * If right stick X,Y Hyp is > 0.8 then set the target angle
   * Reset the Shooter angle and speed to default if button not pressed.
   * 
   * Set the following data if the appropriate buttons are pressed
   * Robot Angle to turn to
   * Shooter angle to move to
   * Shooter speed to run at.
   */
  private void setShotData(double _x, double _y) {
    // set the target angle only if the x,y hyp are > deadband
    GD.G_RobotTargetAngle.setTargetAngle(_x, _y);
    // Assume blue
    double allianceSign = 1.0;
    allianceSign = GD.G_Alliance == Alliance.Red ? -allianceSign : allianceSign;

    // if x,y hyp > deadband reset the shooter angle and speed
    // if (GD.G_RobotTargetAngle.getHyp() > k.DRIVE.TARGET_ANGLE_DEADBAND) {
    //  // GD.G_ShooterAngle = k.SHOOTER.ROTATE_OFFSET_ANGLE_DEG;
    //   GD.G_ShooterSpeed = 0.0;
    //   GD.G_Intake_Speed = 0.0;
    // }
    // Handle target angle buttons if pressed to set angles and speeds
    if (k.OI.DRIVER_ENABLE_LEFT_TRIGGERS.getAsBoolean()) {
      if (k.OI.DRIVER_SHOT_POSITION_L1.getAsBoolean()) {
        GD.G_RobotTargetAngle.setTargetAngle(50);
      } else if (k.OI.DRIVER_SHOT_POSITION_L2.getAsBoolean()) {
        GD.G_RobotTargetAngle.setTargetAngle(50);
      } else if (k.OI.DRIVER_SHOT_POSITION_L3.getAsBoolean()) {
        GD.G_RobotTargetAngle.setTargetAngle(-30);
      } else if (k.OI.DRIVER_SHOT_POSITION_L4.getAsBoolean()) {
        GD.G_RobotTargetAngle.setTargetAngle(-50);
      }
    } else if (k.OI.DRIVER_ENABLE_RIGHT_TRIGGERS.getAsBoolean()){
      if (k.OI.DRIVER_SHOT_POSITION_R1.getAsBoolean()) { 
        GD.G_RobotTargetAngle.setTargetAngle(-27 * allianceSign);
      } else if (k.OI.DRIVER_SHOT_POSITION_R2.getAsBoolean()) { 
        GD.G_RobotTargetAngle.setTargetAngle(45);
      } else if (k.OI.DRIVER_SHOT_POSITION_R3.getAsBoolean()) { 
        GD.G_RobotTargetAngle.setTargetAngle(0);
      } else if (k.OI.DRIVER_SHOT_POSITION_R4.getAsBoolean()) { 
        GD.G_RobotTargetAngle.setTargetAngle(-45);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("ChassisSpeeds(Command) MPS(X)", m_speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeeds(Command) MPS(Y)", m_speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeeds(Command) Deg/Sec", m_speeds.omegaRadiansPerSecond * k.CONVERT.RADIANS_TO_DEGREES);
    SmartDashboard.putNumber("vYaw", m_vYaw);
  }
}
