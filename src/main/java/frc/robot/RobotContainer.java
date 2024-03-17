//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot;

import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commandGroups.AutoBlueRightShoot1;
import frc.robot.commandGroups.AutoBlueRightShootPark;
import frc.robot.commandGroups.AutoDoNothing;
import frc.robot.commandGroups.AutoRedLeftShootPark;
import frc.robot.commandGroups.AutoRedLeftShoot1;
import frc.robot.commands.Climber.ClimberDefaultCommand;
import frc.robot.commands.Drive.DrivetrainDefaultCommand;
import frc.robot.commands.Intake.IntakeDefaultCommand;
import frc.robot.commands.Shooter.ShooterDefaultCommand;
import frc.robot.lib.GD;
import frc.robot.lib.ISubsystem;
import frc.robot.lib.k;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * In command-based projects, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Subsystems, commands, and trigger mappings should be defined here.
 * 
 */
public class RobotContainer {

  public static Set<ISubsystem> subsystems = new HashSet<>();

  public static final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final ClimberDefaultCommand m_climberDefaultCommand = new ClimberDefaultCommand(m_climberSubsystem);

  public static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final DrivetrainDefaultCommand m_drivetrainDefaultCommand = new DrivetrainDefaultCommand(m_drivetrainSubsystem);

  public static final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final IntakeDefaultCommand m_intakeDefaultCommand = new IntakeDefaultCommand(m_intakeSubsystem);

  public static final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final ShooterDefaultCommand m_shooterDefaultCommand = new ShooterDefaultCommand(m_shooterSubsystem);

  private Notifier m_telemetry;
  //public static final OrangePi5Vision m_vision = new OrangePi5Vision();
  
  public static final CommandPS5Controller s_driverController = new CommandPS5Controller(k.OI.DRIVER_CONTROLLER_PORT);
  public static final CommandPS5Controller s_operatorController = new CommandPS5Controller(k.OI.OPERATOR_CONTROLLER_PORT);

  SendableChooser<Command> autoChooser = new SendableChooser<>();
  //public static final LEDs m_LEDs = new LEDs(3);
  private void updateDashboard(){
    SmartDashboard.putString("RobotMode", GD.G_RobotMode.toString());
    Iterator<ISubsystem> it = subsystems.iterator();
    while(it.hasNext()){
      it.next().updateDashboard(); // Comment this line out if you want ALL smartdashboard data to be stopped.
    }
  }
  /** This is the constructor for the class. */
  public RobotContainer() {
    m_climberSubsystem.setDefaultCommand(m_climberDefaultCommand);
    m_drivetrainSubsystem.setDefaultCommand(m_drivetrainDefaultCommand);
    m_intakeSubsystem.setDefaultCommand(m_intakeDefaultCommand);
    m_shooterSubsystem.setDefaultCommand(m_shooterDefaultCommand);
    
    // Configure the trigger bindings
    configureBindings();

    // Add all autonomous command groups to the list on the Smartdashboard
    autoChooser.setDefaultOption("Do Nothing", new AutoDoNothing());
    autoChooser.addOption("Blue Right Shoot Park", new AutoBlueRightShootPark(m_drivetrainSubsystem,m_shooterSubsystem,m_intakeSubsystem));
    autoChooser.addOption("Blue Right Shoot 1", new AutoBlueRightShoot1(m_drivetrainSubsystem,m_shooterSubsystem,m_intakeSubsystem));
    autoChooser.addOption("Red Left Shoot 1", new AutoRedLeftShoot1(m_drivetrainSubsystem, m_shooterSubsystem, m_intakeSubsystem));
    autoChooser.addOption("Red Left Shoot Park", new AutoRedLeftShootPark(m_drivetrainSubsystem, m_shooterSubsystem, m_intakeSubsystem));
    SmartDashboard.putData("Autonomous Play",autoChooser);

    // Setup the dashboard notifier that runs at a slower rate than our main robot periodic.
    m_telemetry = new Notifier(this::updateDashboard);
    m_telemetry.startPeriodic(0.1);

    // Configure the trigger bindings
    configureBindings();
  }
 
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   */
  private void configureBindings() {

    k.OI.OPERATOR_INTAKE_SPIN_ON.onTrue(new InstantCommand(m_intakeSubsystem::spinOn, m_intakeSubsystem));
    k.OI.OPERATOR_INTAKE_SPIN_OFF.onTrue(new InstantCommand(m_intakeSubsystem::spinOff, m_intakeSubsystem));
    k.OI.OPERATOR_INTAKE_SPIN_REVERSE.onTrue(new InstantCommand(m_intakeSubsystem::spinReverse, m_intakeSubsystem));
   
    // k.OI.OPERATOR_SHOOTER_ON_HIGH_SPEAKER.onTrue(new InstantCommand(m_shooterSubsystem::setShooterOnHighLong, m_shooterSubsystem ));
    // k.OI.OPERATOR_SHOOTER_ON_HIGH_PODIUM.onTrue(new InstantCommand(m_shooterSubsystem::setShooterOnHighShort, m_shooterSubsystem ));
    // k.OI.OPERATOR_SHOOTER_ON_STAGE.onTrue(new InstantCommand(m_shooterSubsystem::setShooterOnLow, m_shooterSubsystem ));
    k.OI.OPERATOR_SHOOTER_OFF.onTrue(new InstantCommand(m_shooterSubsystem::setShooterOff, m_shooterSubsystem ));


    k.OI.OPERATOR_FLIPPER_EXTEND.onTrue(new InstantCommand(m_shooterSubsystem::setFlipperExtended, m_shooterSubsystem));
    k.OI.OPERATOR_FLIPPER_PRELOAD.onTrue(new InstantCommand(m_shooterSubsystem::setFlipperPreload, m_shooterSubsystem));
    k.OI.OPERATOR_FLIPPER_BACK.onTrue(new InstantCommand(m_shooterSubsystem::setFlippersRetracted, m_shooterSubsystem));
   
    k.OI.DRIVER_RESET_YAW.onTrue(new InstantCommand(m_drivetrainSubsystem::resetYaw, m_drivetrainSubsystem));
    k.OI.DRIVER_DRIVE_MODE_ANGLEFIELDCENTRIC.onTrue(new InstantCommand(m_drivetrainSubsystem::setDriveMode_AngleFieldCentric, m_drivetrainSubsystem));
    k.OI.DRIVER_DRIVE_MODE_FIELDCENTRIC.onTrue(new InstantCommand(m_drivetrainSubsystem::setDriveMode_FieldCentric, m_drivetrainSubsystem));
    k.OI.DRIVER_DRIVE_MODE_ROBOTCENTRIC.onTrue(new InstantCommand(m_drivetrainSubsystem::setDriveMode_RobotCentric, m_drivetrainSubsystem));
    k.OI.DRIVER_DRIVE_MODE_SPEED_HI.onTrue(new InstantCommand(m_drivetrainSubsystem::setDriveSpeedHI, m_drivetrainSubsystem));
    k.OI.DRIVER_DRIVE_MODE_SPEED_LOW.onTrue(new InstantCommand(m_drivetrainSubsystem::setDriveSpeedLOW, m_drivetrainSubsystem));

  }
  /**
   * @return the command to run in autonomous routine
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
