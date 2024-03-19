//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.lib.Swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.GD;
import frc.robot.lib.k;

public class SwerveDrive {
    private int m_moduleCount;
    private SwerveModule[] m_modules;
    private Pigeon2 m_pigeon2;
    private SwerveDriveKinematics m_kinematics;
    private SwerveDriveOdometry m_odometry;
    private SwerveModulePosition[] m_modulePositions;
    private Translation2d[] m_moduleLocations;
    private OdometryThread m_odometryThread;
    private Field2d m_field;
    private PIDController m_turnPid;



    /* Perform swerve module updates in a separate thread to minimize latency */
    private class OdometryThread extends Thread {

        public int SuccessfulDaqs = 0;
        public int FailedDaqs = 0;


        public OdometryThread() {
            super();
            
        }

        @Override
        public void run() {

            /* Run as fast as possible, our signals will control the timing */
            while (true) {
                /* Now update odometry */
                m_modulePositions[0] = m_modules[0].getPosition(true);
                m_modulePositions[1] = m_modules[1].getPosition(true);
                m_modulePositions[2] = m_modules[2].getPosition(true);

                // Assume Pigeon2 is flat-and-level so latency compensation can be performed
                double yawDegrees = m_pigeon2.getYaw().getValueAsDouble();//BaseStatusSignal.getLatencyCompensatedValue(m_pigeon2.getYaw(), m_pigeon2.getAngularVelocityZDevice());

                GD.G_RobotPose = m_odometry.update(Rotation2d.fromDegrees(yawDegrees), m_modulePositions);
                m_field.setRobotPose(m_odometry.getPoseMeters());
                try{
                    Thread.sleep(5);
                }catch(InterruptedException e){
                    System.out.println(e.getMessage());
                }
            }
        }
    }
   
    public SwerveDrive() {
        //SwerveModuleConstantsCreator m_constantsCreator = new SwerveModuleConstantsCreator();
        SwerveModuleConstants m_frontRight = new SwerveModuleConstants( 
            "fr",
            13, true,
            23, true, 
            3, 0.03637,
            k.DRIVEBASE.WHEEL_BASE_X_m / 2.0, -k.DRIVEBASE.WHEEL_BASE_Y_m / 2.0);

        SwerveModuleConstants m_frontLeft = new SwerveModuleConstants(
            "fl",
            12, false,
            22, true,  
            2,0.2234, 
            k.DRIVEBASE.WHEEL_BASE_X_m / 2.0, k.DRIVEBASE.WHEEL_BASE_Y_m / 2.0);
        SwerveModuleConstants m_back = new SwerveModuleConstants(
            "b",
            11, false,  
            21, true, 
            1,-0.2324, 
            -k.DRIVEBASE.WHEEL_BASE_X_m / 2.0, 0.0);

        initialize(m_frontLeft, m_frontRight, m_back);    
    }

    public void initialize(SwerveModuleConstants... _modules){
        m_moduleCount = _modules.length;
        m_pigeon2 = new Pigeon2(k.CANIVORE_IDS.PIGEON2_CANID, k.CANIVORE_IDS.NAME);
        
        m_modules = new SwerveModule[m_moduleCount];
        m_modulePositions = new SwerveModulePosition[m_moduleCount];
        m_moduleLocations = new Translation2d[m_moduleCount];
 
        for(int i = 0; i < m_moduleCount; i++)  {
            m_modules[i] = new SwerveModule(_modules[i]);
            m_moduleLocations[i] = new Translation2d(_modules[i].m_locationX_m, _modules[i].m_locationY_m);
            m_modulePositions[i] = m_modules[i].getPosition(true);
        }

        m_kinematics = new SwerveDriveKinematics(m_moduleLocations);
        m_odometry = new SwerveDriveOdometry(m_kinematics, m_pigeon2.getRotation2d(), getSwervePositions());
        m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);

        m_turnPid = new PIDController(k.DRIVEBASE.TURN_KP, k.DRIVEBASE.TURN_KI, k.DRIVEBASE.TURN_KD);
        m_turnPid.enableContinuousInput(-Math.PI, Math.PI);
        m_turnPid.setTolerance(Math.toRadians(.1),1);
        
        SmartDashboard.putData("Robot Turn PID",m_turnPid);
        m_odometryThread = new OdometryThread();
        
        m_odometryThread.start();
    }
    private SwerveModulePosition[] getSwervePositions() {
        return m_modulePositions;
    }

    public void driveRobotCentric(ChassisSpeeds _speeds) {
        var swerveStates = m_kinematics.toSwerveModuleStates(_speeds);
        setSwerveModules(swerveStates,true, true);
    }

    public void driveFieldCentric(ChassisSpeeds _speeds) {
        var roboCentric = ChassisSpeeds.fromFieldRelativeSpeeds(_speeds, m_pigeon2.getRotation2d());
        var swerveStates = m_kinematics.toSwerveModuleStates(roboCentric);
        setSwerveModules(swerveStates, true, true);
    }

    public void driveAngleFieldCentric(double _xSpeeds, double _ySpeeds, Rotation2d _targetAngle, boolean _enableSteer, boolean _enableDrive) {
        var currentAngle = m_pigeon2.getRotation2d();
        double rotationalSpeed = m_turnPid.calculate(currentAngle.getRadians(), _targetAngle.getRadians());
        rotationalSpeed = MathUtil.applyDeadband(rotationalSpeed, 0.01);
        var roboCentric = ChassisSpeeds.fromFieldRelativeSpeeds(_xSpeeds, _ySpeeds, rotationalSpeed, m_pigeon2.getRotation2d());
        var swerveStates = m_kinematics.toSwerveModuleStates(roboCentric);
        setSwerveModules(swerveStates, _enableSteer, _enableDrive);
    }
    public void setSwerveModules(SwerveModuleState[] _states, boolean _enableSteer, boolean _enableDrive){
        m_modules[0].setDesiredState(_states[0], _enableSteer, _enableDrive);
        m_modules[1].setDesiredState(_states[1], _enableSteer, _enableDrive);
        m_modules[2].setDesiredState(_states[2], _enableSteer, _enableDrive);
    }
    public void driveStopMotion() {
        m_modules[0].setDesiredState(new SwerveModuleState(0, new Rotation2d(Math.toRadians(-45))), true, true); // FR
        m_modules[1].setDesiredState(new SwerveModuleState(0, new Rotation2d(Math.toRadians(45))), true, true); // FL
        m_modules[2].setDesiredState(new SwerveModuleState(0, new Rotation2d(Math.toRadians(90))), true, true); // B
    }

    public void resetYaw() {
        m_pigeon2.setYaw(0);
    }
    public void resetPose(){
        m_odometry.resetPosition(m_pigeon2.getRotation2d(), getSwervePositions(), new Pose2d(3.55,4.28,new Rotation2d()));
    }
    public Pose2d getPoseMeters() {
        return m_odometry.getPoseMeters();
    }

    public double getSuccessfulDaqs() {
        return m_odometryThread.SuccessfulDaqs;
    }

    public double getFailedDaqs() {
        return m_odometryThread.FailedDaqs;
    }

    public double getRobotYaw() {
        return m_pigeon2.getYaw().getValueAsDouble();
    }

    public boolean isTurnPIDatSetpoint() {
        return m_turnPid.atSetpoint();
    }
    /* Put smartdashboard calls in separate thread to reduce performance impact */
    public void updateDashboard() {
       for (int i = 0; i < m_moduleCount; ++i) {
           m_modules[i].updateDashboard();
       }
       SmartDashboard.putNumber("Robot Pose X", GD.G_RobotPose.getX());
       SmartDashboard.putNumber("Robot Pose Y", GD.G_RobotPose.getY());
       SmartDashboard.putNumber("Robot Pose Ang", GD.G_RobotPose.getRotation().getDegrees());

    }
}
