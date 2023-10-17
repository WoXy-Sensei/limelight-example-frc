package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.lib.utils.PIDFController;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.SwerveConstants;


public class FollowApriltagCommand extends CommandBase {
    private double  m_distanceTarget;
    private double rotOut, driOut, tx, gyroAngle;
    private Integer targetPipeline;
    
    PIDFController angleController = new PIDFController(SwerveConstants.LLangleKP, SwerveConstants.LLangleKI, SwerveConstants.LLangleKD, SwerveConstants.LLangleKFF);
    PIDController driveController = new PIDFController(SwerveConstants.LLdriveKP, SwerveConstants.LLdriveKI, SwerveConstants.LLdriveKD, SwerveConstants.LLdriveKFF);
    PIDController strafeController = new PIDController(SwerveConstants.LLstrafeKP, SwerveConstants.LLstrafeKI, SwerveConstants.LLstrafeKD);
    
    Swerve m_swerve;
    LimelightSubsystem m_limelight;
    
    public FollowApriltagCommand(double distance ,Integer targetPipeline,Swerve swerve, LimelightSubsystem limelight)  {
        addRequirements(swerve);
        addRequirements(limelight);
        this.m_swerve = swerve;
        this.m_limelight = limelight;
        this.m_distanceTarget = distance;
        this.targetPipeline = targetPipeline;

        strafeController.setTolerance(5);
        driveController.setTolerance(5);
        angleController.setTolerance(5);
    }

    @Override
    public void initialize() {
        strafeController.reset();
        driveController.reset();
        angleController.reset();
    }

    @Override
    public void execute() {
    
        if (m_limelight.getIsTargetFound()) {
            if (m_limelight.getPipelineInt() == targetPipeline){
                
                gyroAngle = m_swerve.getHeadingNavX();
                tx = m_limelight.getdegRotationToTarget();

                rotOut = -1 * angleController.calculate(gyroAngle, gyroAngle+tx);
                driOut = -0.45 * driveController.calculate(m_limelight.getDistanceLLToGoal(),m_distanceTarget);

               
                m_swerve.drive(new Translation2d(driOut, 0), rotOut, false, true);

            }
        }
       
    }
    @Override
    public void end(boolean interrupted) {
        m_swerve.stopModules();
        m_limelight.setPipelineInt(LimelightConstants.pipeNu_normal);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
