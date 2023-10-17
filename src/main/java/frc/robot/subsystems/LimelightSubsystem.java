package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.vision.LimeLight;

public class LimelightSubsystem extends SubsystemBase {
    
    private final LimeLight m_limelight;
    
    public LimelightSubsystem() {
        m_limelight = new LimeLight();
    }

    public double getDistanceLLToGoal() {
        return m_limelight.getDistanceLLToGoal();
    }

    public double getdegRotationToTarget() {
        return m_limelight.getdegRotationToTarget();
    }

    public boolean getIsTargetFound() {
        return m_limelight.getIsTargetFound();
    }

    public int getPipelineInt() {
        return m_limelight.getPipelineInt();
    }

    public void setPipelineInt(int pipeline) {
        RobotState.setPipeline(pipeline);
        m_limelight.setPipeline(pipeline);
    }


    @Override
    public void periodic() { 
        SmartDashboard.putNumber("Limelight Pipeline Nu", m_limelight.getPipelineInt());  
        SmartDashboard.putNumber("getDistanceLLToGoal", m_limelight.getDistanceLLToGoal());
        SmartDashboard.putNumber("getdegRotationToTarget", m_limelight.getdegRotationToTarget());

    }
}
