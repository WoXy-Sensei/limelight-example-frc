// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.commands.limelight.FollowApriltagCommand;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.subsystems.*;


public class RobotContainer {

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private final Swerve m_Swerve = new Swerve();
  private final LimelightSubsystem m_limelight = new LimelightSubsystem();

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    m_Swerve.setDefaultCommand(
        new TeleopSwerve(
            m_Swerve,
            () -> MathUtil.applyDeadband(m_driverController.getLeftY(), SwerveConstants.stickDeadband), // translationAxis
            () -> MathUtil.applyDeadband(m_driverController.getLeftX(), SwerveConstants.stickDeadband), // strafeAxis
            () -> MathUtil.applyDeadband(-m_driverController.getRightX(), SwerveConstants.stickDeadband), // rotationAxis
            () -> m_driverController.leftBumper().getAsBoolean()));

    configureBindings();
  }

  private void configureBindings() {
    m_driverController.start().onTrue(Commands.runOnce(() -> m_Swerve.zeroGyro()));

    m_driverController.x()
        .toggleOnTrue(Commands.runOnce(() -> m_limelight.setPipelineInt(LimelightConstants.pipeNu_hp45_april))
            .andThen(new FollowApriltagCommand(100,LimelightConstants.pipeNu_hp45_april, m_Swerve, m_limelight)));
  }


  
}
