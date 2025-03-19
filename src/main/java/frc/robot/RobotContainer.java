// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final CoralSubsystem m_coralSubSystem = new CoralSubsystem();
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
  private final SendableChooser<Command> autoChooser;
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    // Coral Subsystem Commands
    NamedCommands.registerCommand("runIntake", m_coralSubSystem.runIntakeHoldCommand());
    NamedCommands.registerCommand("reverseIntake", m_coralSubSystem.reverseIntakeHoldCommand());
    NamedCommands.registerCommand("setFeederStation", m_coralSubSystem.setSetpointCommand(Setpoint.kFeederStation));
    NamedCommands.registerCommand("setLevel2", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2));
    NamedCommands.registerCommand("setLevel3", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3));
    NamedCommands.registerCommand("setLevel4", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel4));

    // Algae Subsystem Commands
    NamedCommands.registerCommand("runBallIntake", m_algaeSubsystem.runIntakeCommand());
    NamedCommands.registerCommand("reverseBallIntake", m_algaeSubsystem.reverseIntakeCommand());
    NamedCommands.registerCommand("stowBallIntake", m_algaeSubsystem.stowCommand());

    // Drive Subsystem Commands
    NamedCommands.registerCommand("setX", m_robotDrive.setXCommand());
    NamedCommands.registerCommand("zeroHeading", m_robotDrive.zeroHeadingCommand());

    configureButtonBindings();
    UsbCamera CoralCam = CameraServer.startAutomaticCapture();
    UsbCamera ElevatorCam = CameraServer.startAutomaticCapture();
    CoralCam.setResolution(640, 480);
    ElevatorCam.setResolution(640, 480);
    autoChooser = AutoBuilder.buildAutoChooser("None");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(
                    m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(
                    m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(
                    m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
    m_algaeSubsystem.setDefaultCommand(m_algaeSubsystem.idleCommand());
  }

  private void configureButtonBindings() {
    // Left Stick Button -> Set swerve to X
    m_driverController.leftStick().whileTrue(m_robotDrive.setXCommand());

    // Left Bumper -> Run tube intake
    m_driverController.leftBumper().whileTrue(m_coralSubSystem.runIntakeCommand());

    // Right Bumper -> Run tube intake in reverse
    m_driverController.rightBumper().whileTrue(m_coralSubSystem.reverseIntakeCommand());

    // B Button -> Elevator/Arm to human player position, set ball intake to stow
    // when idle
    m_driverController
        .b()
        .onTrue(
            m_coralSubSystem
                .setSetpointCommand(Setpoint.kFeederStation)
                .alongWith(m_algaeSubsystem.stowCommand()));

    // A Button -> Elevator/Arm to level 2 position
    m_driverController.a().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2));

    // X Button -> Elevator/Arm to level 3 position
    m_driverController.x().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3));

    // Y Button -> Elevator/Arm to level 4 position
    m_driverController.y().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel4));

    // Right Trigger -> Run ball intake, set to leave out when idle
    m_driverController
        .rightTrigger(OIConstants.kTriggerButtonThreshold)
        .whileTrue(m_algaeSubsystem.runIntakeCommand());

    // Left Trigger -> Run ball intake in reverse, set to stow when idle
    m_driverController
        .leftTrigger(OIConstants.kTriggerButtonThreshold)
        .whileTrue(m_algaeSubsystem.reverseIntakeCommand());

    // Start Button -> Zero swerve heading
    m_driverController.start().onTrue(m_robotDrive.zeroHeadingCommand());

    m_driverController.povDown().onTrue(m_algaeSubsystem.stowCommand());
  }

  public double getSimulationTotalCurrentDraw() {
    // for each subsystem with simulation
    return m_coralSubSystem.getSimulationCurrentDraw()
        + m_algaeSubsystem.getSimulationCurrentDraw();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
