// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DriveCommands.DriveCommands;
import frc.robot.DriveTrain.Swerve;
import frc.robot.DriveTrain.Swerve.SwervePathConstraints;
import lib.ForgePlus.NetworkTableUtils.NTPublisher;
import lib.ForgePlus.NetworkTableUtils.NTSendableChooser;
import lib.ForgePlus.RobotState.RobotLifeCycle;

public class RobotContainer {

  private final Swerve chassis;

  private final CommandXboxController driver = new CommandXboxController(0);

  private final List<RobotLifeCycle> lifecycleSubsystems;

  private PathPlannerAuto lateral;
  private PathPlannerAuto mover;
  private PathPlannerAuto inicio;

  public NTSendableChooser<Command> autoChooser = new NTSendableChooser<>(NTPublisher.ROBOT, "AutoSelector");

  public RobotContainer() {

    chassis = new Swerve(SwervePathConstraints.kNormal);

    lifecycleSubsystems = List.of(RobotState.getInstance()); //Add more subsystems here that implements RobotLifeCycle class

    NTPublisher.publish("Joysticks", "Driver1", driver);
  
    lateral = new PathPlannerAuto("lat");
    mover = new PathPlannerAuto("mover");
    inicio = new PathPlannerAuto("inicio");

    autoChooser.setDefault("mover", mover).
    add("inicio", inicio).
    add("lat", lateral).
    publish();

    configureBindings();

  }

  private void configureBindings() {
    
    chassis.setDefaultCommand(DriveCommands.joystickDrive(chassis, ()-> -driver.getLeftY(), ()-> -driver.getLeftX(), ()-> -driver.getRightX()));

    driver.x().whileTrue(chassis.getPathFinder().toPoseCommand(new Pose2d(3.177, 4.031, Rotation2d.kZero)));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public List<RobotLifeCycle> getLifeCycle() {
    return lifecycleSubsystems;
  }
}