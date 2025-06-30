// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.aruco.Aruco;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DriveCommands.DriveCommands;
import frc.robot.DriveTrain.Swerve;
import frc.robot.DriveTrain.Swerve.SwervePathConstraints;
import lib.ForgePlus.NetworkTableUtils.NTPublisher;
import lib.ForgePlus.NetworkTableUtils.NTSendableChooser;

public class RobotContainer {

  private final Swerve chassis;

  private final CommandXboxController driver = new CommandXboxController(0);

  private PathPlannerAuto lateral;
  private PathPlannerAuto mover;
  private PathPlannerAuto inicio;

  public NTSendableChooser<Command> autoChooser = new NTSendableChooser<>(NTPublisher.ROBOT, "AutoSelector");

  public RobotContainer() {

    chassis = new Swerve(SwervePathConstraints.kNormal);

    NTPublisher.publish("Joysticks", "Driver1", driver);
  
    lateral = new PathPlannerAuto("lat");
    mover = new PathPlannerAuto("mover");
    inicio = new PathPlannerAuto("inicio");

    autoChooser.setDefault("mover", mover).add("inicio", inicio).add("lat", lateral).publish();
  
    configureBindings();

  }

  private void configureBindings() {
    
    chassis.setDefaultCommand(DriveCommands.joystickDrive(chassis, ()-> -driver.getLeftY(), ()-> -driver.getLeftX(), ()-> -driver.getRightX()));

    driver.b().whileTrue(Commands.runOnce(()-> {chassis.o().setPose(new Pose2d(3.24, 4.04,new Rotation2d()));}, chassis));
    //driver.x().whileTrue(chassis.getPathFinder().toPoseCommand(new Pose2d(0.53, 3.6, Rotation2d.kZero)));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}