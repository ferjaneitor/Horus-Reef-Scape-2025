// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Drive.SwerveJoyStickcmd;
import frc.robot.LimeLight.LimeLightSubsystem;
import frc.robot.Swerve.SwerveSubsystem;
import frc.robot.constants.OIConstants;

/*
 * @Author: Juan Felipe Zepeda del Toro
 * @Author: Fernando Joel Cruz Briones
 * Version 1.0
 */

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  @SuppressWarnings("unused")
  private final LimeLightSubsystem LimeLightSubsystem = new LimeLightSubsystem();

  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
  @SuppressWarnings("unused")
  private final Joystick AddOnsJoystick = new Joystick(OIConstants.kAddOnsControllerPort);

  public RobotContainer() {

    swerveSubsystem.setDefaultCommand(new SwerveJoyStickcmd(
      swerveSubsystem, 
      () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis), 
      () -> driverJoystick.getRawAxis(OIConstants.kDriverYAxis), 
      () -> driverJoystick.getRawAxis(OIConstants.kDriverZAxis), 
      () -> driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)
    ));

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void mResetEncoders() {
    swerveSubsystem.resetEncoders();
  }
  
}
