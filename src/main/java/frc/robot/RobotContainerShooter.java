// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;
import org.littletonrobotics.junction.Logger;

/**
 * Minimal container for shooter-only testing.
 * Avoids constructing other subsystems so missing CAN devices won't fault.
 */
public class RobotContainerShooter {

  private final CommandXboxController driverXbox = new CommandXboxController(0);
  private final CommandXboxController operatorXbox = new CommandXboxController(1);

  private final Shooter m_shooter = new Shooter();

  public RobotContainerShooter() {
    configureBindings();
  }

  private void configureBindings() {
    // Shooter commands
    driverXbox.y().whileTrue(m_shooter.shootFuelCommand());
    driverXbox.x().whileTrue(m_shooter.SpeedUpShooterCommand());
    driverXbox.a().whileTrue(m_shooter.RotateHoodDownCommand());
    driverXbox.b().whileTrue(m_shooter.RotateHoodUpCommand());

    // SysId on operator controller
    operatorXbox.a().whileTrue(m_shooter.sysIdQuasistaticForward());
    operatorXbox.b().whileTrue(m_shooter.sysIdQuasistaticReverse());
    operatorXbox.x().whileTrue(m_shooter.sysIdDynamicForward());
    operatorXbox.y().whileTrue(m_shooter.sysIdDynamicReverse());
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }

  public void setMotorBrake(boolean brake) {
    // No drivebase in this container.
  }

  public void logControllerInputs() {
    Logger.recordOutput("Input/Driver/LeftX", driverXbox.getLeftX());
    Logger.recordOutput("Input/Driver/LeftY", driverXbox.getLeftY());
    Logger.recordOutput("Input/Driver/RightX", driverXbox.getRightX());
    Logger.recordOutput("Input/Driver/RightY", driverXbox.getRightY());
    Logger.recordOutput("Input/Driver/LeftTrigger", driverXbox.getLeftTriggerAxis());
    Logger.recordOutput("Input/Driver/RightTrigger", driverXbox.getRightTriggerAxis());

    Logger.recordOutput("Input/Operator/LeftX", operatorXbox.getLeftX());
    Logger.recordOutput("Input/Operator/LeftY", operatorXbox.getLeftY());
    Logger.recordOutput("Input/Operator/RightX", operatorXbox.getRightX());
    Logger.recordOutput("Input/Operator/RightY", operatorXbox.getRightY());
    Logger.recordOutput("Input/Operator/LeftTrigger", operatorXbox.getLeftTriggerAxis());
    Logger.recordOutput("Input/Operator/RightTrigger", operatorXbox.getRightTriggerAxis());
  }
}
