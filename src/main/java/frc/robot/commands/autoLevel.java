// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.driveTrain;

public class autoLevel extends CommandBase {
  /** Creates a new autoLevel. */
  PIDController angController;
  public autoLevel() {
    // Use addRequirements() here to declare subsystem dependencies.
    angController = new PIDController(.01f/8f, 0, 0);
    addRequirements(driveTrain.getInstance());
    angController.setSetpoint(0);
    angController.setTolerance(3);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   double output = angController.calculate(driveTrain.getInstance().getRoll());
    output = MathUtil.clamp(output, -.02, .02);
    driveTrain.getInstance().move(output, output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return angController.atSetpoint();
  }
}
