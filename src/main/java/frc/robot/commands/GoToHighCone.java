// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class GoToHighCone extends CommandBase {

    Wrist wrist;
    Elevator elevator;
    /** Creates a new DriveForward. */
    public GoToHighCone(Wrist wrist, Elevator elevator) {
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(wrist, elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {



      new SequentialCommandGroup(

        new InstantCommand(() -> wrist.targetWristAngle = Constants.WRIST_CONE_HIGH_POSITION).withTimeout(0.5),
        new InstantCommand(() -> elevator.targetElevatorPosition = Constants.ELEVATOR_CONE_HIGH_LEVEL).withTimeout(0.5)
          
      );
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}   
