package org.usfirst.frc.team4915.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4915.robot.Robot;

/**
 *
 */
public class DriveForwardCommand extends Command {
    private int m_withinAllowableClosedLoopErrorCount;
    private static final int FINISH_COUNT_THRESHOLD = 10;
    
	public DriveForwardCommand() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.driveTrainSubsystem);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	    m_withinAllowableClosedLoopErrorCount = 0;
	    // initialize encoders
	    try
        {
            Robot.driveTrainSubsystem.resetPosition();
        }
        catch (InterruptedException e)
        {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
	    // give it one rotation to run 
	    Robot.driveTrainSubsystem.driveInDistance(Math.PI*6);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
	    // for motor safety -- need to update 'set' with target vs. current position
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
	    if (Robot.driveTrainSubsystem.withinAllowableClosedLoopError())
	    {
	        m_withinAllowableClosedLoopErrorCount++;
	    }
	    else
	    {
	        m_withinAllowableClosedLoopErrorCount = 0;
	    }
	    // isFinished runs every 20ms, and we look for more than 10 occurrences
	    // within the allowable closed loop error range. It takes at least
	    // 200ms to stabilize and return true
	    if (m_withinAllowableClosedLoopErrorCount > FINISH_COUNT_THRESHOLD)
	    {
	        return true;
	    }
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
        Robot.driveTrainSubsystem.stop();	    
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	    end();
	}
}
