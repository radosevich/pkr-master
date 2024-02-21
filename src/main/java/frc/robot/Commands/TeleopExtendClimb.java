// delete this file before comp if not in use

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Climb;

public class TeleopExtendClimb extends Command{
    private final Climb m_climb;

    private boolean done;

    public TeleopExtendClimb(Climb climb) {
        m_climb = climb;

        addRequirements(m_climb);
    }
    @Override
    public void initialize() {
    // Called when the command is initially scheduled.
      done = false;
    }
    @Override
    public void execute() {
        // Start up launch motors
        m_climb.rightClimb(Constants.kClimbSpeed);
        m_climb.leftClimb(Constants.kClimbSpeed);
        Timer.delay(0.5);
        m_climb.leftClimb(Constants.kStopSpeed);
        m_climb.rightClimb(Constants.kStopSpeed);
        done = true;
        done = true;
    }
}
