package frc.robot.generated.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Manipulators.Intake;
import frc.robot.generated.Manipulators.SpeakerShooter;

public class AutoSpeakerShootCommand extends Command{

    private final SpeakerShooter mShooter;

    public AutoSpeakerShootCommand(SpeakerShooter shooter){
        mShooter = shooter;
    }
    
    @Override
    public void execute(){
        mShooter.SpeakerShootCommand();
    }
    @Override
    public void end(boolean interrupted){
        mShooter.StopShootingCommand();
    }

}
