package frc.robot.generated.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Manipulators.Intake;
import frc.robot.generated.Manipulators.SpeakerShooter;

public class AutoSpeakerShootOnlyCommand extends Command{

    private final SpeakerShooter mShooter;
    private final Intake mIntake;

    public AutoSpeakerShootOnlyCommand(SpeakerShooter shooter, Intake intake){
        mShooter = shooter;
        mIntake = intake;
    }
    
    @Override
    public void execute(){
        mShooter.SpeakerShootCommand();
        mIntake.IntakeWithoutIndexer();
    }
    @Override
    public void end(boolean interrupted){
        mShooter.StopShootingCommand();
    }

}
