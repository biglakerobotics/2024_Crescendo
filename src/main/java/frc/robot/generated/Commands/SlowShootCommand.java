package frc.robot.generated.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Manipulators.SlowShooter;

public class SlowShootCommand extends Command{

    private final SlowShooter mShooter;

    public SlowShootCommand(SlowShooter shooter){
        mShooter = shooter;
    }
    
    // @Override
    // public void execute(){
    //     mShooter.SlowShootCommand();
    // }
    @Override
    public void end(boolean interrupted){
        mShooter.StopShootingCommand();
    }

}
