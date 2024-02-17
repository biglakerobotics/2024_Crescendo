package frc.robot.generated.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Manipulators.AmpShooter;

public class AmpShootCommand extends Command{

    private final AmpShooter mShooter;

    public AmpShootCommand(AmpShooter shooter){
        mShooter = shooter;
    }

    public void execute(){
        mShooter.AmpShootCommand();
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
