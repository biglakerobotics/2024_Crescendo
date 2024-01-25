package frc.robot.generated.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Manipulators.TopShooter;

public class TopShootCommand extends Command{

    private final TopShooter mShooter;

    public TopShootCommand(TopShooter shooter){
        mShooter = shooter;
    }
    
    @Override
    public void execute(){
        mShooter.ShootCommand();
    }
    @Override
    public void end(boolean interrupted){
        mShooter.StopShootingCommand();
    }

}
