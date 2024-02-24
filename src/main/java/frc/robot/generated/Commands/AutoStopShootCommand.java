package frc.robot.generated.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Manipulators.Shooter;

public class AutoStopShootCommand extends Command{

    private final Shooter mShooter;

    public AutoStopShootCommand(Shooter shooter){
        mShooter = shooter;
    }
    
    @Override
    public void execute(){
        mShooter.StopShootingCommand();
    }
    @Override
    public void end(boolean interrupted){
        mShooter.StopShootingCommand();
    }

}
