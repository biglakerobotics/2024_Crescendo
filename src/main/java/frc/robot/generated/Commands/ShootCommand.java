package frc.robot.generated.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Manipulators.Shooter;

public class ShootCommand extends Command{

    private final Shooter mShooter;

    public ShootCommand(Shooter shooter){
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
