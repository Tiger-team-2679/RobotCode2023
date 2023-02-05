package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class OI {
    public static final CommandXboxController driverController = new CommandXboxController(Constants.OI.driverPort);
    public static final CommandXboxController opertatorController = new CommandXboxController(Constants.OI.opertatorPort);
}
