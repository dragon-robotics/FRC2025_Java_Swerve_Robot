package frc.robot.subsystems.controller;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControllerIOXbox implements ControllerIO {
    // Create an Xbox controller instance
    private final CommandXboxController controller;

    public ControllerIOXbox(CommandXboxController controller) {
        // Initialize the Xbox controller
        this.controller = controller;
    }

    @Override
    public void setControllerRumble(double rumbleValue) {
        controller.setRumble(RumbleType.kBothRumble, rumbleValue);
    }


    @Override
    public void setControllerRumble(double rumbleValue, boolean left) {
        controller.setRumble(left ? RumbleType.kRightRumble : RumbleType.kLeftRumble, rumbleValue);
    }
}
