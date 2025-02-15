package frc.robot.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class OperatorDashboard {
    ShuffleboardTab dashboard;
    // Where are we scoring? //
    private SimpleWidget coralScoreLocation;
    // Where are we intaking? //
    private SimpleWidget coralIntakeLocation;
    // Where is the elevator? //
    private SimpleWidget elevatorHeight;

    // What game pieces are in the robot? //
    private SimpleWidget hasCoral;
    private SimpleWidget hasAlgae;

    // Are we ready to score? //
    private SimpleWidget readyToAlignToReef; // Does the robot cameras see the reef station tags?

    // Simple Diagnostic Information //
    // Match Time //
    // FMS Info //
    // Auto Chooser //
    // Camera Feed //

    public OperatorDashboard() {
        dashboard = Shuffleboard.getTab("Operator Dashboard");
    }
}
