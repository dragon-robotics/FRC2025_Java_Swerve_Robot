package frc.robot.util;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class OperatorDashboard {
    ShuffleboardTab dashboard;

    // Which camera is active? //
    private SimpleWidget leftCamActive;
    private SimpleWidget rightCamActive;

    // Where are we scoring? //
    private SimpleWidget reefStationL1;
    private SimpleWidget reefStationLeftL2;
    private SimpleWidget reefStationLeftL3;
    private SimpleWidget reefStationLeftL4;
    private SimpleWidget reefStationRightL2;
    private SimpleWidget reefStationRightL3;
    private SimpleWidget reefStationRightL4;

    // Where are we intaking? //
    private SimpleWidget coralStationLeft;
    private SimpleWidget coralStationRight;
    
    // Where is the elevator? //
    private SimpleWidget elevatorHeight;

    // What game pieces are in the robot? //
    private SimpleWidget hasCoral;
    private SimpleWidget hasAlgae;

    // Are we ready to score? //
    private SimpleWidget alignedToScore;

    public OperatorDashboard() {
        dashboard = Shuffleboard.getTab("Dashboard");

        alignedToScore = dashboard
                .add("Aligned to Score", false)
                .withPosition(0, 0)
                .withSize(14, 2)
                .withWidget(BuiltInWidgets.kBooleanBox);
        
        leftCamActive = dashboard
                .add("Left Camera Active", false)
                .withPosition(5, 2)
                .withSize(1, 1)
                .withWidget(BuiltInWidgets.kBooleanBox);
        
        rightCamActive = dashboard
                .add("Right Camera Active", false)
                .withPosition(8, 2)
                .withSize(1, 1)
                .withWidget(BuiltInWidgets.kBooleanBox);

        reefStationL1 = dashboard
                .add("Reef Station L1", false)
                .withPosition(5, 6)
                .withSize(4, 1)
                .withWidget(BuiltInWidgets.kBooleanBox);

        reefStationLeftL2 = dashboard
                .add("Reef Station Left L2", false)
                .withPosition(5, 5)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kBooleanBox);
        
        reefStationLeftL3 = dashboard
                .add("Reef Station Left L3", false)
                .withPosition(5, 4)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kBooleanBox);

        reefStationLeftL4 = dashboard
                .add("Reef Station Left L4", false)
                .withPosition(5, 3)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kBooleanBox);

        reefStationRightL2 = dashboard
                .add("Reef Station Right L2", false)
                .withPosition(7, 5)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kBooleanBox);

        reefStationRightL3 = dashboard
                .add("Reef Station Right L3", false)
                .withPosition(7, 4)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kBooleanBox);
        
        reefStationRightL4 = dashboard
                .add("Reef Station Right L4", false)
                .withPosition(7, 3)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kBooleanBox);

        coralStationLeft = dashboard
                .add("Coral Station Left", false)
                .withPosition(5, 7)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kBooleanBox);
        
        coralStationRight = dashboard
                .add("Coral Station Right", false)
                .withPosition(7, 7)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kBooleanBox);

    }
}
