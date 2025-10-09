package frc.robot.subsystems.controller;

import lombok.Getter;
import lombok.Setter;

public interface ControllerIO {
  default void setControllerRumble(double rumbleValue) {
    throw new UnsupportedOperationException("setControllerRumble is not implemented");
  }

  default void setControllerRumble(double rumbleValue, boolean left) {
    throw new UnsupportedOperationException("setControllerRumble is not implemented");
  }

  class ControllerIOInputs {

    // Is the Controller connected? //
    @Getter @Setter private boolean controllerConnected;

    // Controller data //
    @Getter @Setter private int curRumbleValue;
  }

  default void updateInputs(ControllerIOInputs inputs) {}
}
