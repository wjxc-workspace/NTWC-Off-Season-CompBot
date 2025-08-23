package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ButtonBoxConstants.Button;
import frc.robot.Constants.ButtonBoxConstants.Port;

public class ButtonBox {
  private final Map<Port, CommandJoystick> joystickMap;

  public ButtonBox(CommandJoystick js1, CommandJoystick js2) {
    joystickMap = Map.of(
        Port.k1, js1,
        Port.k2, js2);
  }

  public Trigger getButton(Button button) {
    CommandJoystick joystick = joystickMap.get(button.port);
    switch (button.type) {
      case kAxis:
        if (button.positiveAxis) {
          return joystick.axisGreaterThan(button.id, 0.05);
        } else {
          return joystick.axisLessThan(button.id, -0.05);
        }
      case kButton:
        return joystick.button(button.id);
    }
    return null;
  }
}
