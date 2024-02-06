package frc.robot.controlboard;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.IOConstants;

public class ControlBoard {

      private final Joystick _joystickOne = new Joystick(IOConstants.JOYSTICK_ONE);
      private static ControlBoard _instance = null;


      public static ControlBoard getInstance() {

        if (_instance == null) {
            _instance = new ControlBoard();
        }

        return _instance;
    }


    private ControlBoard() {
    }

    public double getX() {
        return -_joystickOne.getX();
    }

    public double getY() {
        return -_joystickOne.getY();
    }

    public Joystick getJoystick() {
        return _joystickOne;
    }
}
