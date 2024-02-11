package frc.robot.controlboard;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.IOConstants;

public class ControlBoard {

      private final Joystick _joystickOne = new Joystick(IOConstants.JOYSTICK_ONE);
      private final Joystick _joystickTwo = new Joystick(IOConstants.JOYSTICK_TWO);
      private static ControlBoard _instance = null;


      public static ControlBoard getInstance() {

        if (_instance == null) {
            _instance = new ControlBoard();
        }

        return _instance;
    }


    private ControlBoard() {
    }


    public double getRot() {
        return _joystickTwo.getX();
    }

    public double getForward() {
        if(Math.abs(_joystickOne.getY()) > 0.1) {
            return -_joystickOne.getY();
        }
        return 0;
    }

    public Joystick getJoystick() {
        return _joystickOne;
    }
}
