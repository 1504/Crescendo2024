package frc.robot.controlboard;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.IOConstants;

public class ControlBoard {

      private final Joystick _joystickOne = new Joystick(IOConstants.JOYSTICK_ONE);
      private final Joystick _joystickTwo = new Joystick(IOConstants.JOYSTICK_TWO);
      private final XboxController _xboxController = new XboxController(IOConstants.XBOX_CONTROLLER);
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
        return _xboxController.getRightX();
        //return _joystickTwo.getX();
    }

    public double getForward() {
        if (Math.abs(_xboxController.getLeftY()) > 0.1) {
            return _xboxController.getLeftY();
        }
        return 0;
        /*if(Math.abs(_joystickOne.getY()) > 0.1) {
            return -_joystickOne.getY();
        }
        return 0; */
    }

    public Joystick getJoystick() {
        return _joystickOne;
    }

    public XboxController getXboxController() {
        return _xboxController;
    }
}
