package frc.robot.controlboard;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.IOConstants;

public class ControlBoard {

      private final Joystick _joystickOne = new Joystick(IOConstants.JOYSTICK_ONE);
      private final Joystick _joystickTwo = new Joystick(IOConstants.JOYSTICK_TWO);
      private final XboxController _xboxController_1 = new XboxController(IOConstants.XBOX_CONTROLLER_1);
      private final XboxController _xboxController_2 = new XboxController(IOConstants.XBOX_CONTROLLER_2);
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
        return _xboxController_2.getRightX();
        //return _joystickTwo.getX();
    }

    public double getForward() {
        if (Math.abs(_xboxController_2.getLeftY()) > 0.1) {
            return _xboxController_2.getLeftY();
        }
        return 0;
    }

    public XboxController getXboxController1() {
        return _xboxController_1;
    }

    public XboxController getXboxController2() {
        return _xboxController_2;
    }
}
