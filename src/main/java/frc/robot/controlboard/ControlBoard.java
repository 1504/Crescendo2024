package frc.robot.controlboard;

//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.IOConstants;

public class ControlBoard {

      private final XboxController _xboxController_gadgets = new XboxController(IOConstants.XBOX_CONTROLLER_GADGETS);
      private final XboxController _xboxController_drive = new XboxController(IOConstants.XBOX_CONTROLLER_DRIVE);

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
        return _xboxController_drive.getRightX();
        //return _joystickTwo.getX();
    }

    public double getForward() {
        if (Math.abs(_xboxController_drive.getLeftY()) > 0.1) {
            return _xboxController_drive.getLeftY();
        }
        return 0;
    }

    public XboxController getDriveController() {
        return _xboxController_drive;
    }

    public XboxController getGadgetsController() {
        return _xboxController_gadgets;
    }
}
