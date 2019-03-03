package frc.robot.utils;

import edu.wpi.first.wpilibj.Joystick;

public class ButtonDebouncer {
    private Joystick joystick;
    private int buttonNumber;

    private boolean previousState = false;

    public ButtonDebouncer(Joystick joystick, int buttonNumber) {
        this.joystick = joystick;
        this.buttonNumber = buttonNumber;
    }

    public boolean get() {
        boolean currentState = joystick.getRawButton(buttonNumber);
        if (!previousState && currentState) {
            previousState = currentState;
            return true;
        } else {
            previousState = currentState;
            return false;
        }
    }
}