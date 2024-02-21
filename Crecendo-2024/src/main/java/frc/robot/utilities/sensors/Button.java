package frc.robot.utilities.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class Button {

    private DigitalInput input;
    public boolean hasBeenPressed = false;
    public Button(int port)
    {
        input = new DigitalInput(port);
    }

    public boolean isPressed()
    {
        if(hasBeenPressed)
        {
        hasBeenPressed = false;
        }
        else
        {
        hasBeenPressed = true;
        }
        return !input.get();
    } 

    public boolean wasPressed()
    {
        return hasBeenPressed;
    } 

    public void reset()
    {
        hasBeenPressed = false;
    }
}
