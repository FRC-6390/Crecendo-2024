package frc.robot.utilities.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utilities.controller.Debouncer;

public class Button {

    private DigitalInput input;
    public boolean hasBeenPressed = false;
    public Debouncer debouncer;
    public Button(DigitalInput input)
    {
        this.input = input;
       // super(input::get);
        // input = new DigitalInput(port);
        // debouncer = new Debouncer(this, 0.5f);
    }

    // private boolean get(){
    //     return !input.get();
    // }

    public boolean isPressed()
    {
        return !input.get();
    } 
}
