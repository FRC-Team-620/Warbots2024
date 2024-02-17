package org.jmhsrobotics.frc2024.utils;
package org.jmhsrobotics.frc2024.utils;

import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class Rumble {
    // alright, make different types of rumble. Different patterns of rumbling?

    /** A weak rumble on both sides of the controller
     * 
     * @param controller - the xbox controller that we're rumbling
     */
    public static void weakRumble(XboxController controller)
    {
        controller.setRumble(RumbleType.kBothRumble, .1);
    }

    public static void strongRumble(XboxController controller)
    {
        controller.setRumble(RumbleType.kBothRumble, 1);
    }

    // public static void mediumRumble(XboxController controller)
    // {
    //     controller.setRumble(RumbleType.kBothRumble, .5);
    // }

    

    
}
