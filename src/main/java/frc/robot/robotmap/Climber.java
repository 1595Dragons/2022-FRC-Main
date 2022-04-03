package frc.robot.robotmap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Climber {

	// Left climber
	private static final int climberLeftInID = 8; //1
	private static final int climberLeftOutID = 9; //2
	public static final DoubleSolenoid climberLeft = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, climberLeftInID, climberLeftOutID);

	// Right climber
	private static final int climberRightInID = 10; //3
	private static final int climberRightOutID = 11; //4
	public static final DoubleSolenoid climberRight = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, climberRightInID, climberRightOutID);

}
