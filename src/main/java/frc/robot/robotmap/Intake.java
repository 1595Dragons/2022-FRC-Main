package frc.robot.robotmap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Intake {

	// Intake motor
	private static final int intakeMotorID = 32;
	public static final CANSparkMax intakeMotor = new CANSparkMax(intakeMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

	public static final double intakeBack = 0.2d; //FIXME
	public static final double intakeForward = -0.4d; //FIXME
	public static final double autoIntakeTime = 1.85d; //FIXME

	private static final int intakeSolenoidIn = 12; // 5
	private static final int intakeSolenoidOut = 13; // 6
	public static DoubleSolenoid intakeSolenoid = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, intakeSolenoidIn, intakeSolenoidOut);

}
