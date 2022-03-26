/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A subsystem that contains ONLY the compressor for the pneumatics.
 * @see ClimbSubsystem
 * @see DiskSubsystem
 */
public class PneumaticsSubsystem extends SubsystemBase {

  private Compressor compressor;

  /**
   * Creates a new Pneumatics.
   */
  public PneumaticsSubsystem() {
    compressor = new Compressor(PneumaticsModuleType.CTREPCM); //https://docs.wpilib.org/en/stable/docs/software/hardware-apis/pneumatics/pneumatics.html
    compressor.enableDigital();
  }
  
  /**
	 * Allows the compressor to pump air at low pressures (not all the time).
	 */
	public void enableCompressor() {
		compressor.enableDigital();
	}

	/**
	 * Prevents the compressor from pumping air, at any time.
	 */
	public void disableCompressor() {
		compressor.disable();
	}

  @Override
  public void periodic() {

  }

}
