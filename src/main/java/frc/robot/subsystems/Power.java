// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Power extends SubsystemBase {
  /** Creates a new Power. */
  private final PowerDistribution mPdp = new PowerDistribution(0,ModuleType.kCTRE);

  public Power() {}

  @Override
  public void periodic() {
    // PDP values
    double current00 = mPdp.getCurrent(0);
    double current01 = mPdp.getCurrent(1);
    double current02 = mPdp.getCurrent(2);
    double current03 = mPdp.getCurrent(3);
    double current04 = mPdp.getCurrent(4);
    double current05 = mPdp.getCurrent(5);
    double current06 = mPdp.getCurrent(6);
    double current07 = mPdp.getCurrent(7);
    double current08 = mPdp.getCurrent(8);
    double current09 = mPdp.getCurrent(9);
    double current10 = mPdp.getCurrent(10);
    double current11 = mPdp.getCurrent(11);
    double current12 = mPdp.getCurrent(12);
    double current13 = mPdp.getCurrent(13);
    double current14 = mPdp.getCurrent(14);
    double current15 = mPdp.getCurrent(15);

    double voltage = mPdp.getVoltage();
    double temperature = mPdp.getTemperature();
    double totalCurrent = mPdp.getTotalCurrent();
    double totalPower = mPdp.getTotalPower();

    SmartDashboard.putNumber("Voltage", voltage);
    SmartDashboard.putNumber("Total Current", totalCurrent);
    SmartDashboard.putNumber("Total Power", totalPower);
    SmartDashboard.putNumber("Temperature", temperature);

    SmartDashboard.putNumber("Current 0", current00 );
    SmartDashboard.putNumber("Current 1", current01 );
    SmartDashboard.putNumber("Current 2", current02 );
    SmartDashboard.putNumber("Current 3", current03 );
    SmartDashboard.putNumber("Current 4", current04 );
    SmartDashboard.putNumber("Current 5", current05 );
    SmartDashboard.putNumber("Current 6", current06 );
    SmartDashboard.putNumber("Current 7", current07 );
    SmartDashboard.putNumber("Current 8", current08 );
    SmartDashboard.putNumber("Current 9", current09 );
    SmartDashboard.putNumber("Current 10", current10 );
    SmartDashboard.putNumber("Current 11", current11 );
    SmartDashboard.putNumber("Current 12", current12 );
    SmartDashboard.putNumber("Current 13", current13 );
    SmartDashboard.putNumber("Current 14", current14 );
    SmartDashboard.putNumber("Current 15", current15 );
    

  }
}
