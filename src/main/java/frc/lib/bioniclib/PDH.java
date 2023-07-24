package frc.lib.bioniclib;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PDH {

  private final int kChannelCount = 24;
  private PowerDistribution m_pdh;

  public PDH(int canID) {
    m_pdh = new PowerDistribution(canID, ModuleType.kRev);
  }

  public void periodic() {
    SmartDashboard.putNumber("PDH/Voltage", m_pdh.getVoltage());
    SmartDashboard.putNumber("PDH/Total Current", m_pdh.getTotalCurrent());
    for (int channel = 0; channel < kChannelCount; channel++) {
      SmartDashboard.putNumber(
          ("PDH/Ch" + String.valueOf(channel) + " Current"),
          m_pdh.getCurrent(channel));
    }
  }
}
