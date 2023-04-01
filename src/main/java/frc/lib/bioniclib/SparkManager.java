package frc.lib.bioniclib;

import java.util.ArrayList;
import java.util.function.Consumer;

import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SparkManager {

  private ArrayList<REVLibError> m_statusList = new ArrayList<>();
  private String m_systemName;
  private int m_attempt = 1;
  private Runnable m_config;
  public Consumer<REVLibError> statusTracker = m_statusList::add;

  public SparkManager(String systemName) {
    m_systemName = systemName;
  }

  private boolean checkErrors() {
    ArrayList<REVLibError> errors = new ArrayList<>();
    for (REVLibError e : m_statusList) {
      if (!e.equals(REVLibError.kOk))
        errors.add(e);
      if (errors.size() != 0) {
        SmartDashboard.putString("Config Errors/" + m_systemName + " Attempt " + m_attempt, errors.toString());
        return true;
      } else {
        SmartDashboard.putString("Config Errors/" + m_systemName + " Attempt " + m_attempt, "No errors detected");
      }
    }
    return false;
  }

  public void forceConfig() {
    m_config.run();
    if (checkErrors()) {
      m_attempt++;
      m_config.run();
    }
  }

  public void setConfigRunnable(Runnable config) {
    m_config = config;
  }
}
