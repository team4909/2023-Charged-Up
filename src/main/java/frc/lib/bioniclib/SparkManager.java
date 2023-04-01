package frc.lib.bioniclib;

import java.util.ArrayList;
import java.util.function.Consumer;

import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SparkManager {

  private ArrayList<REVLibError> statusList = new ArrayList<>();
  private String m_systemName;
  public Consumer<REVLibError> statusTracker = statusList::add;

  public SparkManager(String systemName) {
    m_systemName = systemName;
  }

  public void reportErrors() {
    ArrayList<REVLibError> errors = new ArrayList<>();
    for (REVLibError e : statusList) {
      if (!e.equals(REVLibError.kOk))
        errors.add(e);

      if (errors.size() != 0)
        SmartDashboard.putString("Config Errors/" + m_systemName, errors.toString());
      else
        SmartDashboard.putString("Config Errors/" + m_systemName, "No errors detected");
    }
  }
}
