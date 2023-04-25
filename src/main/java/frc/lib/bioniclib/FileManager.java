package frc.lib.bioniclib;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.Comparator;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;

public class FileManager {

  private final String kSimLogPath = "./logs-simulation";
  private final int kMaxOldLogFiles = 10;

  private static FileManager m_instance;
  private boolean m_dataLogsInitialized = false;

  public void initDataLog() {
    if (m_dataLogsInitialized)
      return;
    if (Constants.SIM) {
      Path path = Path.of(kSimLogPath);
      try {
        Files.createDirectories(path);
        // Delete old logs to prevent wasting storage
        File[] logs = new File(path.toAbsolutePath().toString()).listFiles();
        if (logs.length > kMaxOldLogFiles) {
          Arrays.sort(logs, Comparator.comparingLong(File::lastModified).reversed());
          for (int i = logs.length - 1; i >= kMaxOldLogFiles; i--)
            Files.delete(logs[i].toPath());
        }
      } catch (IOException e) {
        e.printStackTrace();
      }
      DataLogManager.start(path.toString());
    } else {
      DataLogManager.start();
      System.out.println("DataLog output folder " + Filesystem.getOperatingDirectory().getAbsolutePath()
          + " has " + Filesystem.getOperatingDirectory().getFreeSpace() / 1024 + " kB of free space.");
    }

    DriverStation.startDataLog(DataLogManager.getLog());
    m_dataLogsInitialized = true;
  }

  public static FileManager getInstance() {
    return m_instance = (m_instance == null) ? new FileManager() : m_instance;
  }
}