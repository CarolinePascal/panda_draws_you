#include "panda_draws_you/FileExplorer.h"
#include <ros/ros.h>

void FileExplorer::openFile()
{
  QString QPath = QString::fromStdString(m_directoryPath);
  
  QString output =  QFileDialog::getOpenFileName(
        this,
        "Open Sketch",
        QPath,
        "All files (*.*) ;; PNG files (*.png)");

  m_filePath = output.toUtf8().constData();
}

std::string FileExplorer::getFilePath()
{
  return(m_filePath);
}