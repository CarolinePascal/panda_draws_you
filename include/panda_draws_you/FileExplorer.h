#include <QApplication>
#include <QFileDialog>
#include <QDebug>

#include <string.h>

class FileExplorer : public QWidget
{
  public:

    FileExplorer(std::string directoryPath) : QWidget(), m_filePath(""), m_directoryPath(directoryPath) {};

    void openFile();
    std::string getFilePath();

  private:
    std::string m_filePath, m_directoryPath; 
};