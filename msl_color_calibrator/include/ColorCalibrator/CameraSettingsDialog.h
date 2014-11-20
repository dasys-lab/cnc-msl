#ifndef CAMERASETTINGSDIALOG_H
#define CAMERASETTINGSDIALOG_H

#include <QDialog>

#include "Robot.h"

namespace Ui {
class CameraSettingsDialog;
}

class CameraSettingsDialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit CameraSettingsDialog(QWidget *parent = 0);
    ~CameraSettingsDialog();

    static CameraSettingsDialog *getInstance(QWidget *parent = 0);

    void show();
    
private:
    Ui::CameraSettingsDialog *ui;

    static CameraSettingsDialog* instance;

private slots:
    void sendSettingsSlot();
    void selectRobotSlot(Robot *robot);
};

#endif // CAMERASETTINGSDIALOG_H
