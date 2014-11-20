#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <list>
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <time.h>       /* time_t, struct tm, time, localtime, strftime */

#include <msl_sensor_msgs/CameraSettings.h>

#include "PolygonQGraphicsScene.h"
#include "CameraSettingsDialog.h"
#include "Command.h"
#include "ImageResource.h"
#include "NetworkCommunicator.h"
#include "Robot.h"

const int CC_AREA =             460;
const int CC_IMAGE_WIDTH =      640;
const int CC_IMAGE_HEIGHT =     480;
const int CC_ROBOT_MAX_ID =     25;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    ~MainWindow();

    Ui::MainWindow *ui;

    static MainWindow* getInstance();

    Robot* getSelectedRobot();

protected:
    explicit MainWindow(QWidget *parent = 0);

private:
    static MainWindow* m_instance;

    QString m_imagePwd;

    unsigned char * m_lookupTable;
    CommandList* m_commandList;

    std::list<ImageResource*> m_images;
    std::map<int, Robot*> m_robots;

    unsigned char * getCurrentLookupTable();
    QString getLookupTablePwd();

    void processFilter();
    void addImage(ImageResource* imgRes, bool show);
    void showSelectedImage();
    void initLookupTable(bool defaultValues);
    void writeLookupTableSrcImage(bool add, bool outer);
    void writeLookupTableLookup(bool add);
    void writeLookupTableInterpolate();
    void writeLookupTableYuvfull(bool add);
    void updateLookupTable();
    void updateSenderComboBox();
    bool saveImageToFile(ImageResource *imgRes, QString filename);
    void saveAllImagesToFile(QString suffix);
    bool saveLookupTableToFile(unsigned char * lookupTable, QString filename);
    bool loadLookupTableToFile(unsigned char * lookupTable, QString filename);

private Q_SLOTS:
    void updateCommands();
    void handleReceivedImage(ImageResource *img);
    void handleReceivedSettings(const msl_sensor_msgs::CameraSettings::ConstPtr& msg);

    void leftDrawnSrcImageSlot(PolygonQGraphicsScene* scn);
    void rightDrawnSrcImageSlot(PolygonQGraphicsScene* scn);
    void leftDrawnLookupSlot(PolygonQGraphicsScene* scn);
    void rightDrawnLookupSlot(PolygonQGraphicsScene* scn);
    void leftClickedLookupSlot(PolygonQGraphicsScene* scn, int x, int y);
    void rightClickedLookupSlot(PolygonQGraphicsScene* scn, int x, int y);
    void leftDrawnYuvfullSlot(PolygonQGraphicsScene* scn);
    void rightDrawnYuvfullSlot(PolygonQGraphicsScene* scn);

    void on_selectImageComboBox_currentIndexChanged(int index);
    void on_prevImageButton_released();
    void on_nextImageButtonButton_released();
    void on_selectImageCloseButton_released();
    void on_loadImageFromSelectedROSButton_released();
    void on_loadImageFromAllROSButton_released();

    void on_lowerThresholdSlider_valueChanged(int value);
    void on_upperThresholdSlider_valueChanged(int value);
    void on_thresholdSlider_sliderReleased();
    void on_thresholdSpinBox_editingFinished();
    void on_clearLookuptableButton_released();
    void on_defaultLookuptableButton_released();

    void on_ySlider_valueChanged(int value);
    void on_emThresholdSlider_valueChanged(int value);
    void on_emThresholdSpinBox_valueChanged(double value);
    void on_selectRobotComboBox_currentIndexChanged(int index);

    void on_loadImageFromFileAction_triggered();
    void on_saveImageToFileAction_triggered();
    void on_saveAllImagesToRAWFileAction_triggered();
    void on_saveAllImagesToPNGFileAction_triggered();
    void on_loadLookupTableFromFileAction_triggered();
    void on_saveLookupTableToFileAction_triggered();
    void on_closeAction_triggered();
    void on_undoAction_triggered();
    void on_redoAction_triggered();
    void on_rectAction_triggered();
    void on_polyAction_triggered();
    void on_pushButton_released();

    void on_searchBallCheckBox_toggled(bool checked);

    void on_drawBallCheckBox_toggled(bool checked);

    void on_drawROICheckBox_toggled(bool checked);

    void on_drawLinesCheckBox_toggled(bool checked);

    void on_processImageSelectionComboBox_currentIndexChanged(int index);

    void on_pushButton_2_released();

Q_SIGNALS:
    void selectedRobot(Robot *robot);
};

#endif // MAINWINDOW_H
