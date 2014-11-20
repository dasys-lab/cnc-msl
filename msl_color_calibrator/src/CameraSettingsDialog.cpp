#include "CameraSettingsDialog.h"
#include "ui_camerasettingsdialog.h"

#include "mainwindow.h"

#include "ROSCommunicator.h"

CameraSettingsDialog* CameraSettingsDialog::instance;

CameraSettingsDialog::CameraSettingsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CameraSettingsDialog)
{
    ui->setupUi(this);

    connect(ui->brightnessSlider, SIGNAL(valueChanged(int)), ui->brightnessSpinBox, SLOT(setValue(int)));
    connect(ui->brightnessSpinBox, SIGNAL(valueChanged(int)), ui->brightnessSlider, SLOT(setValue(int)));

    connect(ui->exposureSlider, SIGNAL(valueChanged(int)), ui->exposureSpinBox, SLOT(setValue(int)));
    connect(ui->exposureSpinBox, SIGNAL(valueChanged(int)), ui->exposureSlider, SLOT(setValue(int)));

    connect(ui->whiteBalanceCheckBox, SIGNAL(toggled(bool)), ui->whiteBalance1Slider, SLOT(setEnabled(bool)));
    connect(ui->whiteBalanceCheckBox, SIGNAL(toggled(bool)), ui->whiteBalance1SpinBox, SLOT(setEnabled(bool)));
    connect(ui->whiteBalanceCheckBox, SIGNAL(toggled(bool)), ui->whiteBalance2Slider, SLOT(setEnabled(bool)));
    connect(ui->whiteBalanceCheckBox, SIGNAL(toggled(bool)), ui->whiteBalance2SpinBox, SLOT(setEnabled(bool)));
    connect(ui->whiteBalance1Slider, SIGNAL(valueChanged(int)), ui->whiteBalance1SpinBox, SLOT(setValue(int)));
    connect(ui->whiteBalance1SpinBox, SIGNAL(valueChanged(int)), ui->whiteBalance1Slider, SLOT(setValue(int)));
    connect(ui->whiteBalance2Slider, SIGNAL(valueChanged(int)), ui->whiteBalance2SpinBox, SLOT(setValue(int)));
    connect(ui->whiteBalance2SpinBox, SIGNAL(valueChanged(int)), ui->whiteBalance2Slider, SLOT(setValue(int)));

    connect(ui->hueSlider, SIGNAL(valueChanged(int)), ui->hueSpinBox, SLOT(setValue(int)));
    connect(ui->hueSpinBox, SIGNAL(valueChanged(int)), ui->hueSlider, SLOT(setValue(int)));

    connect(ui->saturationSlider, SIGNAL(valueChanged(int)), ui->saturationSpinBox, SLOT(setValue(int)));
    connect(ui->saturationSpinBox, SIGNAL(valueChanged(int)), ui->saturationSlider, SLOT(setValue(int)));

    connect(ui->gammaCheckBox, SIGNAL(toggled(bool)), ui->gammaSlider, SLOT(setEnabled(bool)));
    connect(ui->gammaCheckBox, SIGNAL(toggled(bool)), ui->gammaSpinBox, SLOT(setEnabled(bool)));
    connect(ui->gammaSlider, SIGNAL(valueChanged(int)), ui->gammaSpinBox, SLOT(setValue(int)));
    connect(ui->gammaSpinBox, SIGNAL(valueChanged(int)), ui->gammaSlider, SLOT(setValue(int)));

    connect(ui->shutterCheckBox, SIGNAL(toggled(bool)), ui->shutterSlider, SLOT(setEnabled(bool)));
    connect(ui->shutterCheckBox, SIGNAL(toggled(bool)), ui->shutterSpinBox, SLOT(setEnabled(bool)));
    connect(ui->shutterSlider, SIGNAL(valueChanged(int)), ui->shutterSpinBox, SLOT(setValue(int)));
    connect(ui->shutterSpinBox, SIGNAL(valueChanged(int)), ui->shutterSlider, SLOT(setValue(int)));

    connect(ui->gainCheckBox, SIGNAL(toggled(bool)), ui->gainSlider, SLOT(setEnabled(bool)));
    connect(ui->gainCheckBox, SIGNAL(toggled(bool)), ui->gainSpinBox, SLOT(setEnabled(bool)));
    connect(ui->gainSlider, SIGNAL(valueChanged(int)), ui->gainSpinBox, SLOT(setValue(int)));
    connect(ui->gainSpinBox, SIGNAL(valueChanged(int)), ui->gainSlider, SLOT(setValue(int)));

    connect(ui->sendSettingsButton, SIGNAL(released()), this, SLOT(sendSettingsSlot()));

    connect(MainWindow::getInstance(), SIGNAL(selectedRobot(Robot*)), this, SLOT(selectRobotSlot(Robot*)));
}

CameraSettingsDialog::~CameraSettingsDialog()
{
    delete ui;
}

CameraSettingsDialog *CameraSettingsDialog::getInstance(QWidget *parent) {
    if (instance == NULL) {
        instance = new CameraSettingsDialog(parent);
    }
    return instance;
}

void CameraSettingsDialog::sendSettingsSlot() {
    Robot* robot = MainWindow::getInstance()->getSelectedRobot();
    if (robot) {
        if (!ROSCommunicator::isROScoreRunning()) {
            cout << "roscore is not running" << endl;
        } else {
            if (!ROSCommunicator::isInitialized()) {
                ROSCommunicator::initialize();
                sleep(1); // wait for ros
            }

            if (ros::ok()) {
                cout << "send camera values to: " << robot->getID() << endl;
                CameraCalibration::Settings* settings = new CameraCalibration::Settings();
                //                 settings->useBrightness = true;
                settings->brightness = ui->brightnessSpinBox->value();
                settings->exposure = ui->exposureSpinBox->value();
                settings->autoWhiteBalance = ui->whiteBalanceCheckBox->isChecked() == false;
                settings->whiteBalance1 = ui->whiteBalance1SpinBox->value();
                settings->whiteBalance2 = ui->whiteBalance2SpinBox->value();
                settings->hue = ui->hueSpinBox->value();
                settings->saturation = ui->saturationSpinBox->value();
                settings->enabledGamma = ui->gammaCheckBox->isChecked();
                settings->gamma = ui->gammaSpinBox->value();
                settings->autoShutter = ui->shutterCheckBox->isChecked() == false;
                settings->shutter = ui->shutterSpinBox->value();
                settings->autoGain = ui->gainCheckBox->isChecked() == false;
                settings->gain = ui->gainSpinBox->value();

                ROSCommunicator::sendSettings(robot->getID(), settings);

                delete settings;
            }
        }
    }
}

void CameraSettingsDialog::selectRobotSlot(Robot *robot) {
    bool enableComponents = false;
    if (robot) {
        enableComponents = true;

        ui->brightnessSlider->setMinimum(robot->brightnessMin);
        ui->brightnessSlider->setMaximum(robot->brightnessMax);
        ui->brightnessSpinBox->setMinimum(robot->brightnessMin);
        ui->brightnessSpinBox->setMaximum(robot->brightnessMax);
        ui->brightnessSlider->setValue(robot->getBrightness());

        ui->exposureSlider->setMinimum(robot->exposureMin);
        ui->exposureSlider->setMaximum(robot->exposureMax);
        ui->exposureSpinBox->setMinimum(robot->exposureMin);
        ui->exposureSpinBox->setMaximum(robot->exposureMax);
        ui->exposureSlider->setValue(robot->getExposure());

        ui->whiteBalance1Slider->setMinimum(robot->whiteBalance1Min);
        ui->whiteBalance1Slider->setMaximum(robot->whiteBalance1Max);
        ui->whiteBalance1SpinBox->setMinimum(robot->whiteBalance1Min);
        ui->whiteBalance1SpinBox->setMaximum(robot->whiteBalance1Max);
        ui->whiteBalance2Slider->setMinimum(robot->whiteBalance2Min);
        ui->whiteBalance2Slider->setMaximum(robot->whiteBalance2Max);
        ui->whiteBalance2SpinBox->setMinimum(robot->whiteBalance2Min);
        ui->whiteBalance2SpinBox->setMaximum(robot->whiteBalance2Max);
        ui->whiteBalanceCheckBox->setChecked(robot->isAutoWhiteBalanceUsed() == false);
        ui->whiteBalance1Slider->setValue(robot->getWhiteBalance1());
        ui->whiteBalance2Slider->setValue(robot->getWhiteBalance2());

        ui->hueSlider->setMinimum(robot->hueMin);
        ui->hueSlider->setMaximum(robot->hueMax);
        ui->hueSpinBox->setMinimum(robot->hueMin);
        ui->hueSpinBox->setMaximum(robot->hueMax);
        ui->hueSlider->setValue(robot->getHue());

        ui->saturationSlider->setMinimum(robot->saturationMin);
        ui->saturationSlider->setMaximum(robot->saturationMax);
        ui->saturationSpinBox->setMinimum(robot->saturationMin);
        ui->saturationSpinBox->setMaximum(robot->saturationMax);
        ui->saturationSlider->setValue(robot->getSaturation());

        ui->gammaSlider->setMinimum(robot->gammaMin);
        ui->gammaSlider->setMaximum(robot->gammaMax);
        ui->gammaSpinBox->setMinimum(robot->gammaMin);
        ui->gammaSpinBox->setMaximum(robot->gammaMax);
        ui->gammaCheckBox->setChecked(robot->isGamma());
        ui->gammaSlider->setValue(robot->getGamma());

        ui->shutterSlider->setMinimum(robot->shutterMin);
        ui->shutterSlider->setMaximum(robot->shutterMax);
        ui->shutterSpinBox->setMinimum(robot->shutterMin);
        ui->shutterSpinBox->setMaximum(robot->shutterMax);
        ui->shutterCheckBox->setChecked(robot->isAutoShutterUsed() == false);
        ui->shutterSlider->setValue(robot->getShutter());

        ui->gainSlider->setMinimum(robot->gainMin);
        ui->gainSlider->setMaximum(robot->gainMax);
        ui->gainSpinBox->setMinimum(robot->gainMin);
        ui->gainSpinBox->setMaximum(robot->gainMax);
        ui->gainCheckBox->setChecked(robot->isAutoGainUsed() == false);
        ui->gainSlider->setValue(robot->getGain());

        if (robot->getLastUpdate() > 0) {
            time_t updateTime = robot->getLastUpdate();
            struct tm * updateTimeInfo;
            char buffer[80];

            updateTimeInfo = localtime(&updateTime);

            strftime(buffer, 80, "%T", updateTimeInfo);
            ui->settingsUpdatedLabel->setText(QString(buffer));
        } else {
            ui->settingsUpdatedLabel->setText(QString("Never"));
        }
    } else {
        ui->brightnessSlider->setValue(ui->brightnessSlider->minimum());
        ui->exposureSlider->setValue(ui->exposureSlider->minimum());
        ui->whiteBalance1Slider->setValue(ui->whiteBalance1Slider->minimum());
        ui->whiteBalance2Slider->setValue(ui->whiteBalance2Slider->minimum());
        ui->hueSlider->setValue(ui->hueSlider->minimum());
        ui->saturationSlider->setValue(ui->saturationSlider->minimum());
        ui->gammaSlider->setValue(ui->gammaSlider->minimum());
        ui->shutterSlider->setValue(ui->shutterSlider->minimum());
        ui->gainSlider->setValue(ui->gainSlider->minimum());
        ui->settingsUpdatedLabel->setText(QString(""));
    }

    ui->brightnessSlider->setEnabled(enableComponents);
    ui->brightnessSpinBox->setEnabled(enableComponents);

    ui->exposureSlider->setEnabled(enableComponents);
    ui->exposureSpinBox->setEnabled(enableComponents);

    ui->whiteBalance1Slider->setEnabled(enableComponents);
    ui->whiteBalance1SpinBox->setEnabled(enableComponents);
    ui->whiteBalance2Slider->setEnabled(enableComponents);
    ui->whiteBalance2SpinBox->setEnabled(enableComponents);
//        ui->whiteBalanceCheckBox->setEnabled(enableComponents);

    ui->hueSlider->setEnabled(enableComponents);
    ui->hueSpinBox->setEnabled(enableComponents);

    ui->saturationSlider->setEnabled(enableComponents);
    ui->saturationSpinBox->setEnabled(enableComponents);

    ui->gammaSlider->setEnabled(enableComponents);
    ui->gammaSpinBox->setEnabled(enableComponents);
    ui->gammaCheckBox->setEnabled(enableComponents);

    ui->shutterSlider->setEnabled(enableComponents);
    ui->shutterSpinBox->setEnabled(enableComponents);
    ui->shutterCheckBox->setEnabled(enableComponents);

    ui->gainSlider->setEnabled(enableComponents);
    ui->gainSpinBox->setEnabled(enableComponents);
    ui->gainCheckBox->setEnabled(enableComponents);

    ui->sendSettingsButton->setEnabled(enableComponents);
}

void CameraSettingsDialog::show() {
    QWidget::show();

    selectRobotSlot(MainWindow::getInstance()->getSelectedRobot());
}
