#include "mainwindow.h"

#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsPixmapItem>
#include <QPixmap>
#include <QRgb>

#include <QDesktopWidget>

#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <QScrollBar>
#include <QTreeWidgetItemIterator>

#include <QActionGroup>

#include <iostream>

#include "ui_mainwindow.h"

#include "filters/FilterYUVToRGB.h"
#include "filters/FilterYUVExtractSubImages.h"
#include "filters/FilterLinePoints.h"
#include "filters/FilterLinePointsROI.h"
#include "filters/FilterSobelDir.h"
#include "filters/FilterTemplateMatching.h"

#include "helpers/BallHelper.h"
#include "helpers/SpicaHelper.h"

#include "SystemConfig.h"
#include "Configuration.h"

#include "ROSCommunicator.h"

using namespace supplementary;
using namespace std;

MainWindow* MainWindow::m_instance;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    int primaryScreen = QApplication::desktop()->primaryScreen();
    move(QApplication::desktop()->screen(primaryScreen)->rect().center() - rect().center());

    m_imagePwd = QDir::homePath();

    m_commandList = new CommandList();
    connect(m_commandList, SIGNAL(listChanged()), this, SLOT(updateCommands()));

    // INIT LOOKUPTABLE WITH DEFAULTVALUES
    m_lookupTable = (unsigned char *) malloc(256*256 + 2);
    AbstractCommand* command = new CommandClearLookuptable(m_lookupTable, true);
    command->doIt();
    delete command;

    QGraphicsView *graphicsView;
    QGraphicsScene *scn;
    PolygonQGraphicsScene *polygonScn;

    // INIT SOURCE SCENE
    graphicsView = ui->graphicsView;
    polygonScn = new PolygonQGraphicsScene(graphicsView);
    graphicsView->setScene(polygonScn);

    connect(polygonScn, SIGNAL(leftDrawn(PolygonQGraphicsScene*)), this, SLOT(leftDrawnSrcImageSlot(PolygonQGraphicsScene*)));
    connect(polygonScn, SIGNAL(rightDrawn(PolygonQGraphicsScene*)), this, SLOT(rightDrawnSrcImageSlot(PolygonQGraphicsScene*)));

    // INIT LOOKUP SCENE
    graphicsView = ui->graphicsView_lookup;
    polygonScn = new PolygonQGraphicsScene(graphicsView);
    graphicsView->setScene(polygonScn);

    connect(polygonScn, SIGNAL(leftDrawn(PolygonQGraphicsScene*)), this, SLOT(leftDrawnLookupSlot(PolygonQGraphicsScene*)));
    connect(polygonScn, SIGNAL(rightDrawn(PolygonQGraphicsScene*)), this, SLOT(rightDrawnLookupSlot(PolygonQGraphicsScene*)));
    connect(polygonScn, SIGNAL(leftClicked(PolygonQGraphicsScene*,int,int)), this, SLOT(leftClickedLookupSlot(PolygonQGraphicsScene*,int,int)));
    connect(polygonScn, SIGNAL(rightClicked(PolygonQGraphicsScene*,int,int)), this, SLOT(rightClickedLookupSlot(PolygonQGraphicsScene*,int,int)));

    // INIT YUVFULL SCENE
    graphicsView = ui->graphicsView_yuvfull;
    polygonScn = new PolygonQGraphicsScene(graphicsView);
    graphicsView->setScene(polygonScn);

    connect(polygonScn, SIGNAL(leftDrawn(PolygonQGraphicsScene*)), this, SLOT(leftDrawnYuvfullSlot(PolygonQGraphicsScene*)));
    connect(polygonScn, SIGNAL(rightDrawn(PolygonQGraphicsScene*)), this, SLOT(rightDrawnYuvfullSlot(PolygonQGraphicsScene*)));

    // INIT GRAY SCENE
    graphicsView = ui->graphicsView_gray;
    scn = new QGraphicsScene(graphicsView);
    graphicsView->setScene(scn);

    // INIT UV SCENE
    graphicsView = ui->graphicsView_uv;
    scn = new QGraphicsScene(graphicsView);
    graphicsView->setScene(scn);

    // INIT ROI SCENE
    graphicsView = ui->graphicsView_roi;
    scn = new QGraphicsScene(graphicsView);
    graphicsView->setScene(scn);

    // INIT ROIROLAND SCENE
    graphicsView = ui->graphicsView_roiroland;
    scn = new QGraphicsScene(graphicsView);
    graphicsView->setScene(scn);

    // INIT BRIGHTENEDGRAY SCENE
    graphicsView = ui->graphicsView_brightenedgray;
    scn = new QGraphicsScene(graphicsView);
    graphicsView->setScene(scn);

    // INIT LIST ITEMS
    SystemConfig* sc = SystemConfig::getInstance();
    Configuration *globals = (*sc)["Globals"];
    shared_ptr<vector<string> > teamBPtr = globals->getSections("Globals", "Team", NULL);
    std::vector<std::string> * team = teamBPtr.get();

    ui->selectRobotComboBox->addItem(QString("unknown"), QVariant(-1));
    for (unsigned int i = 0; i < team->size(); i++) {
        int id = globals->get<int>("Globals", "Team", (*team)[i].c_str(), "ID", NULL);
        if (id <= CC_ROBOT_MAX_ID) {
            // INIT ROBOTS
            Robot *robot = new Robot(id, (*team)[i].c_str());
            m_robots.insert(std::pair<int, Robot*>(id, robot));

            // INIT TREEWIDGET
            QTreeWidgetItem *item = new QTreeWidgetItem();
            item->setText(0, QString("%1").arg(id));
            item->setText(1, QString("%1").arg((*team)[i].c_str()));
            item->setData(0, Qt::UserRole, QVariant(id));
            ui->rosTreeWidget->addTopLevelItem(item);

            // ADD ROBOT TO COMBOBOX
            ui->selectRobotComboBox->addItem(QString("%1 (%2)").arg(QString::fromStdString(robot->getHostname())).arg(id), QVariant(id));
        }
    }

    for (std::map<int, Robot*>::iterator it = m_robots.begin(); it != m_robots.end(); it++) {
        Robot *robot = it->second;

        // LOAD LOOKUPTABLE
        char path[256];
        strcpy(path, getenv("DOMAIN_FOLDER"));

        std::ostringstream oss;
        oss << path << "/etc/" << robot->getHostname() << "/LookupTable.txt";
        QString qfilename = oss.str().c_str();
        if (!loadLookupTableToFile(robot->m_lookupTable, qfilename)) {
            AbstractCommand* command = new CommandClearLookuptable(robot->m_lookupTable, true);
            command->doIt();
            delete command;
        }

        // LOAD DEFAULT CAMERA PARAMETERS
        SystemConfig* sc = SystemConfig::getInstance();
        SystemConfig::setHostname(robot->getHostname());
        Configuration *vision = (*sc)["Vision"];

        std::string camera_vendor = vision->tryGet<std::string>("Imaging Source", "Vision", "Camera1394Settings", "CameraVendor", NULL);
        int firstBlank = camera_vendor.find(" ");
        if (std::string::npos != firstBlank) {
            camera_vendor = camera_vendor.substr(0 , firstBlank);
        }

        //        camera_vendor = "Imaging"; // TODO: das ist nur tmp

        Configuration *cameraVendor = (*sc)["CameraVendor"];
        //        cameraVendor->tryGet<ConfigNode>("CameraVendor", camera_vendor.c_str(), "BrightnessMin", NULL);
        robot->brightnessMin = cameraVendor->get<int>("CameraVendor", camera_vendor.c_str(), "BrightnessMin", NULL);
        robot->brightnessMax = cameraVendor->get<int>("CameraVendor", camera_vendor.c_str(), "BrightnessMax", NULL);
        robot->exposureMin = cameraVendor->get<int>("CameraVendor", camera_vendor.c_str(), "ExposureMin", NULL);
        robot->exposureMax = cameraVendor->get<int>("CameraVendor", camera_vendor.c_str(), "ExposureMax", NULL);
        robot->whiteBalance1Min = cameraVendor->get<int>("CameraVendor", camera_vendor.c_str(), "WhiteBalance1Min", NULL);
        robot->whiteBalance1Max = cameraVendor->get<int>("CameraVendor", camera_vendor.c_str(), "WhiteBalance1Max", NULL);
        robot->whiteBalance2Min = cameraVendor->get<int>("CameraVendor", camera_vendor.c_str(), "WhiteBalance2Min", NULL);
        robot->whiteBalance2Max = cameraVendor->get<int>("CameraVendor", camera_vendor.c_str(), "WhiteBalance2Max", NULL);
        robot->hueMin = cameraVendor->get<int>("CameraVendor", camera_vendor.c_str(), "HueMin", NULL);
        robot->hueMax = cameraVendor->get<int>("CameraVendor", camera_vendor.c_str(), "HueMax", NULL);
        robot->saturationMin = cameraVendor->get<int>("CameraVendor", camera_vendor.c_str(), "SaturationMin", NULL);
        robot->saturationMax = cameraVendor->get<int>("CameraVendor", camera_vendor.c_str(), "SaturationMax", NULL);
        robot->gammaMin = cameraVendor->get<int>("CameraVendor", camera_vendor.c_str(), "GammaMin", NULL);
        robot->gammaMax = cameraVendor->get<int>("CameraVendor", camera_vendor.c_str(), "GammaMax", NULL);
        robot->shutterMin = cameraVendor->get<int>("CameraVendor", camera_vendor.c_str(), "ShutterMin", NULL);
        robot->shutterMax = cameraVendor->get<int>("CameraVendor", camera_vendor.c_str(), "ShutterMax", NULL);
        robot->gainMin = cameraVendor->get<int>("CameraVendor", camera_vendor.c_str(), "GainMin", NULL);
        robot->gainMax = cameraVendor->get<int>("CameraVendor", camera_vendor.c_str(), "GainMax", NULL);

        robot->setUseBrightness(vision->tryGet<bool>(false, "Vision", "Camera1394Settings", "UseBrightness", NULL), false);
        if (robot->usingBrightness()) {
            robot->setBrightness(vision->get<int>("Vision", "Camera1394Settings", "Brightness", NULL), false);
        } else {
            robot->setBrightness(0, false);
        }
        robot->setExposure(vision->get<int>("Vision", "Camera1394Settings", "Exposure", NULL), false);
        robot->setAutoWhiteBalance(false, false); // never used
        robot->setWhiteBalance1(vision->get<int>("Vision", "Camera1394Settings", "WB1", NULL), false);
        robot->setWhiteBalance2(vision->get<int>("Vision", "Camera1394Settings", "WB2", NULL), false);
        robot->setHue(vision->get<int>("Vision", "Camera1394Settings", "Hue", NULL), false);
        robot->setSaturation(vision->get<int>("Vision", "Camera1394Settings", "Saturation", NULL), false);
        robot->setEnabledGamma(false, false);
        robot->setGamma(vision->get<int>("Vision", "Camera1394Settings", "Gamma", NULL), false);
        if(vision->tryGet<bool>(false, "Vision", "Camera1394Settings", "SetManSettingsMode", NULL)) {
            //            cam->setManualSettingModes();
            robot->setAutoShutter(false, false);
        } else {
            robot->setAutoShutter(true, false);
        }
        robot->setShutter(vision->tryGet<int>(30, "Vision", "Camera1394Settings", "Shutter", NULL), false);
        robot->setAutoGain(vision->get<bool>("Vision", "Camera1394Settings", "AutoGain", NULL), false);
        if (robot->isAutoGainUsed() == false) {
            robot->setGain(0, false);
        } else {
            robot->setGain(vision->get<int>("Vision", "Camera1394Settings", "Gain", NULL), false);
        }
	SystemConfig::resetHostname();
    }

    connect(ui->ySlider, SIGNAL(valueChanged(int)), ui->ySpinBox, SLOT(setValue(int)));
    connect(ui->ySpinBox, SIGNAL(valueChanged(int)), ui->ySlider, SLOT(setValue(int)));

    connect(ui->upperThresholdSlider, SIGNAL(valueChanged(int)), ui->upperThresholdSpinBox, SLOT(setValue(int)));
    connect(ui->upperThresholdSpinBox, SIGNAL(valueChanged(int)), ui->upperThresholdSlider, SLOT(setValue(int)));
    connect(ui->lowerThresholdSlider, SIGNAL(valueChanged(int)), ui->lowerThresholdSpinBox, SLOT(setValue(int)));
    connect(ui->lowerThresholdSpinBox, SIGNAL(valueChanged(int)), ui->lowerThresholdSlider, SLOT(setValue(int)));
    connect(ui->upperThresholdSpinBox, SIGNAL(editingFinished()), this, SLOT(on_thresholdSpinBox_editingFinished()));
    connect(ui->lowerThresholdSpinBox, SIGNAL(editingFinished()), this, SLOT(on_thresholdSpinBox_editingFinished()));
    connect(ui->upperThresholdSlider, SIGNAL(sliderReleased()), this, SLOT(on_thresholdSlider_sliderReleased()));
    connect(ui->lowerThresholdSlider, SIGNAL(sliderReleased()), this, SLOT(on_thresholdSlider_sliderReleased()));

    on_selectRobotComboBox_currentIndexChanged(ui->selectRobotComboBox->currentIndex());

    QActionGroup *actionGroup = new QActionGroup(this);
    actionGroup->addAction(ui->rectAction);
    actionGroup->addAction(ui->polyAction);

    qRegisterMetaType<msl_sensor_msgs::CameraSettings::ConstPtr>("msl_sensor_msgs::CameraSettings::ConstPtr");
    connect(ROSCommunicator::getInstance(), SIGNAL(receivedSettings(msl_sensor_msgs::CameraSettings::ConstPtr)), this, SLOT(handleReceivedSettings(msl_sensor_msgs::CameraSettings::ConstPtr)));
    connect(NetworkCommunicator::getInstance(), SIGNAL(receivedImage(ImageResource*)), this, SLOT(handleReceivedImage(ImageResource*)));

    if (ROSCommunicator::isROScoreRunning()) {
        if (!ROSCommunicator::isInitialized()) {
            ROSCommunicator::initialize();
            sleep(1); // wait for ros
        }

        if (ros::ok()) {
            vector<int> receiverIDs;
            cout << "request camera values from: ";
            for (std::map<int, Robot*>::iterator it = m_robots.begin(); it != m_robots.end(); it++) {
                cout << it->first << " ";
                receiverIDs.push_back(it->first);
            }
            cout << endl;
            ROSCommunicator::requestSettings(receiverIDs);
        }
    }

    SpicaHelper::initialize();

    updateLookupTable();
    updateSenderComboBox();
    processFilter();
}

MainWindow::~MainWindow()
{
    delete ui;

    free(m_lookupTable);

    for (std::map<int, Robot*>::iterator it = m_robots.begin(); it != m_robots.end(); it++) {
        delete it->second;
    }

    for (std::list<ImageResource*>::iterator it = m_images.begin(); it != m_images.end(); it++) {
        delete *it;
    }

    delete m_commandList;
}

MainWindow* MainWindow::getInstance() {
    if(!m_instance) {
        m_instance = new MainWindow();
    }
    return m_instance;
}

unsigned char * MainWindow::getCurrentLookupTable() {
    Robot* robot = getSelectedRobot();
    if (robot) {
        return robot->m_lookupTable;
    }
    return m_lookupTable;
}

QString MainWindow::getLookupTablePwd() {
    Robot* robot = getSelectedRobot();
    if (robot) {
        char path[256];
        strcpy(path, getenv("DOMAIN_FOLDER"));

        std::ostringstream oss;
        oss << path << "/etc/" << robot->getHostname() << "/LookupTable.txt";
        return oss.str().c_str();
    }
    return QDir::homePath();
}

Robot* MainWindow::getSelectedRobot() {
    int index = ui->selectRobotComboBox->currentIndex();
    if (index > 0) {
        int robotID = ui->selectRobotComboBox->itemData(index).toInt();
        if (m_robots.find(robotID) != m_robots.end()) {
            Robot *robot = m_robots.at(robotID);
            return robot;
        }
    }
    return NULL;
}

void MainWindow::processFilter() {
    ui->graphicsView_gray->scene()->clear();
    ui->graphicsView_uv->scene()->clear();
    ui->graphicsView_roi->scene()->clear();
    ui->graphicsView_roiroland->scene()->clear();
    ui->graphicsView_brightenedgray->scene()->clear();

    int index = ui->selectImageComboBox->currentIndex();
    if (index > -1) {
        std::list<ImageResource*>::iterator it = m_images.begin();
        std::advance(it, index);
        ImageResource* imgRes = *it;

        SystemConfig* sc = SystemConfig::getInstance();
        if (imgRes->getSender() != -1) {
            Robot *robot = m_robots.at(imgRes->getSender());
            SystemConfig::setHostname(robot->getHostname());
        } else {
            SystemConfig::resetHostname();
        }
        Configuration *vision = (*sc)["Vision"];
        short mx = vision->get<short>("Vision", "CameraMX", NULL);
        short my = vision->get<short>("Vision", "CameraMY", NULL);

        int edgethresh = vision->get<int>("Vision", "BallEdgethres", NULL);
        int edgemaskthresh = vision->get<int>("Vision", "BallEdgemaskthres", NULL);
        int maskThresh = vision->get<int>("Vision", "BallTemplMaskThres", NULL);
        Configuration *kh = (*sc)["KickHelper"];
        int kickerCount = (int)kh->tryGet<int>(3, "KickConfiguration", "KickerCount", NULL);
        if(kickerCount>3) kickerCount=3;

        unsigned char * currImage = NULL;

        unsigned char * image_gray = NULL;
        unsigned char * image_uv = NULL;
        unsigned char * image_roi = NULL;
        unsigned char * image_roiRoland = NULL;
        unsigned char * imbrightenedGray = NULL;

        unsigned char * imageSDir = NULL;
        unsigned char * imBall = NULL;

        FilterYUVExtractSubImages filterYUVExtractSubImages(imgRes->getWidth(), imgRes->getHeight(), CC_AREA);

        unsigned char * lookupTable = getCurrentLookupTable();
        for(int u=0; u<256; u++) {
            for(int v=0; v<256; v++) {
                filterYUVExtractSubImages.setLookupTableValue(u*256 + v, lookupTable[u*256 + v]);
                /*                filterYUVExtractSubImages.setLookupTableROIValue(u*256 + v, 0);
                filterYUVExtractSubImages.setLookupTableROIRolandValue(u*256 + v, 0);
                for(int y=0; y<256; y++) {
                    filterYUVExtractSubImages.setLookupTableUVYValue(y*256*256 + u*256 + v, 0);
                }*/
            }
        }

        unsigned char * content = (unsigned char *) malloc(imgRes->getWidth() * imgRes->getHeight() * 2);
        for (int i = 0; i < imgRes->getWidth() * imgRes->getHeight() * 2; i++) {
            content[i] = imgRes->getContentAt(i);
        }
        filterYUVExtractSubImages.process(content, imgRes->getWidth(), imgRes->getHeight(), mx, my, image_gray, image_uv, image_roi, image_roiRoland, imbrightenedGray);
        free(content);

        QImage *img;
        int img_y, img_x;
        int value;



        /*if (imgRes->getSender() != -1) {
            ScanLineHelper scanHelper;
            ScanLineHelperBall scanHelperBall;
            DistanceLookupHelper distanceHelper(CC_AREA);
            BallHelper ballHelper(CC_AREA);

            FilterLinePoints filterLinePoints(CC_AREA);
            FilterLinePointsROI filterLinePointsROI(CC_AREA);
            FilterSobelDir filterSobelDir(CC_AREA, CC_AREA);
            FilterTemplateMatching filterTMatch(CC_AREA, CC_AREA);



            std::vector<LinePoint> linePoints;
            linePoints.clear();

            std::vector<LinePoint> linePointsROI;
            linePointsROI.clear();

            int *balls;
            int ballCount = 0, clusterCount = 0;
            BallClusterHelp ballClusterHelp;
            ballCluster *cluster = new ballCluster[10000];

            std::vector<ROIData> roiData;
            ROIData curBallROI;

            std::vector<BlobBounds> ballBlobs;
            ballBlobs.clear();



            if (ui->drawLinesCheckBox->isChecked()) {
                cout << "Stage 4: Detect LinepointsballBlobs" << endl;
                image_gray = filterLinePoints.process((unsigned char *) image_gray, CC_AREA, CC_AREA, linePoints, distanceHelper, scanHelper);
            }

            if (ui->searchBallCheckBox->isChecked()) {
                cout << "Stage 5: Detect ROIs" << endl;
                roiData = filterLinePointsROI.process((unsigned char *) image_uv, CC_AREA, CC_AREA, linePointsROI, distanceHelper, scanHelperBall);

                cout << "Stage 6: Insert Tracked ROI into ROI List" << endl;
                roiData.insert(roiData.end()-kickerCount, curBallROI);

                cout << "Stage 7: Compute Sobel on ROIs" << endl;
                imageSDir = filterSobelDir.process(image_uv, image_uv, roiData, CC_AREA, CC_AREA, edgethresh, edgemaskthresh);

                cout << "Stage 8: Apply Templatematching" << endl;
                imBall = filterTMatch.process(imageSDir, balls, ballCount, image_uv, roiData, maskThresh, CC_AREA, CC_AREA, 4, 19, 6, image_roi);

                cout << "Stage 9:" << endl;
                clusterCount = ballClusterHelp.clusterBalls(balls, ballCount, cluster, 200);

                cout << "Stage 11: Cluster available Balls" << endl;
                Particle maxParticle;
                maxParticle.posx = 0;
                maxParticle.posy = 0;
                Point p = ballHelper.getBallFromBlobs(cluster, clusterCount, roiData, ballBlobs, &maxParticle);
                if(roiData.size() > 0)
                    curBallROI = roiData[0];

                if(ui->drawBallCheckBox->isChecked()) {
                    ballClusterHelp.visualizeCluster(image_gray, CC_AREA, CC_AREA, cluster, clusterCount);
                    ballClusterHelp.visualizeCluster(image_roi, CC_AREA, CC_AREA, cluster, clusterCount);
                }

                if(ui->drawROICheckBox->isChecked()) {
                    filterLinePointsROI.visualizeROIs(image_gray, roiData, CC_AREA, CC_AREA);
                    filterLinePointsROI.visualizeROIs(image_roi, roiData, CC_AREA, CC_AREA);
                }
            }
        }*/

        SystemConfig::resetHostname();

        // SHOW GRAY IMAGE
        img = new QImage(CC_AREA, CC_AREA, QImage::Format_RGB32);

        currImage = image_gray;
        for(img_y = 0; img_y < CC_AREA; img_y++) {
            for(img_x = 0; img_x < CC_AREA; img_x++)
            {
                value = *currImage++;
                img->setPixel(img_x, img_y, qRgb(value, value, value));
            }
        }

        ui->graphicsView_gray->scene()->addPixmap(QPixmap::fromImage(img->scaledToWidth(ui->graphicsView_gray->width()-2)));
//        ui->graphicsView_gray->scene()->addPixmap(QPixmap::fromImage(*img));

        // SHOW UV IMAGE
        img = new QImage(CC_AREA, CC_AREA, QImage::Format_RGB32);

        currImage = image_uv;
        for(img_y = 0; img_y < CC_AREA; img_y++) {
            for(img_x = 0; img_x < CC_AREA; img_x++)
            {
                value = *currImage++;
                img->setPixel(img_x, img_y, qRgb(value, value, value));
            }
        }

        ui->graphicsView_uv->scene()->addPixmap(QPixmap::fromImage(img->scaledToWidth(ui->graphicsView_uv->width()-2)));
//        ui->graphicsView_uv->scene()->addPixmap(QPixmap::fromImage(*img));

        // SHOW ROI IMAGE
        img = new QImage(CC_AREA, CC_AREA, QImage::Format_RGB32);

        currImage = image_roi;
        for(img_y = 0; img_y < CC_AREA; img_y++) {
            for(img_x = 0; img_x < CC_AREA; img_x++)
            {
                value = *currImage++;
                img->setPixel(img_x, img_y, qRgb(value, value, value));
            }
        }

        ui->graphicsView_roi->scene()->addPixmap(QPixmap::fromImage(img->scaledToWidth(ui->graphicsView_roi->width()-2)));
//        ui->graphicsView_roi->scene()->addPixmap(QPixmap::fromImage(*img));

        // SHOW ROIROLAND IMAGE
        img = new QImage(CC_AREA, CC_AREA, QImage::Format_RGB32);

        currImage = image_roiRoland;
        for(img_y = 0; img_y < CC_AREA; img_y++) {
            for(img_x = 0; img_x < CC_AREA; img_x++)
            {
                value = *currImage++;
                img->setPixel(img_x, img_y, qRgb(value, value, value));
            }
        }

        ui->graphicsView_roiroland->scene()->addPixmap(QPixmap::fromImage(img->scaledToWidth(ui->graphicsView_roiroland->width()-2)));
//        ui->graphicsView_roiroland->scene()->addPixmap(QPixmap::fromImage(*img));

        // SHOW BRIGHTENEDGRAY IMAGE
        img = new QImage(CC_AREA, CC_AREA, QImage::Format_RGB32);

        currImage = imbrightenedGray;
        for(img_y = 0; img_y < CC_AREA; img_y++) {
            for(img_x = 0; img_x < CC_AREA; img_x++)
            {
                value = *currImage++;
                img->setPixel(img_x, img_y, qRgb(value, value, value));
            }
        }

        ui->graphicsView_brightenedgray->scene()->addPixmap(QPixmap::fromImage(img->scaledToWidth(ui->graphicsView_brightenedgray->width()-2)));
//        ui->graphicsView_brightenedgray->scene()->addPixmap(QPixmap::fromImage(*img));
    }
}

void MainWindow::addImage(ImageResource* imgRes, bool show) {
    m_images.push_back(imgRes);

    time_t updateTime = time(NULL);
    struct tm * updateTimeInfo;
    char buffer[80];

    updateTimeInfo = localtime(&updateTime);

    strftime(buffer, 80, "%T", updateTimeInfo);
    ui->selectImageComboBox->addItem(QString("%1 %2")
                                     .arg(QString(buffer))
                                     .arg(QString::fromStdString(imgRes->m_filename)));

    if (show) {
        ui->selectImageComboBox->setCurrentIndex(ui->selectImageComboBox->count()-1);
        ui->inputTabWidget->setCurrentIndex(0);
        showSelectedImage();
    }
}

void MainWindow::showSelectedImage() {
    ((PolygonQGraphicsScene*) ui->graphicsView->scene())->clear();

    int index = ui->selectImageComboBox->currentIndex();
    if (index > -1) {
        unsigned char * imageRGB = NULL;

        std::list<ImageResource*>::iterator it = m_images.begin();
        std::advance(it, index);
        ImageResource* imgRes = *it;

        QImage *img = new QImage(imgRes->getWidth(), imgRes->getHeight(), QImage::Format_RGB32);

        FilterYUVToRGB filterYUVToRGB(imgRes->getWidth(), imgRes->getHeight());
        unsigned char * content = (unsigned char *) malloc(imgRes->getWidth() * imgRes->getHeight() * 2);
        for (int i = 0; i < imgRes->getWidth() * imgRes->getHeight() * 2; i++) {
            content[i] = imgRes->getContentAt(i);
        }
        imageRGB = filterYUVToRGB.process(content, imgRes->getWidth() * imgRes->getHeight() * 2);
        free(content);

        int img_y, img_x;
        for(img_y = 0; img_y < imgRes->getHeight(); img_y++) {
            for(img_x = 0; img_x < imgRes->getWidth(); img_x++)
            {
                int r = *imageRGB++;
                int g = *imageRGB++;
                int b = *imageRGB++;

                QRgb value = qRgb(r, g, b);
                img->setPixel(img_x, img_y, value);
            }
        }

        QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage(*img));
        item->setZValue(0.0);
        ui->graphicsView->scene()->addItem(item);
        delete img;
    }

    updateSenderComboBox();
    processFilter();
}

void MainWindow::initLookupTable(bool defaultValues) {
    AbstractCommand* command = new CommandClearLookuptable(getCurrentLookupTable(), defaultValues);
    if (command->doIt()) {
        m_commandList->addCommand(command);
    }
}

void MainWindow::writeLookupTableSrcImage(bool add, bool outer) {
    int index = ui->selectImageComboBox->currentIndex();
    if (index > -1) {
        std::list<ImageResource*>::iterator it = m_images.begin();
        std::advance(it, index);
        ImageResource* imgRes = *it;

        unsigned char * content = (unsigned char *) malloc(imgRes->getWidth() * imgRes->getHeight() * 2);
        for (int i = 0; i < imgRes->getWidth() * imgRes->getHeight() * 2; i++) {
            content[i] = imgRes->getContentAt(i);
        }

        PolygonQGraphicsScene* scn = (PolygonQGraphicsScene*) ui->graphicsView->scene();
        QPolygon polygon = scn->getPolygon();

        AbstractCommand* command = new CommandImage2Lookuptable(content, imgRes->getWidth(), imgRes->getHeight(), getCurrentLookupTable(), polygon, add, outer);
        if (command->doIt()) {
            m_commandList->addCommand(command);
        } else {
            delete command;
        }

        free(content);
    }
}

void MainWindow::writeLookupTableLookup(bool add) {
    PolygonQGraphicsScene* scn = (PolygonQGraphicsScene*) ui->graphicsView_lookup->scene();
    QPolygon polygon = scn->getPolygon();

    AbstractCommand* command = new CommandYUV2Lookuptable(getCurrentLookupTable(), polygon, add);

    if (command->doIt()) {
        m_commandList->addCommand(command);
    } else {
        delete command;
    }
}

void MainWindow::writeLookupTableInterpolate() {
    PolygonQGraphicsScene* scn = (PolygonQGraphicsScene*) ui->graphicsView_lookup->scene();
    QPolygon polygon = scn->getPolygon();

    AbstractCommand* command = new CommandInterpolateYUV2Lookuptable(getCurrentLookupTable(), polygon, ui->emThresholdSpinBox->value(), ui->emClusterSpinBox->value());;

    if (command->doIt()) {
        m_commandList->addCommand(command);
    } else {
        delete command;
    }
}

void MainWindow::writeLookupTableYuvfull(bool add) {
    PolygonQGraphicsScene* scn = (PolygonQGraphicsScene*) ui->graphicsView_yuvfull->scene();
    QPolygon polygon = scn->getPolygon();

    AbstractCommand* command = new CommandYUV2Lookuptable(getCurrentLookupTable(), polygon, add);

    if (command->doIt()) {
        m_commandList->addCommand(command);
    } else {
        delete command;
    }
}

void MainWindow::updateLookupTable() {
    QImage *img = new QImage(256, 256, QImage::Format_RGB32);
    QImage *img_yuv = new QImage(256, 256, QImage::Format_RGB32);

    FilterYUVToRGB filterYUVToRGB(4, 1);

    unsigned char * tmp_image = NULL;
    unsigned char * imageRGB = NULL;

    tmp_image = (unsigned char *) malloc(4 * 1);

    unsigned char * lookupTable = getCurrentLookupTable();

    int y = ui->ySlider->value();
    int value;
    for(int u=0; u<256; u++) {
        for(int v=0; v<256; v++) {
            value = lookupTable[u*256 + v];

            tmp_image[0] = u;
            tmp_image[1] = y;
            tmp_image[2] = v;
            tmp_image[3] = y;

            imageRGB = filterYUVToRGB.process(tmp_image, 4 * 1 * 2);

            int r0 = *imageRGB++;
            int g0 = *imageRGB++;
            int b0 = *imageRGB++;

            int r1 = *imageRGB++;
            int g1 = *imageRGB++;
            int b1 = *imageRGB++;

            int r = (r0 + r1) / 2;
            int g = (g0 + g1) / 2;
            int b = (b0 + b1) / 2;

            QRgb rgbValue = qRgb(r, g, b);
            if (value > 0) {
                img->setPixel(u, 255 - v, rgbValue);
            } else {
                img->setPixel(u, 255 - v, qRgb(255, 255, 255));
            }
            img_yuv->setPixel(u, 255 - v, rgbValue);
        }
    }

    free(tmp_image);

    ((PolygonQGraphicsScene*) ui->graphicsView_lookup->scene())->clear();
    QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage(*img));
    item->setZValue(0.0);
    ui->graphicsView_lookup->scene()->addItem(item);


    ((PolygonQGraphicsScene*) ui->graphicsView_yuvfull->scene())->clear();
    QGraphicsPixmapItem* item_yuv = new QGraphicsPixmapItem(QPixmap::fromImage(*img_yuv));
    item_yuv->setZValue(0.0);
    ui->graphicsView_yuvfull->scene()->addItem(item_yuv);

    delete img;
    delete img_yuv;

    ui->lowerThresholdSlider->setValue(lookupTable[256 * 256 + 0]);
    ui->upperThresholdSlider->setValue(lookupTable[256 * 256 + 1]);
}

void MainWindow::updateSenderComboBox() {
    int imgIndex = ui->selectImageComboBox->currentIndex();
    if (imgIndex > -1) {
        std::list<ImageResource*>::iterator it = m_images.begin();
        std::advance(it, imgIndex);
        ImageResource* imgRes = *it;

        ui->selectRobotComboBox->setEnabled(!imgRes->isSenderFixed());
    } else {
        ui->selectRobotComboBox->setCurrentIndex(0);
        ui->selectRobotComboBox->setEnabled(true);
    }
}

bool MainWindow::saveImageToFile(ImageResource *imgRes, QString filename) {
    int lastIndex = filename.lastIndexOf(".") + 1;
    int length = filename.size() - lastIndex;
    QString extension = filename.mid(lastIndex, length);

    if (extension.compare(QString("raw"), Qt::CaseInsensitive) == 0) {
        QFileInfo f(filename);
        m_imagePwd = f.dir().absolutePath();

        QByteArray ba = filename.toLocal8Bit();
        char * path_filename = ba.data();

        FILE * imagefile = fopen(path_filename, "w");
        if(imagefile != NULL) {
            int contentSize = imgRes->getWidth() * imgRes->getHeight() * 2;
            unsigned char * content = (unsigned char *) malloc(contentSize);
            for (int i = 0; i < contentSize; i++) {
                content[i] = imgRes->getContentAt(i);
            }
            fwrite(content , sizeof(unsigned char) , contentSize , imagefile );
            fclose(imagefile);
            free(content);
        }
        return true;
    } else if (extension.compare(QString("png"), Qt::CaseInsensitive) == 0) {
        QFileInfo f(filename);
        m_imagePwd = f.dir().absolutePath();

        unsigned char * imageRGB = NULL;

        QImage *img = new QImage(imgRes->getWidth(), imgRes->getHeight(), QImage::Format_RGB32);

        FilterYUVToRGB filterYUVToRGB(imgRes->getWidth(), imgRes->getHeight());
        unsigned char * content = (unsigned char *) malloc(imgRes->getWidth() * imgRes->getHeight() * 2);
        for (int i = 0; i < imgRes->getWidth() * imgRes->getHeight() * 2; i++) {
            content[i] = imgRes->getContentAt(i);
        }
        imageRGB = filterYUVToRGB.process(content, imgRes->getWidth() * imgRes->getHeight() * 2);
        free(content);

        int img_y, img_x;
        for(img_y = 0; img_y < imgRes->getHeight(); img_y++) {
            for(img_x = 0; img_x < imgRes->getWidth(); img_x++)
            {
                int r = *imageRGB++;
                int g = *imageRGB++;
                int b = *imageRGB++;

                QRgb value = qRgb(r, g, b);
                img->setPixel(img_x, img_y, value);
            }
        }

        img->save(filename);
        delete img;
        return true;
    }
    return false;
}

void MainWindow::saveAllImagesToFile(QString suffix) {
    if (m_images.size() > 0) {
        QString qfilename = QFileDialog::getExistingDirectory(
                    this,
                    tr("Select a Directory"),
                    m_imagePwd
                    );
        if(!qfilename.isNull()) {
            qfilename.append(QString("/"));
            time_t currTime = time(NULL);
            int i = 1;
            for (std::list<ImageResource*>::iterator it = m_images.begin(); it != m_images.end(); it++, i++) {
                ImageResource* imgRes = *it;

                QString currFilename = QString("");
                if (imgRes->getSender() != -1) {
                    currFilename.append(QString("%1_").arg(imgRes->getSender()));
                }
                currFilename.append(QString("%1-").arg(currTime));
                for (int j = 1; j < 4; j++) {
                    if (i < pow(10, j)) {
                        currFilename.append(QString("0"));
                    }
                }
                currFilename.append(QString("%1").arg(i));
                currFilename.append(suffix);

                QString fullFilename = qfilename;
                fullFilename.append(currFilename);

                saveImageToFile(imgRes, fullFilename);
                /*
                if (saveImageToFile(imgRes, fullFilename)) {
                    imgRes->m_filename = currFilename.toStdString();
                    QString itemText = ui->selectImageComboBox->itemText(i - 1);
                    ui->selectImageComboBox->setItemText(i - 1, QString("%1 %2")
                                                         .arg(itemText.left(itemText.indexOf(" ")))
                                                         .arg(currFilename));
                }*/
            }
        }
    }
}

bool MainWindow::saveLookupTableToFile(unsigned char * lookupTable, QString filename) {
    QFileInfo f(filename);
    if (f.suffix().isEmpty())
        filename += ".txt";

    QFile file(filename);
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&file);

    for(int u = 0; u < 256; u++) {
        for(int v = 255; v >= 0; v--) {
            int value = lookupTable[u*256 + v];
            out << value;
            if (v > 0)
                out << " ";
        }
        out << "\n";
    }
    out << lookupTable[256 * 256 + 0] << " ";
    out << lookupTable[256 * 256 + 1];

    file.close();
    return true;
}

bool MainWindow::loadLookupTableToFile(unsigned char * lookupTable, QString filename) {
    QFileInfo f(filename);
    if (f.exists()) {
        QFile file(filename);
        file.open(QIODevice::ReadOnly | QIODevice::Text);
        QTextStream in(&file);

        int u = 0;
        QString line;
        while(!in.atEnd()) {
            line = in.readLine();
            QStringList v_values = line.split(QString(" "));
            for (int i = 0; i < v_values.size(); i++) {
                int value = v_values.at(i).toInt();
                if (u < 256) {
                    int v = 255 - i;
                    lookupTable[u*256 + v] = value;
                } else {
                    lookupTable[u*256 + i] = value;
                }
            }
            u++;
        }
        file.close();
        return true;
    }
    return false;
}

void MainWindow::updateCommands() {
    ui->undoAction->setEnabled(m_commandList->canUndo());
    ui->redoAction->setEnabled(m_commandList->canRedo());

    updateLookupTable();
    processFilter();
}

void MainWindow::handleReceivedImage(ImageResource *img) {
    addImage(img, true);
}

void MainWindow::handleReceivedSettings(const msl_sensor_msgs::CameraSettings::ConstPtr& msg) {
    if (msg->senderID > -1) {
        cout << "msg->senderID=" << msg->senderID;
        cout << endl;

        cout << "msg->useBrightness=" << (msg->useBrightness ? "true" : "false");
        cout << " msg->brightness=" << msg->brightness;
        cout << endl;

        cout << "msg->exposure=" << msg->exposure;
        cout << endl;

        cout << "msg->autoWhiteBalance=" << (msg->autoWhiteBalance ? "true" : "false");
        cout << " msg->whiteBalance1=" << msg->whiteBalance1;
        cout << " msg->whiteBalance2=" << msg->whiteBalance2;
        cout << endl;

        cout << "msg->hue=" << msg->hue;
        cout << endl;

        cout << "msg->saturation=" << msg->saturation;
        cout << endl;

        cout << "msg->enabledGamma=" << (msg->enabledGamma ? "true" : "false");
        cout << " msg->gamma=" << msg->gamma;
        cout << endl;

        cout << "msg->autoShutter=" << (msg->autoShutter ? "true" : "false");
        cout << " msg->shutter=" << msg->shutter;
        cout << endl;

        cout << "msg->autoGain=" << (msg->autoGain ? "true" : "false");
        cout << " msg->gain=" << msg->gain;
        cout << endl;

        if (m_robots.find(msg->senderID) != m_robots.end()) {
            Robot *robot = m_robots.at(msg->senderID);

            robot->setUseBrightness(msg->useBrightness);
            robot->setBrightness(msg->brightness);

            robot->setExposure(msg->exposure);

            robot->setAutoWhiteBalance(msg->autoWhiteBalance);
            robot->setWhiteBalance1(msg->whiteBalance1);
            robot->setWhiteBalance2(msg->whiteBalance2);

            robot->setHue(msg->hue);

            robot->setSaturation(msg->saturation);

            robot->setEnabledGamma(msg->enabledGamma);
            robot->setGamma(msg->gamma);

            robot->setAutoShutter(msg->autoShutter);
            robot->setShutter(msg->shutter);

            robot->setAutoGain(msg->autoGain);
            robot->setGain(msg->gain);

            if (getSelectedRobot() && getSelectedRobot()->getID() == msg->senderID) {
                emit selectedRobot(robot);
            }
        } else {
            cout << "unknown host" << endl;
        }
    }
}

void MainWindow::leftDrawnSrcImageSlot(PolygonQGraphicsScene* scn) {
    if (scn->checkModifiers(Qt::ShiftModifier)) {
        writeLookupTableSrcImage(true, true);
    } else {
        writeLookupTableSrcImage(true, false);
    }
}

void MainWindow::rightDrawnSrcImageSlot(PolygonQGraphicsScene* scn) {
    if (scn->checkModifiers(Qt::ShiftModifier)) {
        writeLookupTableSrcImage(false, true);
    } else {
        writeLookupTableSrcImage(false, false);
    }
}

void MainWindow::leftDrawnLookupSlot(PolygonQGraphicsScene* scn) {
    if (scn->checkModifiers(Qt::ControlModifier)) {
        writeLookupTableInterpolate();
    } else {
        writeLookupTableLookup(true);
    }
}

void MainWindow::rightDrawnLookupSlot(PolygonQGraphicsScene* scn) {
    writeLookupTableLookup(false);
}

void MainWindow::leftClickedLookupSlot(PolygonQGraphicsScene* scn, int x, int y) {
    int borderWidth = 1;
    if (ui->graphicsView_lookup->getZoomState() > 6)
        borderWidth = 1;//0;

    QPolygon polygon = QPolygon();
    polygon.append(QPoint(x, y));
    polygon.append(QPoint(x+1, y));
    polygon.append(QPoint(x+1, y+1));
    polygon.append(QPoint(x, y+1));
    AbstractCommand* command = new CommandYUV2Lookuptable(getCurrentLookupTable(), polygon, true);
    if (command->doIt()) {
        m_commandList->addCommand(command);
    }
}

void MainWindow::rightClickedLookupSlot(PolygonQGraphicsScene* scn, int x, int y) {
    int borderWidth = 1;
    if (ui->graphicsView_lookup->getZoomState() > 6)
        borderWidth = 1;//0;

    QPolygon polygon = QPolygon();
    polygon.append(QPoint(x, y));
    polygon.append(QPoint(x+1, y));
    polygon.append(QPoint(x+1, y+1));
    polygon.append(QPoint(x, y+1));
    AbstractCommand* command = new CommandYUV2Lookuptable(getCurrentLookupTable(), polygon, false);
    if (command->doIt()) {
        m_commandList->addCommand(command);
    }
}

void MainWindow::leftDrawnYuvfullSlot(PolygonQGraphicsScene* scn){
    writeLookupTableYuvfull(true);
}

void MainWindow::rightDrawnYuvfullSlot(PolygonQGraphicsScene* scn){
    writeLookupTableYuvfull(false);
}

void MainWindow::on_selectImageComboBox_currentIndexChanged(int index)
{
    if (index > -1) {
        std::list<ImageResource*>::iterator it = m_images.begin();
        std::advance(it, index);
        ImageResource* imgRes = *it;

        int senderIndex = ui->selectRobotComboBox->findData(imgRes->getSender());
        ui->selectRobotComboBox->setCurrentIndex(senderIndex);
    }

    showSelectedImage();
}

void MainWindow::on_prevImageButton_released()
{
    int index = ui->selectImageComboBox->currentIndex();
    if (index > -1) {
        if (index == 0) {
            index = ui->selectImageComboBox->count();
        }
        ui->selectImageComboBox->setCurrentIndex(index-1);
    }
}

void MainWindow::on_nextImageButtonButton_released()
{
    int index = ui->selectImageComboBox->currentIndex();
    if (index > -1) {
        if (index == ui->selectImageComboBox->count()-1) {
            index = -1;
        }
        ui->selectImageComboBox->setCurrentIndex(index+1);
    }
}

void MainWindow::on_selectImageCloseButton_released()
{
    int index = ui->selectImageComboBox->currentIndex();
    if (index > -1) {
        std::list<ImageResource*>::iterator it = m_images.begin();
        std::advance(it, index);
        delete *it;
        m_images.erase(it);

        ui->selectImageComboBox->removeItem(index);
    }
}

void MainWindow::on_loadImageFromSelectedROSButton_released()
{
    QTreeWidgetItemIterator it(ui->rosTreeWidget, QTreeWidgetItemIterator::Selected);
    while (*it) {
        QVariant data = (*it)->data(0, Qt::UserRole);
        NetworkCommunicator::requestImage(data.toInt());
        it++;
    }
}

void MainWindow::on_loadImageFromAllROSButton_released()
{
    QTreeWidgetItemIterator it(ui->rosTreeWidget, QTreeWidgetItemIterator::All);
    while (*it) {
        QVariant data = (*it)->data(0, Qt::UserRole);
        NetworkCommunicator::requestImage(data.toInt());
        it++;
    }
}

void MainWindow::on_lowerThresholdSlider_valueChanged(int value)
{
    ui->upperThresholdSlider->setMinimum(value);
    ui->upperThresholdSpinBox->setMinimum(value);
}

void MainWindow::on_upperThresholdSlider_valueChanged(int value)
{
    ui->lowerThresholdSlider->setMaximum(value);
    ui->lowerThresholdSpinBox->setMaximum(value);
}

void MainWindow::on_thresholdSlider_sliderReleased()
{
    AbstractCommand* command = new CommandThresholds2Lookuptable(getCurrentLookupTable(), ui->lowerThresholdSlider->value(), ui->upperThresholdSlider->value());
    if (command->doIt()) {
        m_commandList->addCommand(command);
    }
}

void MainWindow::on_thresholdSpinBox_editingFinished()
{
    AbstractCommand* command = new CommandThresholds2Lookuptable(getCurrentLookupTable(), ui->lowerThresholdSpinBox->value(), ui->upperThresholdSpinBox->value());
    if (command->doIt()) {
        m_commandList->addCommand(command);
    }
}

void MainWindow::on_clearLookuptableButton_released()
{
    initLookupTable(false);

    updateLookupTable();
    processFilter();
}

void MainWindow::on_defaultLookuptableButton_released()
{
    initLookupTable(true);

    updateLookupTable();
    processFilter();
}

void MainWindow::on_ySlider_valueChanged(int value)
{
    updateLookupTable();
}

void MainWindow::on_emThresholdSlider_valueChanged(int value)
{
    int tmpInt = ui->emThresholdSpinBox->value() * 100;
    double tmpDouble = (double) value / 100000;
    double newValue = tmpInt / 100.0 + tmpDouble;

    double oldValue = ui->emThresholdSpinBox->value();
    tmpInt = oldValue * 100000;
    oldValue = (double) tmpInt / 100000;

    if (oldValue != newValue) {
        ui->emThresholdSpinBox->setValue(newValue);
    }
}

void MainWindow::on_emThresholdSpinBox_valueChanged(double value)
{
    double tmpDouble = value * 100;
    tmpDouble = tmpDouble - (int) tmpDouble;
    int newValue = tmpDouble * 1000;

    if (ui->emThresholdSlider->value() != newValue) {
        ui->emThresholdSlider->setValue(newValue);
    }
}

void MainWindow::on_selectRobotComboBox_currentIndexChanged(int index)
{
    int imgIndex = ui->selectImageComboBox->currentIndex();
    if (imgIndex > -1) {
        std::list<ImageResource*>::iterator it = m_images.begin();
        std::advance(it, imgIndex);
        ImageResource* imgRes = *it;

        int senderID = ui->selectRobotComboBox->itemData(index).toInt();
        imgRes->setSender(senderID);
    }

    if (index > -1) {
        int robotID = ui->selectRobotComboBox->itemData(index).toInt();

        if (!ROSCommunicator::isROScoreRunning()) {
            cout << "roscore is not running" << endl;
        } else {
            if (!ROSCommunicator::isInitialized()) {
                ROSCommunicator::initialize();
                sleep(1); // wait for ros
            }

            if (ros::ok()) {
                cout << "request camera values from: " << robotID << endl;
                vector<int> receiverIDs;
                receiverIDs.push_back(robotID);
                ROSCommunicator::requestSettings(receiverIDs);
            }
        }
    }

    updateLookupTable();
    processFilter();

    emit selectedRobot(getSelectedRobot());
}

void MainWindow::on_loadImageFromFileAction_triggered()
{
    QFileDialog dialog(this);
    dialog.setDirectory(m_imagePwd);
    dialog.setFileMode(QFileDialog::ExistingFiles);
    dialog.setNameFilter(tr("RAW (*.raw);; All files (*.*)"));

    if (dialog.exec()) {
        QStringList qfilenames = dialog.selectedFiles();

        for (int i = 0; i < qfilenames.size(); ++i) {
            QString qfilename = qfilenames.at(i);

            QFileInfo f(qfilename);
            m_imagePwd = f.dir().absolutePath();

            QByteArray ba = qfilename.toLocal8Bit();
            char * path_filename = ba.data();

            FILE * imagefile = fopen(path_filename, "r");

            if(imagefile != NULL) {
                int width = CC_IMAGE_WIDTH;
                int height = CC_IMAGE_HEIGHT;

                unsigned char * content = (unsigned char *) malloc(width * height * 2);
                fread(content, sizeof(char), width * height * 2, imagefile);
                fclose(imagefile);
                ImageResource* img = new ImageResource(content, width, height);
                int lastIndex = qfilename.lastIndexOf("/") + 1;
                int length = qfilename.size() - lastIndex;
                QString filename = qfilename.mid(lastIndex, length);
                img->m_filename = filename.toStdString();
                if (filename.indexOf("_") > -1) {
                    int robotID = filename.mid(0, filename.indexOf("_")).toInt();
                    if (m_robots.find(robotID) != m_robots.end()) {
                        img->setSender(robotID);
                        img->setSenderIsFixed(true);
                    }
                }
                addImage(img, i == qfilenames.size()-1);
                free(content);
            }
            else {
                cout << "LOGFILE IS NULL" << endl;
            }
        }
    }
}

void MainWindow::on_saveImageToFileAction_triggered()
{
    int index = ui->selectImageComboBox->currentIndex();
    if (index > -1) {
        std::list<ImageResource*>::iterator it = m_images.begin();
        std::advance(it, index);
        ImageResource* imgRes = *it;

        QString sf;
        QString qfilename = QFileDialog::getSaveFileName(
                    this,
                    tr("Save File"),
                    m_imagePwd,
                    tr("RAW (*.raw);; PNG (*.png)"),
                    &sf
                    );
        if (qfilename != NULL) {
            QFileInfo f(qfilename);
            if (f.suffix().isEmpty())
                qfilename += sf.mid(sf.indexOf("."), sf.indexOf(")") - sf.indexOf("."));
            saveImageToFile(imgRes, qfilename);
        }
    }
}

void MainWindow::on_saveAllImagesToRAWFileAction_triggered()
{
    saveAllImagesToFile(".raw");
}

void MainWindow::on_saveAllImagesToPNGFileAction_triggered()
{
    saveAllImagesToFile(".png");
}

void MainWindow::on_loadLookupTableFromFileAction_triggered()
{
    QString qfilename = QFileDialog::getOpenFileName(this, tr("Open File"),
                                                     getLookupTablePwd(),
                                                     tr("TXT (*.txt);; All files (*.*)"));
    if (qfilename != NULL) {
        loadLookupTableToFile(getCurrentLookupTable(), qfilename);

        m_commandList->clear();

        ui->inputTabWidget->setCurrentIndex(1);

        updateLookupTable();
        processFilter();
    }
}

void MainWindow::on_saveLookupTableToFileAction_triggered()
{
    QString qfilename = QFileDialog::getSaveFileName(this, tr("Save File"), getLookupTablePwd(), tr("TXT (*.txt)"));
    if (qfilename != NULL) {
        saveLookupTableToFile(getCurrentLookupTable(), qfilename);
    }
}

void MainWindow::on_closeAction_triggered()
{
    this->close();
}

void MainWindow::on_undoAction_triggered()
{
    m_commandList->undo();

    updateLookupTable();
    processFilter();
}

void MainWindow::on_redoAction_triggered()
{
    m_commandList->redo();

    updateLookupTable();
    processFilter();
}

void MainWindow::on_rectAction_triggered()
{
    ((PolygonQGraphicsScene*) ui->graphicsView->scene())->setDrawingMode(RECT);
    ((PolygonQGraphicsScene*) ui->graphicsView_lookup->scene())->setDrawingMode(RECT);
    ((PolygonQGraphicsScene*) ui->graphicsView_yuvfull->scene())->setDrawingMode(RECT);
}

void MainWindow::on_polyAction_triggered()
{
    ((PolygonQGraphicsScene*) ui->graphicsView->scene())->setDrawingMode(POLYGON);
    ((PolygonQGraphicsScene*) ui->graphicsView_lookup->scene())->setDrawingMode(POLYGON);
    ((PolygonQGraphicsScene*) ui->graphicsView_yuvfull->scene())->setDrawingMode(POLYGON);
}

void MainWindow::on_pushButton_released()
{
    CameraSettingsDialog* cameraSettingsDialog;
    //cameraSettingsDialog = new CameraSettingsDialog(this);
    cameraSettingsDialog = CameraSettingsDialog::getInstance(this);
    cameraSettingsDialog->show();
}

void MainWindow::on_drawLinesCheckBox_toggled(bool checked)
{
    processFilter();
}

void MainWindow::on_searchBallCheckBox_toggled(bool checked)
{
    ui->drawBallCheckBox->setEnabled(checked);
    ui->drawROICheckBox->setEnabled(checked);
    if (!checked) {
        ui->drawBallCheckBox->setChecked(false);
        ui->drawROICheckBox->setChecked(false);
    }
    processFilter();
}

void MainWindow::on_drawBallCheckBox_toggled(bool checked)
{
    processFilter();
}

void MainWindow::on_drawROICheckBox_toggled(bool checked)
{
    processFilter();
}

void MainWindow::on_processImageSelectionComboBox_currentIndexChanged(int index)
{
    processFilter();
}

void MainWindow::on_pushButton_2_released()
{
    int index = ui->selectImageComboBox->currentIndex();
    if (index > -1) {
        std::list<ImageResource*>::iterator it = m_images.begin();
        std::advance(it, index);
        ImageResource* imgRes = *it;

        unsigned char * content = (unsigned char *) malloc(imgRes->getWidth() * imgRes->getHeight() * 2);
        for (int i = 0; i < imgRes->getWidth() * imgRes->getHeight() * 2; i++) {
            content[i] = imgRes->getContentAt(i);
        }

        unsigned char * lookupTable = getCurrentLookupTable();

        int u = -1, v = -1, y;
        int img_y, img_x;
        for(img_y = 0; img_y < imgRes->getHeight(); img_y++) {
            for(img_x = 0; img_x < imgRes->getWidth() * 2; img_x += 2)
            {
                if ((img_x / 2) % 2 == 0)
                    u  = content[img_y * imgRes->getWidth() * 2 + img_x];
                else
                    v  = content[img_y * imgRes->getWidth() * 2 + img_x];
                y = content[img_y * imgRes->getWidth() * 2 + img_x + 1];
                /*
164 173 54 126
197 211 83 147
257 284 61 115
395 212 99 100
355 284 66 114
400 290 62 87
209 84 258 60
252 57 176 43
*/
                if (img_x >= 164 *2 && img_x <= (164 + 54) *2 &&
                        img_y >= 173 && img_y <= 173 + 126) {
                    lookupTable[u*256 + v] = 255;
                } else if (img_x >= 197 *2 && img_x <= (197 + 83) *2 &&
                           img_y >= 211 && img_y <= 211 + 147) {
                    lookupTable[u*256 + v] = 255;
                } else if (img_x >= 257 *2 && img_x <= (257 + 61) *2 &&
                           img_y >= 284 && img_y <= 284 + 115) {
                    lookupTable[u*256 + v] = 255;
                } else if (img_x >= 395 *2 && img_x <= (395 + 99) *2 &&
                           img_y >= 212 && img_y <= 212 + 100) {
                    lookupTable[u*256 + v] = 255;
                } else if (img_x >= 355 *2 && img_x <= (355 + 66) *2 &&
                           img_y >= 284 && img_y <= 284 + 114) {
                    lookupTable[u*256 + v] = 255;
                } else if (img_x >= 400 *2 && img_x <= (400 + 62) *2 &&
                           img_y >= 290 && img_y <= 290 + 87) {
                    lookupTable[u*256 + v] = 255;
                } else if (img_x >= 209 *2 && img_x <= (209 + 258) *2 &&
                           img_y >= 84 && img_y <= 84 + 60) {
                    lookupTable[u*256 + v] = 255;
                } else if (img_x >= 252 *2 && img_x <= (252 + 176) *2 &&
                           img_y >= 57 && img_y <= 57 + 43) {
                    lookupTable[u*256 + v] = 255;
                }
            }
        }

        free(content);

        updateLookupTable();
        processFilter();
    }
}
