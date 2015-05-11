/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include "../include/msl_voronoi_viewer/main_window.hpp"

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <math.h>
using namespace std;

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace msl_voronoi_viewer
{

	using namespace Qt;

	/*****************************************************************************
	 ** Implementation [MainWindow]
	 *****************************************************************************/

	struct Cropped_voronoi_from_delaunay
	{
		std::list<Segment_2> m_cropped_vd;
		Iso_rectangle_2 m_bbox;

		Cropped_voronoi_from_delaunay(const Iso_rectangle_2& bbox) :
				m_bbox(bbox)
		{
		}

		template<class RSL>
		void crop_and_extract_segment(const RSL& rsl)
		{
			CGAL::Object obj = CGAL::intersection(rsl, m_bbox);
			const Segment_2* s = CGAL::object_cast<Segment_2>(&obj);
			if (s)
				m_cropped_vd.push_back(*s);
		}

		void operator<<(const Ray_2& ray)
		{
			crop_and_extract_segment(ray);
		}
		void operator<<(const Line_2& line)
		{
			crop_and_extract_segment(line);
		}
		void operator<<(const Segment_2& seg)
		{
			crop_and_extract_segment(seg);
		}
	};

	MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
			QMainWindow(parent), qnode(argc, argv)
	{
		index = 0;
		msgs = vector<msl_sensor_msgs::WorldModelDataPtr>(10);
		for (int i = 0; i < msgs.size(); i++)
		{
			msgs.at(i) = nullptr;
		}
		currentMsg = nullptr;
		n = new ros::NodeHandle();
		ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
		QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

		setWindowIcon(QIcon(":/images/icon.png"));
		QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
		QObject::connect(ui.showVoroBtn, SIGNAL(released()), this, SLOT(handleShowVoronoi()));
		QObject::connect(ui.showSitesBtn, SIGNAL(released()), this, SLOT(handleShowSites()));
		QObject::connect(ui.resetSitesBtn, SIGNAL(released()), this, SLOT(handleReset()));
		QObject::connect(ui.actionAdd_Examples, SIGNAL(triggered()), this, SLOT(addExamples()));

		/*********************
		 ** Logging
		 **********************/
		QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

		/*********************
		 ** Auto Start
		 **********************/

		//joystickpub = n.advertise<msl_msgs::JoystickCommand>("/Joystick", 1);
		voronoiSub = n->subscribe("/WorldModel/WorldModelData", 10, &MainWindow::onVoronoiInfo, (MainWindow*)this);
		spinner = new ros::AsyncSpinner(2);
		spinner->start();
		robotId = 0;
		drawFaces = false;
		drawSites = false;
		this->voronoi = make_shared<VoronoiDiagram>();
	}

	MainWindow::~MainWindow()
	{
		//thios is not correctly deleted
		delete spinner;

	}

	/*****************************************************************************
	 ** Implementation [Slots]
	 *****************************************************************************/

	void MainWindow::showNoMasterMessage()
	{
		QMessageBox msgBox;
		msgBox.setText("Couldn't find the ros master.");
		msgBox.exec();
		close();
	}

	void MainWindow::onVoronoiInfo(msl_sensor_msgs::WorldModelDataPtr msg)
	{
		lock_guard<mutex> lock(mtx);
		msgs.push_back(msg);
	}

	/*
	 * These triggers whenever the button is clicked, regardless of whether it
	 * is already checked or not.
	 */

	/*****************************************************************************
	 ** Implemenation [Slots][manually connected]
	 *****************************************************************************/

	/**
	 * This function is signalled by the underlying model. When the model changes,
	 * this will drop the cursor down to the last line in the QListview to ensure
	 * the user can always see the latest log message.
	 */
	void MainWindow::updateLoggingView()
	{
	}

	void MainWindow::addExamples()
	{
		if(index == 0)
		{
		this->voronoi->insert(Site_2(100, 100));
		this->voronoi->insert(Site_2(200, 100));
		this->voronoi->insert(Site_2(200, 500));
		}
		if(index == 1)
		{
		this->voronoi->insert(Site_2(100, 900));
		this->voronoi->insert(Site_2(700, 600));
		this->voronoi->insert(Site_2(800, 100));
		this->voronoi->insert(Site_2(900, 100));
		}
		if(index == 2)
		{
		this->voronoi->insert(Site_2(1000, 100));
		this->voronoi->insert(Site_2(1100, 500));
		this->voronoi->insert(Site_2(600, 900));
		this->voronoi->insert(Site_2(700, 900));
		this->voronoi->insert(Site_2(800, 1000));
		}
		if(index == 3)
		{
			for(auto it = this->voronoi->dual().vertices_begin();
					 it != this->voronoi->dual().vertices_end(); it++)
			{
				if(it->point().x() == 1000 && it->point().y() == 100)
				{
					cout << "removing" << endl;
					((DelaunayTriangulation)this->voronoi->dual()).remove(it->handle());
					cout << "removing" << endl;
				}
			}
		}
		index++;
	}

	void MainWindow::fillVoronoi()
	{
		lock_guard<mutex> lock(mtx);
		if (currentMsg == nullptr)
		{
			currentMsg = msgs.at(0);
		}
		if (currentMsg != nullptr)
		{
			for (int i = 0; i < currentMsg->obstacles.size(); i++)
			{
				Site_2 site = Site_2(currentMsg->obstacles.at(i).x + ui.centralwidget->width() / 2,
										currentMsg->obstacles.at(i).y + ui.centralwidget->height() / 2);
				voronoi->insert(site);
			}
		}
	}

	void MainWindow::handleShowVoronoi()
	{
		fillVoronoi();
		drawFaces = true;
		update();
	}

	void MainWindow::handleShowSites()
	{
		fillVoronoi();
		drawSites = true;
		update();
	}

	void MainWindow::handleReset()
	{
		drawSites = false;
		drawFaces = false;
		currentMsg = nullptr;
		voronoi = nullptr;
		voronoi = make_shared<VoronoiDiagram>();
		update();
	}

	/*****************************************************************************
	 ** Implementation [Menu]
	 *****************************************************************************/

	void MainWindow::closeEvent(QCloseEvent *event)
	{
		QMainWindow::closeEvent(event);
	}

	void MainWindow::paintEvent(QPaintEvent* event)
	{
		ui.frame->move(ui.centralwidget->size().width() / 2 - ui.frame->width() / 2,
						ui.centralwidget->size().height() - 2 * ui.frame->height());
		if (drawFaces)
		{
			int i = 2;
			QPainter painter(this);
			QPen pen(Qt::black, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
			Iso_rectangle_2 bbox(0, 0, ui.centralwidget->size().width(), ui.centralwidget->size().height());
			Cropped_voronoi_from_delaunay vor(bbox);
			DelaunayTriangulation dt = this->voronoi->dual();
			dt.draw_dual(vor);
			for (auto it = vor.m_cropped_vd.begin(); it != vor.m_cropped_vd.end(); it++)
			{
				QLine line;
				if (!std::isnan(it->source().x()) && !std::isnan(it->source().y()) && !std::isnan(it->target().x())
						&& !std::isnan(it->target().y()))
				{
					line.setP1(QPoint(it->source().x(), it->source().y()));
					line.setP2(QPoint(it->target().x(), it->target().y()));
					painter.setPen(pen);
					painter.drawLine(line);
				}
			}
		}

		if (drawSites)
		{
			QPainter painter(this);
			QPen pen(Qt::red, 10, Qt::DashDotLine, Qt::RoundCap, Qt::RoundJoin);

			for (auto it = voronoi->sites_begin(); it != voronoi->sites_end(); it++)
			{
				QPoint point;
				point.setX(it->x());
				point.setY(it->y());
				painter.setPen(pen);
				painter.drawPoint(point);
			}
		}

	}

} // namespace msl_voronoi_viewer

