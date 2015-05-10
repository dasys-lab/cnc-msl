/**
 * @file /include/msl_voronoi_viewer/main_window.hpp
 *
 * @brief Qt based gui for msl_voronoi_viewer.
 *
 * @date November 2010
 **/
#ifndef msl_voronoi_viewer_MAIN_WINDOW_H
#define msl_voronoi_viewer_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

//includes for CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>


/**
 * typedefs to short CGAL's class names e.g.CGAL::Voronoi_diagram_2<CGAL::Delaunay_triangulation_2<CGAL::Exact_predicates_inexact_constructions_kernel>,
CGAL::Delaunay_triangulation_adaptation_traits_2<CGAL::Delaunay_triangulation_2<CGAL::Exact_predicates_inexact_constructions_kernel>>,
CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<CGAL::Delaunay_triangulation_2<CGAL::Exact_predicates_inexact_constructions_kernel>>>
to VoronoiDiagram
 */
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Delaunay_triangulation_2<Kernel> DelaunayTriangulation;
typedef CGAL::Delaunay_triangulation_adaptation_traits_2<DelaunayTriangulation> DelaunayAdaptionTraits;
typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<DelaunayTriangulation> DelaunayAdaptionPolicy;
typedef CGAL::Voronoi_diagram_2<DelaunayTriangulation, DelaunayAdaptionTraits, DelaunayAdaptionPolicy> VoronoiDiagram;
typedef DelaunayAdaptionTraits::Point_2 Point_2;
typedef DelaunayAdaptionTraits::Site_2 Site_2;
typedef Kernel::Iso_rectangle_2 Iso_rectangle_2;
typedef Kernel::Segment_2 Segment_2;
typedef Kernel::Ray_2 Ray_2;
typedef Kernel::Line_2 Line_2;

#include <QtGui/QMainWindow>
#include <QPainter>
#include <QLine>
#include <QPainterPath>
#include "ui_voronoi_main_window.h"
#include <vector>
#include <mutex>


#include "ros/ros.h"
#include "msl_sensor_msgs/WorldModelData.h"
#include "../msl_voronoi_viewer/qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/
using namespace std;
namespace msl_voronoi_viewer {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/


    /******************************************
    ** Manual connections
    *******************************************/
    void handleShowVoronoi();
    void handleShowSites();
    void handleReset();
    void onVoronoiInfo(msl_sensor_msgs::WorldModelDataPtr msg);
    void updateLoggingView(); // no idea why this can't connect automatically
    void addExamples();

private:   


    ros::Subscriber voronoiSub;
    ros::NodeHandle* n;
    ros::AsyncSpinner* spinner;
    int robotId;
	Ui::VoronoiWindowDesign ui;
	QNode qnode;
	void paintEvent(QPaintEvent *event);
		void fillVoronoi();

	bool drawFaces;
	bool drawSites;
	shared_ptr<VoronoiDiagram> voronoi;
	vector<msl_sensor_msgs::WorldModelDataPtr> msgs;
	msl_sensor_msgs::WorldModelDataPtr currentMsg;
	mutex mtx;
};

}  // namespace msl_voronoi_viewer

#endif // msl_voronoi_viewer_MAIN_WINDOW_H
