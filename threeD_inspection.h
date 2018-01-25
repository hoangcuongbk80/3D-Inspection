#ifndef threeD_Inspection_H
#define threeD_Inspection_H

#include <QtWidgets/QMainWindow>
#include "ui_threeD_Inspection.h"
#include "cinspection.h"
#include "rotaTable.h"

#define vtkRenderingCore_AUTOINIT 4(vtkInteractionStyle,vtkRenderingFreeType,vtkRenderingFreeTypeOpenGL,vtkRenderingOpenGL)
#define vtkRenderingVolume_AUTOINIT 1(vtkRenderingVolumeOpenGL)

#include <tchar.h>

// Qt
#include <QMainWindow>
#include <QFileDialog>
#include <QDir>
#include <QDateTime>
#include <QDebug>
#include <QTimer>
#include <QtGui>
#include <QColorDialog>
#include <QInputDialog> 
#include <QtSerialPort/QSerialPort>
#include <QtCore/QtGlobal>
#include <QMessageBox>

// TCP-IP
#include <QTcpSocket>
#include <QtNetwork>
#include "QHostInfo"
#include <QDataStream>
#include <QStringList>
#include <QStringListModel>
#include <QAbstractItemView>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/file_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/common/file_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/common/geometry.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleSwitch.h>

// Boost
#include <boost/math/special_functions/round.hpp>

//Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
#include <vtkColorSeries.h>
#include <vtkOutputWindow.h>
#include <vtkFileOutputWindow.h>
#include <vtkVectorText.h>
#include <vtkElevationFilter.h>
#include <vtkRenderer.h>
#include <vtkPolyDataMapper.h>
#include "vtkSmartPointer.h"

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::visualization;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;

class threeD_Inspection : public QMainWindow
{
	Q_OBJECT

public:
	threeD_Inspection(QWidget *parent = 0);
	~threeD_Inspection();

	// TCP-IP
	int viewIndex;
	int scanMethod;
	int rotaPos;
	bool robotScan;
	bool frameScan;
	bool robotMove;
	bool robotPause;
	bool writeToProbe;
	bool moveOrigin;
	bool showPoints;
	Eigen::Matrix4f RegisMatr;
	QString dataToProbe;
	QString kukaForm;
	QTcpServer *server_Kuka;
	QTcpSocket *socket_Kuka;
	QTcpServer *server_Probe;
	QTcpSocket *socket_Probe;

public slots:
    void loadFileButtonPressed(); 
	void saveButtonPressed(); 
	void Reset_3D_ViewPressed();
	void DownsamplePressed();
	void DenoisePressed();
	void DenoiseByNormalsPressed();
	void Reload_CloudPressed(); 
	void Euclidean_Cluster_ExtractionPressed();
	void Cylinder_FittingPressed();
	void OBB_Recognition();
	void SubOBB_Recognition();
	void RG_SubOBB_Recognition();
	void AutoScanningPressed();
	void DownloadPressed();
	void Pick_and_Place_TestPressed();
	void Denoise_by_segmentPressed();
	void OBB_Draw(const Cloud &, QString, bool);
	void Coordinate_Draw(bool);
	void Cloud_Display(const Cloud &, QString, int, bool, bool, bool);
	void Show_OBB();
	void Recognition_Error_ReportPressed();
	void Initial_Transformation_VirtualCam();
	void Generate_Database_VirtualCam();
	void delay(int);
	void Targe_Cloud();
	void Source_Cloud();
	void OBB_Matching();
	void ICPPress();
	void Frame_ICPPress();
	void ModDri_ICPPress();
	void Registration_Error_Report();
	void Initial_2D_Graph();
	void Line_2D_Graph();
	void Update_Line_2D_Graph(pcl::PointCloud<pcl::PointXYZ> &input);
	void Sphere_Detection();
	void Regis_System_Positioning();
	void AutoRegis_System_Positioning();
	void Upsampling();
	void AutoMode_Pause();
	void AutoMode_Continue();
	void Normals();
	void Regis_Model_Driven();
	void Reset_Project();
	void Generate_Opt_Database();
	void Recog_SubOBB_Checkbox();
	void Recog_OSubOBB_Checkbox();
	void Recog_nonSegSubOBB_Checkbox();
	void Non_Segmentation_SubOBB_Recognition();
	void Gennerate_NonSeg_Database();
	void Overlap_RemovalPress();
	void FrameBall_DetectionPress();
	void Frame_Matching();
	void FrameBased_CoarseRegistration();
	void AutoFrameBased_CoarseRegistration();
	void Frame_Based_Scan();
	void Licensing();
	void OnlineRobotPositiningRegitration();
	void OnlineFrameBasedRegitration();
	void TestFrameCali_Start();
	void TestFrame_Pause();
	void TestFrame_Continue();
	void TestFrame_SaveTransf();
	void OnlineFrameCalibration();
	void SaveSingleScan();
	void SaveRegisScan();

	// Kuka TCP-IP
	void bt_ListenKukaButtonPressed();
	void Origin_KukaPressed();
	void myConnection_Kuka();
	void writeData_Kuka();
	void readData_Auto_Kuka();
	void readData_Kuka();
	void Move_KukaPressed();

	// Probe TCP-IP
	void bt_ListenProbeButtonPressed();
	void readData_Auto_Probe();
	void myConnection_Probe();
	void writeData_Probe();

	// Rotational Table
	void rotaTable_Connect();
	void rotaTable_Disconnect();
	void rotaTable_Move_60_Degree();
	void rotaTab_ZeroPos();
	void rotaTable_Move();

	// Examples
	void ICP_ExamplePressed();
	void SVD_ExamplePressed();
	void ToolTestPress();

protected:
	/** @brief The PCL visualizer object */
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
	pcl::visualization::PCLPlotter plotter;
	cinspection *cInspection = new cinspection();
	rotaTable *rotationalTable = new rotaTable();

	/** @brief The point cloud displayed */
	PointCloudT::Ptr cloud_;
	PointCloudT::Ptr source_;
	PointCloudT::Ptr target_;
	PointCloudT::Ptr segmented_cloud;
	PointCloudT::Ptr recognised_cloud;
	PointCloudT::Ptr model_cloud;
	PointCloudT::Ptr Segmented_Indicates;
	PointCloudT::Ptr features;
	PointCloudT::Ptr center_features;
	pcl::PointCloud<pcl::PointXYZ>::Ptr frameTrans_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr LoadframeTrans_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz;

private:
	Ui::threeD_InspectionClass ui;
};

#endif // threeD_Inspection_H
