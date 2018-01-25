#include "threeD_Inspection.h"

threeD_Inspection::threeD_Inspection(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	ui.comboBox_ObjRecog->addItem("Pipe_Fitting");
	ui.comboBox_ObjRecog->addItem("Cylinder");
	ui.comboBox_ObjRecog->addItem("Hammer");
	ui.comboBox_ObjRecog->addItem("L_Shape"); 
	ui.comboBox_ObjRecog->addItem("Retangle");
	ui.comboBox_ObjRecog->addItem("Syntec");
	ui.comboBox_ObjRecog->addItem("Big_Blade");
	ui.comboBox_ObjRecog->addItem("Random");
	//------------------------------------------------------------------------------------------------

	ui.comboBox_Scan_Obj->addItem("Aluminum_Turbine_Blade");
	ui.comboBox_Scan_Obj->addItem("Composite_Turbine_Blade");

	ui.comboBox_FrameSize_Obj->addItem("median"); 
	ui.comboBox_FrameSize_Obj->addItem("tall");
	ui.comboBox_FrameSize_Obj->addItem("short");
	//------------------------------------------------------------------------------------------------
	// Setup the TCP-IP
	server_Kuka = new QTcpServer(this);
	socket_Kuka = new QTcpSocket(this);
	server_Probe = new QTcpServer(this);
	socket_Probe = new QTcpSocket(this);
	viewIndex = 0;
	scanMethod = 0;
	rotaPos = 0;
	robotScan = false;
	robotMove = false;
	robotPause = false;
	frameScan = false;
	writeToProbe = false;
	moveOrigin = false;
	showPoints = true;
	RegisMatr = Eigen::Matrix4f::Identity();

	ui.comboBox_Kuka->addItem("Auto");
	ui.comboBox_Kuka->addItem("Manually");
	ui.comboBox_Probe->addItem("Auto");
	ui.comboBox_Probe->addItem("Manually");
	//------------------------------------------------------------------------------------------------
	// Setup the cloud pointer
	cloud_.reset(new PointCloudT);  
	source_.reset(new PointCloudT);
	target_.reset(new PointCloudT);
	segmented_cloud.reset(new PointCloudT);
	Segmented_Indicates.reset(new PointCloudT);
	features.reset(new PointCloudT);
	center_features.reset(new PointCloudT); 
	recognised_cloud.reset(new PointCloudT);
	model_cloud.reset(new PointCloudT);
	frameTrans_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	LoadframeTrans_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	cloudxyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
	//------------------------------------------------------------------------------------------------
	// Set up the QVTK window
	viewer_.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	viewer_->setBackgroundColor(0, 0, 0);
	ui.qvtkWidget->SetRenderWindow(viewer_->getRenderWindow());
	viewer_->setupInteractor(ui.qvtkWidget->GetInteractor(), ui.qvtkWidget->GetRenderWindow());
	viewer_->addOrientationMarkerWidgetAxes(ui.qvtkWidget->GetInteractor());
	ui.qvtkWidget->update();
	//------------------------------------------------------------------------------------------------
	// Set up 2D graph
	Initial_2D_Graph();
	ui.qvtkWidget_graph->update();
	Licensing();
	//------------------------------------------------------------------------------------------------
	// Functions
	connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(loadFileButtonPressed()));
	connect(ui.actionSave, SIGNAL(triggered()), this, SLOT(saveButtonPressed())); 
	connect(ui.actionReset_3D_View, SIGNAL(triggered()), this, SLOT(Reset_3D_ViewPressed())); 
	connect(ui.actionReload_Cloud, SIGNAL(triggered()), this, SLOT(Reload_CloudPressed()));
	connect(ui.actionDownsample, SIGNAL(triggered()), this, SLOT(DownsamplePressed()));
	connect(ui.actionDenoise, SIGNAL(triggered()), this, SLOT(DenoisePressed()));  
	connect(ui.actionDenoise_by_normals, SIGNAL(triggered()), this, SLOT(DenoiseByNormalsPressed()));
	connect(ui.actionEuclidean_Extraction, SIGNAL(triggered()), this, SLOT(Euclidean_Cluster_ExtractionPressed())); 
	connect(ui.actionCylinder_Fitting, SIGNAL(triggered()), this, SLOT(Cylinder_FittingPressed())); 
	connect(ui.actionOBB_Recognition, SIGNAL(triggered()), this, SLOT(OBB_Recognition()));
	connect(ui.actionAutoScanning, SIGNAL(triggered()), this, SLOT(AutoScanningPressed())); 
	connect(ui.actionDownload, SIGNAL(triggered()), this, SLOT(DownloadPressed()));  
	connect(ui.actionPick_and_Place_Test, SIGNAL(triggered()), this, SLOT(Pick_and_Place_TestPressed()));
	connect(ui.actionDenoise_by_segment, SIGNAL(triggered()), this, SLOT(Denoise_by_segmentPressed())); 
	connect(ui.actionShow_OBB, SIGNAL(triggered()), this, SLOT(Show_OBB())); 
	connect(ui.actionRecognition_Error_Report, SIGNAL(triggered()), this, SLOT(Recognition_Error_ReportPressed()));
	connect(ui.actionSubOBB_Recognition, SIGNAL(triggered()), this, SLOT(SubOBB_Recognition())); 
	connect(ui.actionInitial_Transformation_VirtualCam, SIGNAL(triggered()), this, SLOT(Initial_Transformation_VirtualCam())); 
	connect(ui.actionGenerate_Database_VirtualCam, SIGNAL(triggered()), this, SLOT(Generate_Database_VirtualCam())); 
	connect(ui.actionTarge_Cloud, SIGNAL(triggered()), this, SLOT(Targe_Cloud()));
	connect(ui.actionSource_Cloud, SIGNAL(triggered()), this, SLOT(Source_Cloud())); 
	connect(ui.actionOBB_Matching, SIGNAL(triggered()), this, SLOT(OBB_Matching())); 
	connect(ui.actionICP, SIGNAL(triggered()), this, SLOT(ICPPress())); 
	connect(ui.actionRegistration_Error_Report, SIGNAL(triggered()), this, SLOT(Registration_Error_Report())); 
	connect(ui.actionRG_SubOBB, SIGNAL(triggered()), this, SLOT(RG_SubOBB_Recognition())); 
	connect(ui.actionToolTest, SIGNAL(triggered()), this, SLOT(ToolTestPress())); 
	connect(ui.actionSphere_Detection, SIGNAL(triggered()), this, SLOT(Sphere_Detection())); 
	connect(ui.action_Regis_System_Positioning, SIGNAL(triggered()), this, SLOT(Regis_System_Positioning())); 
	connect(ui.actionUpsampling, SIGNAL(triggered()), this, SLOT(Upsampling())); 
	connect(ui.action_AutoMode_Pause, SIGNAL(triggered()), this, SLOT(AutoMode_Pause())); 
	connect(ui.action_AutoMode_Continue, SIGNAL(triggered()), this, SLOT(AutoMode_Continue())); 
	connect(ui.actionNormals, SIGNAL(triggered()), this, SLOT(Normals())); 
	connect(ui.action_Regis_Model_Driven, SIGNAL(triggered()), this, SLOT(Regis_Model_Driven())); 
	connect(ui.actionReset_Project, SIGNAL(triggered()), this, SLOT(Reset_Project())); 
	connect(ui.actionGenerate_Opt_Database, SIGNAL(triggered()), this, SLOT(Generate_Opt_Database())); 
	connect(ui.actionGennerate_NonSeg_Database, SIGNAL(triggered()), this, SLOT(Gennerate_NonSeg_Database()));  
	connect(ui.actionOverlap_Removal, SIGNAL(triggered()), this, SLOT(Overlap_RemovalPress())); 
	connect(ui.action_FrameBall_Detection, SIGNAL(triggered()), this, SLOT(FrameBall_DetectionPress())); 
	connect(ui.actionFrame_Matching, SIGNAL(triggered()), this, SLOT(Frame_Matching())); 
	connect(ui.actionFrame_Based_Registration, SIGNAL(triggered()), this, SLOT(FrameBased_CoarseRegistration())); 
	connect(ui.actionFrame_Based_Scan, SIGNAL(triggered()), this, SLOT(Frame_Based_Scan())); 

	connect(ui.action_FrameScan_Start, SIGNAL(triggered()), this, SLOT(TestFrameCali_Start()));
	connect(ui.action_ScanFrame_Pause, SIGNAL(triggered()), this, SLOT(TestFrame_Pause()));
	connect(ui.action_FrameScan_Continue, SIGNAL(triggered()), this, SLOT(TestFrame_Continue()));
	connect(ui.action_FrameCaliSaveTransf, SIGNAL(triggered()), this, SLOT(TestFrame_SaveTransf()));

	// Checkbox
	connect(ui.checkBox_SubOBB_Recognition, SIGNAL(released()), this, SLOT(Recog_SubOBB_Checkbox()));
	connect(ui.checkBox_OSubOBB_Recognition, SIGNAL(released()), this, SLOT(Recog_OSubOBB_Checkbox()));
	connect(ui.checkBox_nonSeg_Recognition, SIGNAL(released()), this, SLOT(Recog_nonSegSubOBB_Checkbox()));

	// Examples
	connect(ui.actionICP_Example, SIGNAL(triggered()), this, SLOT(ICP_ExamplePressed())); 
	connect(ui.actionSVD_Example, SIGNAL(triggered()), this, SLOT(SVD_ExamplePressed()));

	// Kuka communication
	connect(ui.bt_Listen_Kuka, SIGNAL(pressed()), this, SLOT(bt_ListenKukaButtonPressed()));
	connect(ui.bt_Send_Kuka, SIGNAL(pressed()), this, SLOT(writeData_Kuka()));
	connect(ui.bt_Read_Kuka, SIGNAL(pressed()), this, SLOT(readData_Kuka()));
	connect(server_Kuka, SIGNAL(newConnection()), this, SLOT(myConnection_Kuka())); 
	connect(ui.bt_Origin_Kuka, SIGNAL(pressed()), this, SLOT(Origin_KukaPressed())); 
	connect(ui.bt_Move_Kuka, SIGNAL(pressed()), this, SLOT(Move_KukaPressed()));

	// Probe Communication
	connect(ui.bt_Listen_Probe, SIGNAL(pressed()), this, SLOT(bt_ListenProbeButtonPressed()));
	connect(server_Probe, SIGNAL(newConnection()), this, SLOT(myConnection_Probe()));

	// Rotational Table
	connect(ui.action_rotaTable_Connect, SIGNAL(triggered()), this, SLOT(rotaTable_Connect()));
	connect(ui.action_rotaTable_Disconnect, SIGNAL(triggered()), this, SLOT(rotaTable_Disconnect()));
	connect(ui.action_rotaTable_Move_60_Degree, SIGNAL(triggered()), this, SLOT(rotaTable_Move_60_Degree()));
	connect(ui.action_rotaTab_ZeroPos, SIGNAL(triggered()), this, SLOT(rotaTab_ZeroPos()));
}

threeD_Inspection::~threeD_Inspection()
{
	rotationalTable->Move("X0");
	delay(500);
	server_Kuka->close();
	delete server_Kuka;
	server_Probe->close();
	delete server_Probe;
	fnPerformaxComClose(rotationalTable->m_hUSBDevice);
}

void threeD_Inspection::loadFileButtonPressed()
{
	QElapsedTimer timer;
	QString filename = QFileDialog::getOpenFileName(this, tr("Open point cloud"), "", tr("Point cloud data (*.pcd *.ply *.txt)"));
	//QString filename = "//aidc-ntu3-pc/Users/Public/pointcloud.pcd";
	//ui.textEdit->append(filename);
	
	if (filename.isEmpty()) return;
	if (!cloud_->empty())cloud_->clear();
	if (!cloudxyz->empty())cloudxyz->clear();
	if (filename.endsWith(".pcd", Qt::CaseInsensitive))
	{
		if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename.toStdString(), *cloudxyz) == -1) //* load the file
			ui.textEdit->append("Couldn't read file.pcd \n");
	}
	else if (filename.endsWith(".txt", Qt::CaseInsensitive))
	{
		if (cInspection->loadCloud_txt(filename.toStdString(), *cloudxyz) == 0) //* load the file
			ui.textEdit->append("Couldn't read file.txt \n");
	}
	else ui.textEdit->append("File must be .pcd or .txt");

	// If point cloud contains NaN values, remove them before updating the visualizer point cloud
	if (cloudxyz->size())
	{
		pcl::copyPointCloud(*cloudxyz, *cloud_);
		//Eigen::Affine3f transformMatrix = Eigen::Affine3f::Identity();
		//transformMatrix.rotate(Eigen::AngleAxisf(M_PI * ((float)180 / 180), Eigen::Vector3f::UnitX())); // camera to robot-end-effector
		//pcl::transformPointCloud(*cloud_, *cloud_, transformMatrix);
		ui.textEdit->append("Number of Points: " + QString::number(cloud_->size()));
		Cloud_Display(*cloud_, "Cloud", 2, true, true, true);
	}
	else ui.textEdit->append("No point was loaded");
}

void threeD_Inspection::DownloadPressed()
{
	//QString filename = QFileDialog::getOpenFileName(this, tr("Open point cloud"), "", tr("Point cloud data (*.pcd *.ply *.txt)"));
	QString filename = "//aidc-ntu3-pc/Users/Public/pointcloud.pcd";
	//ui.textEdit->append(filename);
	if (!cloud_->empty())cloud_->clear();
	if (!cloudxyz->empty())cloudxyz->clear();

	if (filename.isEmpty()) return;

	if (filename.endsWith(".pcd", Qt::CaseInsensitive))
	{

		if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename.toStdString(), *cloudxyz) == -1) //* load the file
			ui.textEdit->append("Couldn't read file.pcd \n");

	}
	else if (filename.endsWith(".txt", Qt::CaseInsensitive))
	{
		if (cInspection->loadCloud_txt(filename.toStdString(), *cloudxyz) == 0) //* load the file
			ui.textEdit->append("Couldn't read file.txt \n");
	}
	else ui.textEdit->append("File must be .pcd or .txt");

	// If point cloud contains NaN values, remove them before updating the visualizer point cloud
	if (cloudxyz->size())
	{
		pcl::copyPointCloud(*cloudxyz, *cloud_);
		ui.textEdit->append("Download Points: " + QString::number(cloud_->size()));
		if(showPoints) Cloud_Display(*cloud_, "Cloud", 2, true, true, true);
	}
	else ui.textEdit->append("No point was loaded");
}

void threeD_Inspection::saveButtonPressed()
{
	QString filename = QFileDialog::getSaveFileName(this, tr("Save point cloud"), "", tr("Point cloud data (*.pcd *.ply *.txt)"));
	//QString filename = "D:/PhD-Research/1.pcd";

	if (filename.isEmpty())
		return;
	int return_status = 0;
	if (cloud_->size())
	{
		if (filename.endsWith(".pcd", Qt::CaseInsensitive))
			return_status = pcl::io::savePCDFileBinary(filename.toStdString(), *cloud_);
		else if (filename.endsWith(".txt", Qt::CaseInsensitive))
		{
			if (cInspection->saveCloud(filename.toStdString(), *cloud_) == 0)
				ui.textEdit->append("Cannot save file txt");
		}
		else ui.textEdit->append("Only can save file .pcd or .txt");

	}
	else ui.textEdit->append("Point Cloud is empty to be saved");
	
	if (return_status != 0) ui.textEdit->append("Error writing point cloud");

}

void threeD_Inspection::Reset_3D_ViewPressed()
{
	viewer_->removeAllPointClouds();
	viewer_->removeAllShapes();
}

void threeD_Inspection::Reload_CloudPressed()
{
	if (cloudxyz->size())
	{
		pcl::copyPointCloud(*cloudxyz, *cloud_);
		for (int i = 0; i < cloud_->size(); i++)
		{
			cloud_->points[i].r = 0; cloud_->points[i].g = 0; cloud_->points[i].b = 0;
		}
		ui.textEdit->append("Number of Points: " + QString::number(cloud_->size()));
		Cloud_Display(*cloud_, "Cloud", 2, true, true, true);
	}
	else ui.textEdit->append("Point Cloud is empty");
}

void threeD_Inspection::delay(int ms)
{
	// delay n (seconds)
	QTime dieTime = QTime::currentTime().addMSecs(ms);
	while (QTime::currentTime() < dieTime)
		QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

void threeD_Inspection::OBB_Draw(const Cloud &input, QString name, bool remov)
{
	if (remov) viewer_->removeAllShapes();
	name.append("_1");
	viewer_->addLine<pcl::PointXYZRGB>(input.points[0], input.points[1], 255, 0, 0, name.toUtf8().constData());
	name.append("_2");
	viewer_->addLine<pcl::PointXYZRGB>(input.points[0], input.points[2], 0, 255, 0, name.toUtf8().constData());
	name.append("_3");
	viewer_->addLine<pcl::PointXYZRGB>(input.points[0], input.points[3], 0, 0, 255, name.toUtf8().constData());
	name.append("_4");
	viewer_->addLine<pcl::PointXYZRGB>(input.points[1], input.points[6], 255, 255, 0, name.toUtf8().constData());
	name.append("_5");
	viewer_->addLine<pcl::PointXYZRGB>(input.points[1], input.points[7], 255, 255, 0, name.toUtf8().constData());
	name.append("_6");
	viewer_->addLine<pcl::PointXYZRGB>(input.points[2], input.points[5], 255, 255, 0, name.toUtf8().constData());
	name.append("_7");
	viewer_->addLine<pcl::PointXYZRGB>(input.points[2], input.points[7], 255, 255, 0, name.toUtf8().constData());
	name.append("_8");
	viewer_->addLine<pcl::PointXYZRGB>(input.points[3], input.points[5], 255, 255, 0, name.toUtf8().constData());
	name.append("_9");
	viewer_->addLine<pcl::PointXYZRGB>(input.points[3], input.points[6], 255, 255, 0, name.toUtf8().constData());
	name.append("_10");
	viewer_->addLine<pcl::PointXYZRGB>(input.points[4], input.points[5], 255, 255, 0, name.toUtf8().constData());
	name.append("_11");
	viewer_->addLine<pcl::PointXYZRGB>(input.points[4], input.points[6], 255, 255, 0, name.toUtf8().constData());
	name.append("_12");
	viewer_->addLine<pcl::PointXYZRGB>(input.points[4], input.points[7], 255, 255, 0, name.toUtf8().constData());
	viewer_->resetCamera();
	ui.qvtkWidget->update();
}

void threeD_Inspection::Coordinate_Draw(bool remov)
{
	if (remov) viewer_->removeAllShapes();
	pcl::PointXYZRGB point_0, point_1, point_2, point_3;
	point_0.x = 0;
	point_0.y = 0;
	point_0.z = 0;
	point_1.x = 20;
	point_1.y = 0;
	point_1.z = 0;
	point_2.x = 0;
	point_2.y = 20;
	point_2.z = 0;
	point_3.x = 0;
	point_3.y = 0;
	point_3.z = 20;
	viewer_->addLine<pcl::PointXYZRGB>(point_0, point_1, 255, 0, 0, "Ox");
	viewer_->addLine<pcl::PointXYZRGB>(point_0, point_2, 0, 255, 0, "Oy");
	viewer_->addLine<pcl::PointXYZRGB>(point_0, point_3, 0, 0, 255, "Oz");
	viewer_->resetCamera();
	ui.qvtkWidget->update();
}

void threeD_Inspection::Cloud_Display(const Cloud &input, QString name, int pointSize, bool removCloud, bool removeShape, bool resetCam)
{
	if (input.size() == 0)
	{
		ui.textEdit->append("Point Cloud is empty");
		return;
	}

	pcl::PointCloud<PointT>::Ptr inputShow(new pcl::PointCloud<PointT>);
	pcl::copyPointCloud(input, *inputShow);

	if (removCloud) viewer_->removeAllPointClouds();
	if (removeShape) viewer_->removeAllShapes();
	if (!input.points[0].rgb) cInspection->colorCloudDistances(*inputShow);
	if (!viewer_->updatePointCloud(inputShow, name.toUtf8().constData()))
		viewer_->addPointCloud(inputShow, name.toUtf8().constData());
	viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, name.toUtf8().constData());
	if (resetCam) viewer_->resetCamera();
	ui.qvtkWidget->update();
}

void threeD_Inspection::Show_OBB()
{
	if (cloud_->size() == 0) return;
	cInspection->OBB_features(*cloud_, *features);
	ui.textEdit->append("OBB Max:" + QString::number(features->points[8].x));
	ui.textEdit->append("OBB Mid:" + QString::number(features->points[8].y));
	ui.textEdit->append("OBB Min:" + QString::number(features->points[8].z));
	OBB_Draw(*features, QString::number(1), true);
}

void threeD_Inspection::Reset_Project()
{
	ui.textEdit->clear();

	cloud_.reset(new PointCloudT);
	source_.reset(new PointCloudT);
	target_.reset(new PointCloudT);
	segmented_cloud.reset(new PointCloudT);
	Segmented_Indicates.reset(new PointCloudT);
	features.reset(new PointCloudT);
	center_features.reset(new PointCloudT);
	recognised_cloud.reset(new PointCloudT);
	model_cloud.reset(new PointCloudT);
	frameTrans_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	LoadframeTrans_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	cloudxyz.reset(new pcl::PointCloud<pcl::PointXYZ>);

	viewIndex = 0;
	rotaPos = 0;
	scanMethod = 0;
	robotScan = false;
	frameScan = false;
	scanMethod = 0;
	robotMove = false;
	robotPause = false;
	writeToProbe = false;
	moveOrigin = false;
	showPoints = true;
	RegisMatr = Eigen::Matrix4f::Identity();

	viewer_->removeAllPointClouds();
	viewer_->removeAllShapes();
	ui.qvtkWidget->update();
}

void threeD_Inspection::Licensing()
{
	QString currentTime = QDateTime::currentDateTime().toString("dd.MM.yyyy");;
	/*ui.textEdit->append(currentTime);
	ui.textEdit->append(currentTime.section(".", 0, 0));
	ui.textEdit->append(currentTime.section(".", 1, 1));
	ui.textEdit->append(currentTime.section(".", 2, 2));*/

	int date = currentTime.section(".", 0, 0).toInt();
	int month = currentTime.section(".", 1, 1).toInt();
	int year = currentTime.section(".", 2, 2).toInt();

	QString filename = "CADdata\\keyCloud.pcd";

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Time(new pcl::PointCloud<pcl::PointXYZ>());
	/*cloud_Time->push_back(pcl::PointXYZ(22, 7, 2017));
	pcl::io::savePCDFileBinary(filename.toStdString(), *cloud_Time);
	cloud_Time->clear();*/
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename.toStdString(), *cloud_Time) == -1)
	{
		QMessageBox msgBox;
		msgBox.setText("Can not read file");
		msgBox.exec();
	}

	if (cloud_Time->points[0].x == 22 & cloud_Time->points[0].y == 7 & cloud_Time->points[0].z == 2017)
	if ((year - 2017) * 12 + (month - 7) < 6) // To 30/12/2017
		return;

	cloud_Time->clear();
	cloud_Time->push_back(pcl::PointXYZ(date, month, 2*year));
	pcl::io::savePCDFileBinary(filename.toStdString(), *cloud_Time);
	
	QMessageBox msgBox;
	msgBox.setText("This version was outdate, Please update new version!");
	msgBox.exec();

	exit(EXIT_FAILURE);
}

void threeD_Inspection::SaveSingleScan()
{
	if (source_->size())
	{
		int return_status = 0;
		string savFile = "SaveOnline\\SingleScan\\";
		savFile.append(QString::number(viewIndex).toUtf8().constData());
		savFile.append(".pcd");
		return_status = pcl::io::savePCDFileBinary(savFile, *source_);
		if (return_status != 0) ui.textEdit->append("Error writing point cloud");
	}
}

void threeD_Inspection::SaveRegisScan()
{
	if (cloud_->size())
	{
		int return_status = 0;
		string savFile = "SaveOnline\\RegisScan\\";
		savFile.append(QString::number(viewIndex).toUtf8().constData());
		savFile.append(".pcd");
		return_status = pcl::io::savePCDFileBinary(savFile, *cloud_);
		if (return_status != 0) ui.textEdit->append("Error writing point cloud");
	}
}

// -----------------------------------------------------0--------------------------------------------------------
// Recognition
void threeD_Inspection::DownsamplePressed()
{
	double ratio = ui.SpinBox_Downsample->text().toDouble();
	cInspection->DownSampling(*cloud_, *cloud_, ratio);
	
	ui.textEdit->append("Number of Points: " + QString::number(cloud_->size()));
	Cloud_Display(*cloud_, "Cloud", 2, true, true, true);
}

void threeD_Inspection::Upsampling()
{
	if (cloud_->size() == 0) return;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Buf(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*cloud_, *cloud_Buf);
	double upSamPara_01 = ui.doubleBox_Upsampling_01->text().toDouble(); 
	double upSamPara_02 = ui.doubleBox_Upsampling_02->text().toDouble();
	double upSamPara_03 = ui.doubleBox_Upsampling_03->text().toDouble();

	MovingLeastSquares<PointXYZ, PointXYZ> mls;
	mls.setInputCloud(cloud_Buf);
	mls.setSearchRadius(upSamPara_03);
	mls.setPolynomialFit(true);
	mls.setPolynomialOrder(upSamPara_03);
	mls.setUpsamplingMethod(MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
	mls.setUpsamplingRadius(upSamPara_02);
	mls.setUpsamplingStepSize(upSamPara_01);
	PointCloud<PointXYZ>::Ptr cloud_smoothed(new PointCloud<PointXYZ>());
	mls.process(*cloud_smoothed);

	pcl::copyPointCloud(*cloud_smoothed, *cloud_);
	ui.textEdit->append("Number of Points: " + QString::number(cloud_->size()));
	Cloud_Display(*cloud_, "Cloud", 2, true, true, true);
}

void threeD_Inspection::DenoisePressed()
{
	double StddevMulThresh = ui.doubleSpinBox_denoise->text().toDouble();
	cInspection->Denoise_Statistical(*cloud_, *cloud_, StddevMulThresh);
	
	ui.textEdit->append("Number of Points: " + QString::number(cloud_->size()));
	Cloud_Display(*cloud_, "Cloud", 2, true, true, true);
		
}

void threeD_Inspection::DenoiseByNormalsPressed()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr Surface_Cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_Cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud_, *Surface_Cloud);
	int level = ui.doubleBox_NormalLevel->text().toDouble();
	double scale = ui.doubleBox_NormalScale->text().toDouble();
	double thresh = ui.doubleBox_QualityThesh->text().toDouble();
	for (int i = 0; i < Surface_Cloud->size() / level; i++) input_Cloud->push_back(Surface_Cloud->points[i * level]);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(input_Cloud);
	ne.setSearchSurface(Surface_Cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(3);
	ne.compute(*cloud_normals);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr quali_Cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < cloud_normals->size(); i++)
	{
		pcl::PointXYZRGB point;
		point.x = cloud_->points[i].x;
		point.y = cloud_->points[i].y;
		point.z = cloud_->points[i].z;
		if (abs(cloud_normals->points[i].normal_z) > thresh)
		{
			//ui.textEdit->append("cloud_normals z: " + QString::number(abs(cloud_normals->points[i].normal_z)));
			quali_Cloud->push_back(point);
		}
	}
	pcl::copyPointCloud(*quali_Cloud, *cloud_);
	Cloud_Display(*cloud_, "cloud_", 2, true, true, true);
}

void threeD_Inspection::Denoise_by_segmentPressed()
{
	double tolerance = ui.doubleSpinBox_denoise_EcliSeg->text().toDouble();
	int minpoint = ui.spinBox_minP_Denoise->value();
	cInspection->Denoise_Segment(*cloud_, *cloud_, tolerance, minpoint);

	ui.textEdit->append("Number of Points: " + QString::number(cloud_->size()));
	if (showPoints) Cloud_Display(*cloud_, "Cloud", 2, true, true, true);
}

void threeD_Inspection::Euclidean_Cluster_ExtractionPressed()
{
	ui.textEdit->clear();
	// Euclidean Cluster Extraction
	if (segmented_cloud->size())segmented_cloud->clear();
	double tolerance = ui.SpinBox_Eucli_Seg->text().toDouble();
	int minpoint = ui.spinBox_minP_seg->value();

	if (cloud_->size()) cInspection->Euclidean_Cluster_Extraction(*cloud_, *segmented_cloud, *Segmented_Indicates, *center_features, tolerance, minpoint);
	else
	{
		ui.textEdit->append("Input cloud is empty for segmentation process");
		return;
	}
	for (int i = 0; i < (int)segmented_cloud->size(); i++)
	{
		segmented_cloud->points[i].r = Segmented_Indicates->points[i].r;
		segmented_cloud->points[i].g = Segmented_Indicates->points[i].g;
		segmented_cloud->points[i].b = Segmented_Indicates->points[i].b;
	}
	if (segmented_cloud->size())
	{
		ui.textEdit->append("Number of Points: " + QString::number(segmented_cloud->size()));
		ui.textEdit->append("Number of clusters: " + QString::number(Segmented_Indicates->points[Segmented_Indicates->size()-1].x + 1));
		Cloud_Display(*segmented_cloud, "Cloud_Segmented", 2, true, true, true);
	}
	else ui.textEdit->append("No cluseter is segmented");
	
}

void threeD_Inspection::Cylinder_FittingPressed()
{
	ui.textEdit->clear();
	double DistanceThreshold = ui.SpinBox_DistThreshSeg->text().toDouble();
	double RadiusMin = ui.SpinBox_minRadius->text().toDouble();
	double RadiusMax = ui.SpinBox_maxRadius->text().toDouble();
	int numOfpointThresh = ui.spinBox_Cylinder->text().toDouble();

	if (!segmented_cloud->size())
	{
		ui.textEdit->append("Segmented Cloud is empty");
		return;
	}

	if (center_features->size() != 0) center_features->clear();
	for (int k = 0; k <= Segmented_Indicates->points[Segmented_Indicates->size() - 1].x; k++)
	{
		pcl::PointCloud<PointT>::Ptr cylinders(new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr one_cylinder(new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr cylinder_Indicates(new pcl::PointCloud<PointT>);

		if (recognised_cloud->size()) recognised_cloud->clear();
		for (int i = 0; i < (int)segmented_cloud->size(); i++)
		{
			if (Segmented_Indicates->points[i].x == k)
			{
				pcl::PointXYZRGB point;
				point.x = segmented_cloud->points[i].x;
				point.y = segmented_cloud->points[i].y;
				point.z = segmented_cloud->points[i].z;
				recognised_cloud->push_back(point);
			}
		}
		if (recognised_cloud->size() == 0) continue;
		int numOfCylinder = 0;
		bool finish = false;
		while (!finish)
		{
			cInspection->Cylinder_Detection(*recognised_cloud, *one_cylinder, *recognised_cloud, DistanceThreshold, RadiusMin, RadiusMax);

			if (one_cylinder->size() > numOfpointThresh)
			{
				numOfCylinder++;
				for (int i = 0; i < (int)one_cylinder->size(); i++)
				{
					pcl::PointXYZRGB point;
					point.x = numOfCylinder;
					point.y = 0;
					point.z = 0;
					cylinder_Indicates->push_back(point);
				}
				cInspection->OBB_features(*one_cylinder, *features);
				cInspection->OBB_Inliers(*recognised_cloud, *features, *one_cylinder, *recognised_cloud);
				//cylinders->operator+=(*one_cylinder);

				/*ui.textEdit->append("OBB size X:" + QString::number(features->points[8].x));
				ui.textEdit->append("OBB size Y:" + QString::number(features->points[8].y));
				ui.textEdit->append("OBB size Z:" + QString::number(features->points[8].z));*/

				features->points[9].r = 255;
				features->points[9].g = 255;
				features->points[9].b = 255;
				center_features->push_back(features->points[9]);

				Cloud_Display(*center_features, "Center", 20, false, false, false);
				OBB_Draw(*features, QString::number(200 * k + numOfCylinder), false);
				if (!ui.checkBox_DetecAll->isChecked()) return;
			}
			else finish = true;
		}
	}

	if (!ui.checkBox_Host->isChecked())
	{
		ofstream saveFile;
		saveFile.open("C:\\Users\\Public\\pose.txt", ios::binary);
		if (!saveFile.is_open() || saveFile.fail())
		{
			ui.textEdit->append("Can not save pose");
			return;
		}
		saveFile << features->points[9].x << " " << features->points[9].y << " " << features->points[9].z << "\n";
		saveFile.close();
	}
}

void threeD_Inspection::OBB_Recognition()
{
	QElapsedTimer timer;
	
	if (segmented_cloud->size() == 0 || Segmented_Indicates->size() == 0)
	{
		ui.textEdit->append("Segmeted point cloud is empty");
		return;
	}
	
	for (int k = 0; k <= Segmented_Indicates->points[Segmented_Indicates->size() - 1].x; k++)
	{
		timer.start();
		if (recognised_cloud->size() != 0) recognised_cloud->clear();
		if (model_cloud->size() != 0) model_cloud->clear();
		for (int i = 0; i < (int)segmented_cloud->size(); i++)
		{
			if (Segmented_Indicates->points[i].x == k)
			{
				pcl::PointXYZRGB point;
				point.x = segmented_cloud->points[i].x;
				point.y = segmented_cloud->points[i].y;
				point.z = segmented_cloud->points[i].z;
				recognised_cloud->push_back(point);
			}
		}
		if (recognised_cloud->size() == 0) continue;

		double pass = ui.SpinBox_passRecog->text().toDouble();
		double OBB_toler = ui.doubleSpinBox_OBBTolerance->text().toDouble();
		cInspection->objectName = ui.comboBox_ObjRecog->currentText();
		unsigned char recog_status = cInspection->OBB_Recognition(*recognised_cloud, *model_cloud, OBB_toler, pass);
		if (recog_status == 1) ui.textEdit->append("OBB size not fitted");
		if (recog_status == 2) ui.textEdit->append("Matching score ICP is to low");
		if (recog_status == 3) ui.textEdit->append("Can not load data from virtual camera!");
		if (recog_status == 0)
		{
			ui.textEdit->append("OBB based Recognition Time: " + QString::number(timer.elapsed()) + " ms");
			QString model_str = "model_";
			model_str.append(QString::number(k));
			Cloud_Display(*model_cloud, model_str, 2, false, false, true);
			cInspection->OBB_features(*model_cloud, *features);
			QString obb_str = "obb_";
			obb_str.append(QString::number(k));
			OBB_Draw(*features, obb_str, false);
			//Coordinate_Draw(false);
			if (!ui.checkBox_DetecAll->isChecked())
			{
				if (segmented_cloud->size() != 0) segmented_cloud->clear();
				return;
			}
		}
	}
	if (segmented_cloud->size() != 0) segmented_cloud->clear();
}

void threeD_Inspection::SubOBB_Recognition()
{
	Cloud_Display(*cloud_, "Cloud", 2, true, true, true);
	if (ui.checkBox_nonSeg_Recognition->isChecked())
	{
		Non_Segmentation_SubOBB_Recognition();
		return;
	}
	QElapsedTimer timer;

	if (segmented_cloud->size() == 0 || Segmented_Indicates->size() == 0)
	{
		ui.textEdit->append("Segmeted point cloud is empty");
		return;
	}
	cInspection->desMethd[0] = 0; cInspection->desMethd[1] = 0; cInspection->desMethd[2] = 0;
	if (ui.checkBox_RecogFea_Area->isChecked()) cInspection->desMethd[2] = 1;
	if (ui.checkBox_RecogFea_Dis->isChecked()) cInspection->desMethd[1] = 1;
	if (ui.checkBox_RecogFea_Normal->isChecked()) cInspection->desMethd[0] = 1;

	for (int k = 0; k <= Segmented_Indicates->points[Segmented_Indicates->size() - 1].x; k++)
	{
		timer.start();
		if (recognised_cloud->size() != 0) recognised_cloud->clear();
		if (model_cloud->size() != 0) model_cloud->clear();
		for (int i = 0; i < (int)segmented_cloud->size(); i++)
		{
			if (Segmented_Indicates->points[i].x == k)
			{
				pcl::PointXYZRGB point;
				point.x = segmented_cloud->points[i].x;
				point.y = segmented_cloud->points[i].y;
				point.z = segmented_cloud->points[i].z;
				recognised_cloud->push_back(point);
			}
		}
		if (recognised_cloud->size() == 0) continue;
		
		double pass = ui.SpinBox_passRecog->text().toDouble();
		double OBB_tolerDown = ui.doubleSpinBox_SubOBBTolerance->text().toDouble();
		double OBB_tolerUp = ui.doubleSpinBox_SubOBBToler_Up->text().toDouble();
		double NCCThresh = ui.doubleSpinBox_Recog_NCCThresh->text().toDouble();
		cInspection->objectName = ui.comboBox_ObjRecog->currentText();
		unsigned char recog_status = 3;
		if (ui.checkBox_SubOBB_Recognition->isChecked()) recog_status = cInspection->SubOBB_Recognition(*recognised_cloud, *model_cloud, OBB_tolerDown, pass);
		if (ui.checkBox_OSubOBB_Recognition->isChecked()) recog_status = cInspection->Optimized_SubOBB_Recognition(*recognised_cloud, *model_cloud, NCCThresh, OBB_tolerUp, OBB_tolerDown, pass);
		if (recog_status == 1) ui.textEdit->append("OBB size not fitted");
		if (recog_status == 2) ui.textEdit->append("Matching score ICP is to low");
		if (recog_status == 3) ui.textEdit->append("Can not load data from virtual camera");
		if (recog_status == 0)
		{
			ui.textEdit->append("SubOBB based Recognition Time: " + QString::number(timer.elapsed()) + " ms");
			QString model_str = "model_";
			model_str.append(QString::number(k));
			
			for (int i = 0; i < (int)model_cloud->size(); i++)
			{
				model_cloud->points[i].r = 255;
				model_cloud->points[i].g = 255;
				model_cloud->points[i].b = 255;
			}
			Cloud_Display(*model_cloud, model_str, 2, false, false, true);
			cInspection->OBB_features(*model_cloud, *features);
			QString obb_str = "obb_";
			obb_str.append(QString::number(k));
			OBB_Draw(*features, obb_str, false);
			//Coordinate_Draw(false);
			if (!ui.checkBox_DetecAll->isChecked())
			{
				if (segmented_cloud->size() != 0) segmented_cloud->clear();
				return;
			}
		}
	}
	if (segmented_cloud->size() != 0) segmented_cloud->clear();
}

void threeD_Inspection::Non_Segmentation_SubOBB_Recognition()
{
	ui.textEdit->append("Non_Segmentation_SubOBB_Recognition");
	if (cloud_->size() == 0)
	{
		ui.textEdit->append("Point cloud is empty");
		return;
	}
	if (ui.checkBox_RecogFea_Area->isChecked()) cInspection->desMethd[2] = 1;
	if (ui.checkBox_RecogFea_Dis->isChecked()) cInspection->desMethd[1] = 1;
	if (ui.checkBox_RecogFea_Normal->isChecked()) cInspection->desMethd[0] = 1;

	QElapsedTimer timer;
	timer.start();
	double pass = ui.SpinBox_passRecog->text().toDouble();
	double OBB_toler = ui.doubleSpinBox_SubOBBTolerance->text().toDouble();
	cInspection->objectName = ui.comboBox_ObjRecog->currentText();

	if (model_cloud->size()) model_cloud->clear();
	unsigned char recog_status = cInspection->NonSeg_SubOBB_Recog(*cloud_, *model_cloud, pass);
	if (recog_status == 1) ui.textEdit->append("OBB size not fitted");
	if (recog_status == 2) ui.textEdit->append("Matching score ICP is to low");
	if (recog_status == 0)
	{
		ui.textEdit->append("Recognition Time: " + QString::number(timer.elapsed()) + " ms");
		QString model_str = "model_";
		//model_str.append(QString::number(k));
		Cloud_Display(*model_cloud, model_str, 2, true, false, true);
		cInspection->OBB_features(*model_cloud, *features);
		QString obb_str = "obb_";
		//obb_str.append(QString::number(k));
		OBB_Draw(*features, obb_str, false);
		Coordinate_Draw(false);
	}
}

void threeD_Inspection::RG_SubOBB_Recognition()
{
	if (cloud_->size() == 0) return;
	cInspection->objectName = ui.comboBox_ObjRecog->currentText();
	cInspection->reportCount = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SceneCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ObjectCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ObjectCloud_Buf(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ModelCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PickCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCLBestRota(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCLBestView(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCLBestNCC(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCLBestNumOfSubOBB(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr NCCLinePlot(new pcl::PointCloud<pcl::PointXYZ>);

	int NumOfSamplesX;
	int PickX, PickY;
	int numOfSubOBB_scene[3];
	double ZThresh;

	// Read Points from model and parameters
	cInspection->ReadPCLPointCloud(*ModelCloud);
	cInspection->TransferToOBB_CorrdinateXY(*cloud_, *SceneCloud);
	cInspection->ReaVirtualCameraParameters(NumOfSamplesX, ZThresh);
	cInspection->numOfSubOBB_Estimation(*SceneCloud, *ModelCloud, numOfSubOBB_scene);

	for (int i = 0; i < SceneCloud->size(); i++)
	{
		SceneCloud->points[i].r = 255;
		SceneCloud->points[i].g = 255;
		SceneCloud->points[i].b = 255;
	}
	Cloud_Display(*SceneCloud, "Cloud", 2, true, true, true);
	cInspection->RG_SubOBB_StartFind(*SceneCloud, *PickCloud, numOfSubOBB_scene);
	
	int PickX_Buf, PickY_Buf;
	int iter_buf = 0;
	for (int iter = 1; iter < PickCloud->size(); iter++)
	{
		if (NCCLinePlot->size()) NCCLinePlot->clear();
		plotter.clearPlots();
		Line_2D_Graph();

		double MaxDense = -1 * DBL_MAX;
		int MaxIndex;
		for (int i = 0; i < PickCloud->size(); i++)
		{
			if (MaxDense < PickCloud->points[i].z * PickCloud->points[i].g)
			{
				MaxDense = PickCloud->points[i].z * PickCloud->points[i].g;
				PickX = PickCloud->points[i].x;
				PickY = PickCloud->points[i].y;
				MaxIndex = i;
			}
		}
		PickCloud->points[MaxIndex].z = -1 * DBL_MAX;
		if (iter != 1)
		{
			double distPick = sqrt((PickX - PickX_Buf) * (PickX - PickX_Buf) + (PickY - PickY_Buf) * (PickY - PickY_Buf));
			ui.textEdit->append("distPick: " + QString::number(distPick));
			ui.textEdit->append("NumOfSubOBBY_model: " + QString::number(cInspection->NumOfSubOBBY_model));
			if (distPick <  cInspection->NumOfSubOBBY_model) continue;
		}
		iter_buf++;
		if (iter_buf > 2) iter = PickCloud->size();

		if (ObjectCloud->size()) ObjectCloud->clear();
		PickX_Buf = PickX;
		PickY_Buf = PickY;
		
		for (int grow_Buf = 0; grow_Buf < cInspection->NumOfSubOBBX_model / 2; grow_Buf++)
		{
			for (int growY = 0; growY <= grow_Buf; growY++)
			for (int growX = 0; growX <= grow_Buf; growX++)
			{
				ui.textEdit->append("growY: " + QString::number(growY));
				ui.textEdit->append("growX: " + QString::number(growX));

				if (growY < grow_Buf & growX < grow_Buf) continue;

				for (int k = -1; k < 2; k += 2)
				for (int l = -1; l < 2; l += 2)
				{
					if (k == 0 || l == 0) continue;
					PickX = PickX_Buf + k* growX;
					PickY = PickY_Buf + l* growY;
					if (PickX < 0 || PickY < 0 || PickX >= numOfSubOBB_scene[0] || PickY >= numOfSubOBB_scene[1]) continue;
					if (ObjectCloud_Buf->size()) ObjectCloud_Buf->clear();
					if (ObjectCloud->size()) pcl::copyPointCloud(*ObjectCloud, *ObjectCloud_Buf);
					cInspection->RG_SubOBB_ObjExtraction(*SceneCloud, *ObjectCloud_Buf, numOfSubOBB_scene, PickX, PickY);

					if (ObjectCloud_Buf->size() == 0) continue;
					if (ObjectCloud_Buf->size() == ObjectCloud->size() & ObjectCloud->size() > 0) continue;
					int status = cInspection->RG_SubOBB_Matching(*ObjectCloud_Buf, *ModelCloud, *PCLBestRota, *PCLBestView, *PCLBestNCC, 
						*PCLBestNumOfSubOBB, 0.2);
					
					double BestNCC = PCLBestNCC->points[PCLBestNCC->size() - 1].z;

					if (ObjectCloud_Buf->size())
					{
						if (BestNCC != -2) ui.textEdit->append("NCC: " + QString::number(BestNCC));
						NCCLinePlot->push_back(pcl::PointXYZ(0, 0, BestNCC));
						Update_Line_2D_Graph(*NCCLinePlot);

						for (int i = 0; i < ObjectCloud_Buf->size(); i++)
						{
							ObjectCloud_Buf->points[i].r = 255;
							ObjectCloud_Buf->points[i].g = 0;
							ObjectCloud_Buf->points[i].b = 0;
						}
						Cloud_Display(*ObjectCloud_Buf, "ObjectCloud", 2, false, false, false);
						delay(1000);
					}

					if (status == 1 || status == 2) continue;
					if (status == 0)
					{
						if (ObjectCloud->size()) ObjectCloud->clear();
						if (ObjectCloud_Buf->size()) pcl::copyPointCloud(*ObjectCloud_Buf, *ObjectCloud);
					}
				}
			}
		}
	}
	ui.textEdit->append("Completed!");
}

void threeD_Inspection::Recognition_Error_ReportPressed()
{
	if (recognised_cloud->size() == 0 || model_cloud->size() == 0) return;
	pcl::PointCloud<pcl::PointXYZ>::Ptr error_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cInspection->Distances_CloudToCloud_PCL(*recognised_cloud, *model_cloud, *error_cloud);

	double STD = 0;
	double mindist = error_cloud->points[error_cloud->size()  - 1].x;
	for (int i = 0; i < error_cloud->size() - 1; i++)
	{
		STD += (error_cloud->points[i].x - mindist) * (error_cloud->points[i].x - mindist);
	}
	STD = sqrt(STD) / sqrt(error_cloud->size() - 2);

	ui.textEdit->append("Mean Distance: " + QString::number(mindist));
	ui.textEdit->append("Standard deviation: " + QString::number(STD));

	cInspection->Display_Error_Report(*recognised_cloud, *error_cloud);
}

// -----------------------------------------------------0--------------------------------------------------------
// Virtial Camera
void threeD_Inspection::Initial_Transformation_VirtualCam()
{
	if (cloud_->size() == 0)
	{
		ui.textEdit->append("Point cloud is empty");
		return;
	}
	cInspection->OBB_Alignment(*cloud_, *cloud_);
	Cloud_Display(*cloud_, "Cloud", 2, true, true, true);
	cInspection->OBB_features(*cloud_, *features);
	OBB_Draw(*features, QString::number(1), true);
	Coordinate_Draw(false);
}

void threeD_Inspection::Generate_Database_VirtualCam()
{
	if (cloud_->size() == 0)
	{
		ui.textEdit->append("Point cloud is empty");
		return;
	}

	ofstream modFile;
	ofstream desFile;
	ofstream paraFile;
	QString Qname = ui.textEdit_VirCam_modelName->toPlainText();
	if (Qname == "")
	{
		ui.textEdit->append("No model name");
		return;
	}
	string name;
	name = "CADdata/";
	name.append(Qname.toUtf8().constData());
	name.append("_model.txt");
	modFile.open(name);
	name = "CADdata/";
	name.append(Qname.toUtf8().constData());
	name.append("_descriptor.txt");
	desFile.open(name);
	name = "CADdata/";
	name.append(Qname.toUtf8().constData());
	name.append("_parameters.txt");
	paraFile.open(name);

	double RxMin = ui.doubleSpinBox_VirCam_Rx_Min->text().toDouble();
	double RxMid = ui.doubleSpinBox_VirCam_Rx_Mid->text().toDouble();
	double RxIncre = ui.doubleSpinBox_VirCam_Rx_Incre->text().toDouble();
	double RyMin = ui.doubleSpinBox_VirCam_Ry_Min->text().toDouble();
	double RyMid = ui.doubleSpinBox_VirCam_Ry_Mid->text().toDouble();
	double RyIncre = ui.doubleSpinBox_VirCam_Ry_Incre->text().toDouble();
	double RzMin = ui.doubleSpinBox_VirCam_Rz_Min->text().toDouble();
	double RzMid = ui.doubleSpinBox_VirCam_Rz_Mid->text().toDouble();
	double RzIncre = ui.doubleSpinBox_VirCam_Rz_Incre->text().toDouble();
	double ZThresh = ui.doubleSpinBox_VirCam_Zthresh->text().toDouble();
	int NumOfSubOBBX_model = ui.doubleSpinBox_VirCam_Kx->text().toDouble();
	int NumOfSampleX = ui.doubleSpinBox_VirCam_NumOfSampleX->text().toDouble();
	double StddevMulThresh = ui.doubleSpinBox_VirCam_Noise->text().toDouble();

	double center[3];
	double corner_OBB[3], max_OBB[3], mid_OBB[3], min_OBB[3];
	cInspection->OBB_Calculation(*cloud_, corner_OBB, max_OBB, mid_OBB, min_OBB);
	double lengthOBB_model[3];
	lengthOBB_model[0] = sqrt(max_OBB[0] * max_OBB[0] + max_OBB[1] * max_OBB[1] + max_OBB[2] * max_OBB[2]); // length of max
	lengthOBB_model[1] = sqrt(mid_OBB[0] * mid_OBB[0] + mid_OBB[1] * mid_OBB[1] + mid_OBB[2] * mid_OBB[2]); // length of mid
	lengthOBB_model[2] = sqrt(min_OBB[0] * min_OBB[0] + min_OBB[1] * min_OBB[1] + min_OBB[2] * min_OBB[2]); // length of min
	int NumOfSubOBBY_model = NumOfSubOBBX_model * (double)lengthOBB_model[1] / lengthOBB_model[0] + 0.5;

	ui.textEdit->clear();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud_Virtual(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*cloud_, *Cloud_Virtual);

	for (int RotX = RxMin; RotX < RxMid; RotX += RxIncre)
	for (int RotY = RyMin; RotY < RyMid; RotY += RyIncre)
	for (int RotZ = RzMin; RotZ < RzMid; RotZ += RzIncre)
	{
		Eigen::Affine3f transformMatrix = Eigen::Affine3f::Identity();
		transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (RotX / (double)180), Eigen::Vector3f::UnitX()));
		transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (RotY / (double)180), Eigen::Vector3f::UnitY()));
		transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (RotZ / (double)180), Eigen::Vector3f::UnitZ()));
		desFile << RotX << " " << RotY << " " << RotZ << " ";
		pcl::transformPointCloud(*Cloud_Virtual, *cloud_, transformMatrix);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr VirtualPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ExtractedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr descriptor_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		cInspection->OBB_ExtractVitualPoints(*cloud_, *VirtualPoints, *ExtractedCloud, NumOfSampleX, ZThresh);
		cInspection->Denoise_Statistical(*ExtractedCloud, *ExtractedCloud, StddevMulThresh);
		Cloud_Display(*ExtractedCloud, "cloud_", 2, true, true, true);
		//pcl::copyPointCloud(*ExtractedCloud, *cloud_);

		cInspection->OBB_Calculation(*ExtractedCloud, corner_OBB, max_OBB, mid_OBB, min_OBB);
		double lengthOBB_ExtractedCloud[3];
		lengthOBB_ExtractedCloud[0] = sqrt(max_OBB[0] * max_OBB[0] + max_OBB[1] * max_OBB[1] + max_OBB[2] * max_OBB[2]); // length of max
		lengthOBB_ExtractedCloud[1] = sqrt(mid_OBB[0] * mid_OBB[0] + mid_OBB[1] * mid_OBB[1] + mid_OBB[2] * mid_OBB[2]); // length of mid
		lengthOBB_ExtractedCloud[2] = sqrt(min_OBB[0] * min_OBB[0] + min_OBB[1] * min_OBB[1] + min_OBB[2] * min_OBB[2]); // length of min
		desFile << lengthOBB_ExtractedCloud[0] << " " << lengthOBB_ExtractedCloud[1] << " " << 0 << " ";
		int NumOfSubOBBX = NumOfSubOBBX_model * (double)(lengthOBB_ExtractedCloud[0] / lengthOBB_model[0]) + 0.5;//0.5 means if 1.7->2, if 1.3->1
		int NumOfSubOBBY = NumOfSubOBBY_model * (double)(lengthOBB_ExtractedCloud[1] / lengthOBB_model[1]) + 0.5;//0.5 means if 1.7->2, if 1.3->1
		if (NumOfSubOBBX == 0 || NumOfSubOBBY == 0) return;

		cInspection->RealcoordinateToOBBcoordinateXY(*ExtractedCloud, *ExtractedCloud, corner_OBB, max_OBB, mid_OBB, min_OBB);
		transformMatrix = Eigen::Affine3f::Identity();
		Eigen::Vector4f center_Extracted;
		pcl::compute3DCentroid(*ExtractedCloud, center_Extracted);
		transformMatrix.translation() << -center_Extracted(0, 0), -center_Extracted(1, 0), -center_Extracted(2, 0);
		pcl::transformPointCloud(*ExtractedCloud, *ExtractedCloud, transformMatrix);

		double subobbAreaMax;
		cInspection->OBB_Descriptor(*ExtractedCloud, *descriptor_cloud, NumOfSubOBBX, NumOfSubOBBY, subobbAreaMax);

		desFile << NumOfSubOBBX << " " << NumOfSubOBBY << " " << subobbAreaMax << " ";

		double *array_X = new double[descriptor_cloud->size()];
		double *array_Y = new double[descriptor_cloud->size()];
		for (int i = 0; i < descriptor_cloud->size(); i++)
		{
			array_Y[i] = descriptor_cloud->points[i].z;
			array_X[i] = i;
			desFile << descriptor_cloud->points[i].x << " " << descriptor_cloud->points[i].y << " " << descriptor_cloud->points[i].z << " ";
		}
		desFile << "\n";
		plotter.clearPlots();
		plotter.setXRange(0, descriptor_cloud->size());
		plotter.addPlotData(array_X, array_Y, descriptor_cloud->size(), "", vtkChart::BAR);
		ui.qvtkWidget_graph->update();

		ui.textEdit->append("RotX: " + QString::number(RotX) + " RotY: " + QString::number(RotY) + " RotZ: " + QString::number(RotZ));
		delay(500);
	}

	for (int i = 0; i < Cloud_Virtual->size(); i++)
	{
		modFile << Cloud_Virtual->points[i].x << " " << Cloud_Virtual->points[i].y << " " << Cloud_Virtual->points[i].z << "\n";
	}
	paraFile << NumOfSampleX << "\n";
	paraFile << ZThresh << "\n";
	paraFile << StddevMulThresh << "\n";
	paraFile << NumOfSubOBBX_model << "\n";
	paraFile << NumOfSubOBBY_model << "\n";

	ui.textEdit->append("Completed!");
	modFile.close();
	desFile.close();
	paraFile.close();
	pcl::copyPointCloud(*Cloud_Virtual, *cloud_);
}

void threeD_Inspection::Generate_Opt_Database()
{
	if (cloud_->size() == 0)
	{
		ui.textEdit->append("Point cloud is empty");
		return;
	}

	cInspection->desMethd[0] = 0; cInspection->desMethd[1] = 0; cInspection->desMethd[2] = 0;
	if (ui.checkBox_VirCam_Area->isChecked()) cInspection->desMethd[2] = 1;
	if (ui.checkBox_VirCam_Dis->isChecked()) cInspection->desMethd[1] = 1;
	if (ui.checkBox_VirCam_Normal->isChecked()) cInspection->desMethd[0] = 1;

	ofstream paraFile;
	QString Qname = ui.textEdit_VirCam_modelName->toPlainText();
	if (Qname == "")
	{
		ui.textEdit->append("No model name");
		return;
	}
	string name;
	name = "CADdataOpt/";
	name.append(Qname.toUtf8().constData());
	name.append("_model.pcd");
	string name_model = name;
	name = "CADdataOpt/";
	name.append(Qname.toUtf8().constData());
	name.append("_descriptor.pcd");
	string name_descriptor = name;
	name = "CADdataOpt/";
	name.append(Qname.toUtf8().constData());
	name.append("_parameters.txt");
	paraFile.open(name);

	double RxMin = ui.doubleSpinBox_VirCam_Rx_Min->text().toDouble();
	double RxMid = ui.doubleSpinBox_VirCam_Rx_Mid->text().toDouble();
	double RxIncre = ui.doubleSpinBox_VirCam_Rx_Incre->text().toDouble();
	double RyMin = ui.doubleSpinBox_VirCam_Ry_Min->text().toDouble();
	double RyMid = ui.doubleSpinBox_VirCam_Ry_Mid->text().toDouble();
	double RyIncre = ui.doubleSpinBox_VirCam_Ry_Incre->text().toDouble();
	double RzMin = ui.doubleSpinBox_VirCam_Rz_Min->text().toDouble();
	double RzMid = ui.doubleSpinBox_VirCam_Rz_Mid->text().toDouble();
	double RzIncre = ui.doubleSpinBox_VirCam_Rz_Incre->text().toDouble();
	double ZThresh = ui.doubleSpinBox_VirCam_Zthresh->text().toDouble();
	int NumOfSubOBBX_model = ui.doubleSpinBox_VirCam_Kx->text().toDouble();
	int NumOfSampleX = ui.doubleSpinBox_VirCam_NumOfSampleX->text().toDouble();
	double StddevMulThresh = ui.doubleSpinBox_VirCam_Noise->text().toDouble();

	double corner_OBB[3], max_OBB[3], mid_OBB[3], min_OBB[3];
	cInspection->OBB_Calculation(*cloud_, corner_OBB, max_OBB, mid_OBB, min_OBB);
	double lengthOBB_model[3];
	lengthOBB_model[0] = sqrt(max_OBB[0] * max_OBB[0] + max_OBB[1] * max_OBB[1] + max_OBB[2] * max_OBB[2]); // length of max
	lengthOBB_model[1] = sqrt(mid_OBB[0] * mid_OBB[0] + mid_OBB[1] * mid_OBB[1] + mid_OBB[2] * mid_OBB[2]); // length of mid
	lengthOBB_model[2] = sqrt(min_OBB[0] * min_OBB[0] + min_OBB[1] * min_OBB[1] + min_OBB[2] * min_OBB[2]); // length of min
	int NumOfSubOBBY_model = NumOfSubOBBX_model * (double)lengthOBB_model[1] / lengthOBB_model[0] + 0.5;
	cInspection->SubOBB_Size = lengthOBB_model[0] / (double)NumOfSubOBBX_model;
	ui.textEdit->append("SubOBB_Size: " + QString::number(cInspection->SubOBB_Size));

	ui.textEdit->clear();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud_Virtual(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr descriptor_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*cloud_, *Cloud_Virtual);

	for (int RotX = RxMin; RotX < RxMid; RotX += RxIncre)
	for (int RotY = RyMin; RotY < RyMid; RotY += RyIncre)
	for (int RotZ = RzMin; RotZ < RzMid; RotZ += RzIncre)
	{
		Eigen::Affine3f transformMatrix = Eigen::Affine3f::Identity();
		transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (RotX / (double)180), Eigen::Vector3f::UnitX()));
		transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (RotY / (double)180), Eigen::Vector3f::UnitY()));
		transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (RotZ / (double)180), Eigen::Vector3f::UnitZ()));
		pcl::transformPointCloud(*Cloud_Virtual, *cloud_, transformMatrix);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr VirtualPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ExtractedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr descriptor_cloud_Buf(new pcl::PointCloud<pcl::PointXYZRGB>);
		cInspection->OBB_ExtractVitualPoints(*cloud_, *VirtualPoints, *ExtractedCloud, NumOfSampleX, ZThresh);
		cInspection->Denoise_Statistical(*ExtractedCloud, *ExtractedCloud, StddevMulThresh);
		Cloud_Display(*ExtractedCloud, "cloud_", 2, true, true, true);

		cInspection->OBB_Calculation(*ExtractedCloud, corner_OBB, max_OBB, mid_OBB, min_OBB);
		double lengthOBB_ExtractedCloud[3];
		lengthOBB_ExtractedCloud[0] = sqrt(max_OBB[0] * max_OBB[0] + max_OBB[1] * max_OBB[1] + max_OBB[2] * max_OBB[2]); // length of max
		lengthOBB_ExtractedCloud[1] = sqrt(mid_OBB[0] * mid_OBB[0] + mid_OBB[1] * mid_OBB[1] + mid_OBB[2] * mid_OBB[2]); // length of mid
		lengthOBB_ExtractedCloud[2] = sqrt(min_OBB[0] * min_OBB[0] + min_OBB[1] * min_OBB[1] + min_OBB[2] * min_OBB[2]); // length of min

		int NumOfSubOBB[3];
		NumOfSubOBB[0] = NumOfSubOBBX_model * (double)(lengthOBB_ExtractedCloud[0] / lengthOBB_model[0]) + 0.5;//0.5 means if 1.7->2, if 1.3->1
		NumOfSubOBB[1] = NumOfSubOBBY_model * (double)(lengthOBB_ExtractedCloud[1] / lengthOBB_model[1]) + 0.5;//0.5 means if 1.7->2, if 1.3->1
		if (NumOfSubOBB[0] == 0 || NumOfSubOBB[1] == 0) return;
		NumOfSubOBB[2] = 1;

		cInspection->TransferToOBB_CorrdinateXY(*ExtractedCloud, *ExtractedCloud);

		double ViewPoint[3];
		ViewPoint[0] = RotX; ViewPoint[1] = RotY; ViewPoint[2] = RotZ;
		cInspection->Nonseg_OBB_Descriptor(*ExtractedCloud, *descriptor_cloud_Buf, ViewPoint, 90);
		descriptor_cloud->operator+= (*descriptor_cloud_Buf);

		int arraySize = descriptor_cloud_Buf->points[3].x * descriptor_cloud_Buf->points[3].y;
		ui.textEdit->append("descriptor_cloud_Buf->points[3].x: " + QString::number(descriptor_cloud_Buf->points[3].x));
		ui.textEdit->append("descriptor_cloud_Buf->points[3].y: " + QString::number(descriptor_cloud_Buf->points[3].y));
		double *array_X = new double[arraySize];
		double *array_Y = new double[arraySize];
		for (int i = 0; i < arraySize; i++)
		{
			array_Y[i] = descriptor_cloud->points[i + 4].z;
			array_X[i] = i;
		}
		plotter.clearPlots();
		plotter.setXRange(0, arraySize);
		plotter.addPlotData(array_X, array_Y, arraySize, "", vtkChart::BAR);
		ui.qvtkWidget_graph->update();

		ui.textEdit->append("RotX: " + QString::number(RotX) + " RotY: " + QString::number(RotY) + " RotZ: " + QString::number(RotZ));
		delay(500);
	}

	int return_status = pcl::io::savePCDFileBinary(name_descriptor, *descriptor_cloud);
	return_status = pcl::io::savePCDFileBinary(name_model, *Cloud_Virtual);

	paraFile << NumOfSampleX << "\n";
	paraFile << ZThresh << "\n";
	paraFile << StddevMulThresh << "\n";
	paraFile << NumOfSubOBBX_model << "\n";
	paraFile << NumOfSubOBBY_model << "\n";
	paraFile << cInspection->SubOBB_Size << "\n";

	ui.textEdit->append("Completed!");
	paraFile.close();
	pcl::copyPointCloud(*Cloud_Virtual, *cloud_);
}

void threeD_Inspection::Gennerate_NonSeg_Database()
{
	if (cloud_->size() == 0)
	{
		ui.textEdit->append("Point cloud is empty");
		return;
	}

	ofstream paraFile;
	QString Qname = ui.textEdit_VirCam_modelName->toPlainText();
	if (Qname == "")
	{
		ui.textEdit->append("No model name");
		return;
	}
	string name;
	name = "CADdataNonseg/";
	name.append(Qname.toUtf8().constData());
	name.append("_model.pcd");
	string name_model = name;
	name = "CADdataNonseg/";
	name.append(Qname.toUtf8().constData());
	name.append("_descriptor.pcd");
	string name_descriptor = name;
	name = "CADdataNonseg/";
	name.append(Qname.toUtf8().constData());
	name.append("_parameters.txt");
	paraFile.open(name);

	double RxMin = ui.doubleSpinBox_VirCam_Rx_Min->text().toDouble();
	double RxMid = ui.doubleSpinBox_VirCam_Rx_Mid->text().toDouble();
	double RxIncre = ui.doubleSpinBox_VirCam_Rx_Incre->text().toDouble();
	double RyMin = ui.doubleSpinBox_VirCam_Ry_Min->text().toDouble();
	double RyMid = ui.doubleSpinBox_VirCam_Ry_Mid->text().toDouble();
	double RyIncre = ui.doubleSpinBox_VirCam_Ry_Incre->text().toDouble();
	double RzMin = ui.doubleSpinBox_VirCam_Rz_Min->text().toDouble();
	double RzMid = ui.doubleSpinBox_VirCam_Rz_Mid->text().toDouble();
	double RzIncre = ui.doubleSpinBox_VirCam_Rz_Incre->text().toDouble();
	double ZThresh = ui.doubleSpinBox_VirCam_Zthresh->text().toDouble();
	int NumOfSubOBBX_model = ui.doubleSpinBox_VirCam_Kx->text().toDouble();
	int NumOfSampleX = ui.doubleSpinBox_VirCam_NumOfSampleX->text().toDouble();
	double StddevMulThresh = ui.doubleSpinBox_VirCam_Noise->text().toDouble();

	double corner_OBB[3], max_OBB[3], mid_OBB[3], min_OBB[3];
	cInspection->OBB_Calculation(*cloud_, corner_OBB, max_OBB, mid_OBB, min_OBB);
	double lengthOBB_model[3];
	lengthOBB_model[0] = sqrt(max_OBB[0] * max_OBB[0] + max_OBB[1] * max_OBB[1] + max_OBB[2] * max_OBB[2]); // length of max
	lengthOBB_model[1] = sqrt(mid_OBB[0] * mid_OBB[0] + mid_OBB[1] * mid_OBB[1] + mid_OBB[2] * mid_OBB[2]); // length of mid
	lengthOBB_model[2] = sqrt(min_OBB[0] * min_OBB[0] + min_OBB[1] * min_OBB[1] + min_OBB[2] * min_OBB[2]); // length of min
	int NumOfSubOBBY_model = NumOfSubOBBX_model * (double)lengthOBB_model[1] / lengthOBB_model[0] + 0.5;
	cInspection->SubOBB_Size = lengthOBB_model[0] / (double)NumOfSubOBBX_model;
	ui.textEdit->append("SubOBB_Size: " + QString::number(cInspection->SubOBB_Size));

	ui.textEdit->clear();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud_Virtual(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr descriptor_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*cloud_, *Cloud_Virtual);

	for (int RotX = RxMin; RotX < RxMid; RotX += RxIncre)
	for (int RotY = RyMin; RotY < RyMid; RotY += RyIncre)
	for (int RotZ = RzMin; RotZ < RzMid; RotZ += RzIncre)
	{
		Eigen::Affine3f transformMatrix = Eigen::Affine3f::Identity();
		transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (RotX / (double)180), Eigen::Vector3f::UnitX()));
		transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (RotY / (double)180), Eigen::Vector3f::UnitY()));
		transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (RotZ / (double)180), Eigen::Vector3f::UnitZ()));
		pcl::transformPointCloud(*Cloud_Virtual, *cloud_, transformMatrix);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr VirtualPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ExtractedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr descriptor_cloud_Buf(new pcl::PointCloud<pcl::PointXYZRGB>);
		cInspection->OBB_ExtractVitualPoints(*cloud_, *VirtualPoints, *ExtractedCloud, NumOfSampleX, ZThresh);
		cInspection->Denoise_Statistical(*ExtractedCloud, *ExtractedCloud, StddevMulThresh);
		Cloud_Display(*ExtractedCloud, "cloud_", 2, true, true, true);

		cInspection->OBB_Calculation(*ExtractedCloud, corner_OBB, max_OBB, mid_OBB, min_OBB);
		double lengthOBB_ExtractedCloud[3];
		lengthOBB_ExtractedCloud[0] = sqrt(max_OBB[0] * max_OBB[0] + max_OBB[1] * max_OBB[1] + max_OBB[2] * max_OBB[2]); // length of max
		lengthOBB_ExtractedCloud[1] = sqrt(mid_OBB[0] * mid_OBB[0] + mid_OBB[1] * mid_OBB[1] + mid_OBB[2] * mid_OBB[2]); // length of mid
		lengthOBB_ExtractedCloud[2] = sqrt(min_OBB[0] * min_OBB[0] + min_OBB[1] * min_OBB[1] + min_OBB[2] * min_OBB[2]); // length of min

		int NumOfSubOBB[3];
		NumOfSubOBB[0] = NumOfSubOBBX_model * (double)(lengthOBB_ExtractedCloud[0] / lengthOBB_model[0]) + 0.5;//0.5 means if 1.7->2, if 1.3->1
		NumOfSubOBB[1] = NumOfSubOBBY_model * (double)(lengthOBB_ExtractedCloud[1] / lengthOBB_model[1]) + 0.5;//0.5 means if 1.7->2, if 1.3->1
		if (NumOfSubOBB[0] == 0 || NumOfSubOBB[1] == 0) return;
		NumOfSubOBB[2] = 1;

		cInspection->TransferToOBB_CorrdinateXY(*ExtractedCloud, *ExtractedCloud);

		double ViewPoint[3];
		ViewPoint[0] = RotX; ViewPoint[1] = RotY; ViewPoint[2] = RotZ;
		cInspection->Nonseg_OBB_Descriptor(*ExtractedCloud, *descriptor_cloud_Buf, ViewPoint, 10);
		descriptor_cloud->operator+= (*descriptor_cloud_Buf);

		int arraySize = descriptor_cloud_Buf->points[3].x * descriptor_cloud_Buf->points[3].y;
		ui.textEdit->append("descriptor_cloud_Buf->points[3].x: " + QString::number(descriptor_cloud_Buf->points[3].x));
		ui.textEdit->append("descriptor_cloud_Buf->points[3].y: " + QString::number(descriptor_cloud_Buf->points[3].y));
		double *array_X = new double[arraySize];
		double *array_Y = new double[arraySize];
		for (int i = 0; i < arraySize; i++)
		{
			array_Y[i] = descriptor_cloud->points[i + 4].z;
			array_X[i] = i;
		}
		plotter.clearPlots();
		plotter.setXRange(0, arraySize);
		plotter.addPlotData(array_X, array_Y, arraySize, "", vtkChart::BAR);
		ui.qvtkWidget_graph->update();

		ui.textEdit->append("RotX: " + QString::number(RotX) + " RotY: " + QString::number(RotY) + " RotZ: " + QString::number(RotZ));
		delay(500);
	}

	int return_status = pcl::io::savePCDFileBinary(name_descriptor, *descriptor_cloud);
	return_status = pcl::io::savePCDFileBinary(name_model, *Cloud_Virtual);

	paraFile << NumOfSampleX << "\n";
	paraFile << ZThresh << "\n";
	paraFile << StddevMulThresh << "\n";
	paraFile << NumOfSubOBBX_model << "\n";
	paraFile << NumOfSubOBBY_model << "\n"; 
	paraFile << cInspection->SubOBB_Size << "\n";

	ui.textEdit->append("Completed!");
	paraFile.close();
	pcl::copyPointCloud(*Cloud_Virtual, *cloud_);
}

// -----------------------------------------------------0--------------------------------------------------------
// Registration
void threeD_Inspection::Targe_Cloud()
{
	if (cloud_->size() == 0)
	{
		ui.textEdit->append("Cloud empty!");
		return;
	}
	else
	{
		pcl::copyPointCloud(*cloud_, *target_);
		for (int i = 0; i < target_->size(); i++)
		{
			target_->points[i].r = 255;
			target_->points[i].g = 0;
			target_->points[i].b = 0;
		}
		if(showPoints) Cloud_Display(*target_, "target", 2, true, true, true);
		ui.textEdit->append("OK! target available");
	}
}

void threeD_Inspection::Source_Cloud()
{
	if (cloud_->size() == 0)
	{
		ui.textEdit->append("Cloud empty!");
		return;
	}
	else
	{
		pcl::copyPointCloud(*cloud_, *source_);
		for (int i = 0; i < source_->size(); i++)
		{
			source_->points[i].r = 0;
			source_->points[i].g = 255;
			source_->points[i].b = 0;
		}
		if (showPoints == true) Cloud_Display(*source_, "source", 2, false, true, true);
		ui.textEdit->append("OK! source available");
	}
}

void threeD_Inspection::OBB_Matching()
{
	if (source_->size() == 0 || target_->size() == 0)
	{
		ui.textEdit->append("source or target is empty!");
		return;
	}
	cInspection->OBB_Alignment(*source_, *source_);
	Cloud_Display(*source_, "source", 2, true, true, true);
	cInspection->OBB_Alignment(*target_, *target_);
	Cloud_Display(*target_, "target", 2, false, false, false);
	ui.textEdit->append("OBB Matching finish!");
}

void threeD_Inspection::Regis_System_Positioning()
{
	double X = ui.doubleBox_RegisX->text().toDouble();
	double Y = ui.doubleBox_RegisY->text().toDouble();
	double Z = ui.doubleBox_RegisZ->text().toDouble();
	double RX = ui.doubleBox_RegisRX->text().toDouble();
	double RY = ui.doubleBox_RegisRY->text().toDouble();
	double RZ = ui.doubleBox_RegisRZ->text().toDouble();

	Eigen::Affine3f transformMatrix = Eigen::Affine3f::Identity();
	transformMatrix.translation() << X, Y, Z;
	transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (RX / (double)180), Eigen::Vector3f::UnitX()));
	transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (RY / (double)180), Eigen::Vector3f::UnitY()));
	transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (RZ / (double)180), Eigen::Vector3f::UnitZ()));
	pcl::transformPointCloud(*source_, *source_, transformMatrix);
	
	if(source_->size()) pcl::copyPointCloud(*source_, *cloud_);
	if(target_->size()) cloud_->operator+=(*target_);
	ui.textEdit->append("Number of Points: " + QString::number(cloud_->size()));
	Cloud_Display(*cloud_, "cloud_", 2, true, true, true);
}

void threeD_Inspection::AutoRegis_System_Positioning()
{
	double X = ui.doubleBox_RegisX->text().toDouble();
	double Y = ui.doubleBox_RegisY->text().toDouble();
	double Z = ui.doubleBox_RegisZ->text().toDouble();
	double RX = ui.doubleBox_RegisRX->text().toDouble();
	double RY = ui.doubleBox_RegisRY->text().toDouble();
	double RZ = ui.doubleBox_RegisRZ->text().toDouble();

	if (!ui.checkBox_Scan_Off->isChecked())
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr Pose(new pcl::PointCloud<pcl::PointXYZ>);
		cInspection->loadCloud_txt("ScanPose\\registrationPose.txt", *Pose);
		X = X * (Pose->points[viewIndex - 1].y - Pose->points[0].y);
		Y = Y * (Pose->points[viewIndex - 1].z - Pose->points[0].z);
		Z = Z * (Pose->points[viewIndex - 1].x - Pose->points[0].x);
		RX = 0; RY = 0; RZ = 0;
	}
	ui.textEdit->append("X: " + QString::number(X) + " Y: " + QString::number(Y) + " Z: " + QString::number(Z));
	ui.textEdit->append("RX: " + QString::number(RX) + " RX: " + QString::number(RX) + " RZ: " + QString::number(RZ));

	Eigen::Affine3f transformMatrix = Eigen::Affine3f::Identity();
	transformMatrix.translation() << X, Y, Z;
	transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (RX / (double)180), Eigen::Vector3f::UnitX()));
	transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (RY / (double)180), Eigen::Vector3f::UnitY()));
	transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (RZ / (double)180), Eigen::Vector3f::UnitZ()));
	pcl::transformPointCloud(*source_, *source_, transformMatrix);

	if (ui.checkBox_Scan_ModeDriven->isChecked() == 1 & ui.checkBox_Scan_Off->isChecked() == 0)
	{
		pcl::transformPointCloud(*source_, *source_, RegisMatr);
		ui.textEdit->append("RegisMatr[2][3]: " + QString::number(RegisMatr(2, 3)));
	}

	if (source_->size()) pcl::copyPointCloud(*source_, *cloud_);
	if (target_->size()) cloud_->operator+=(*target_);
	ui.textEdit->append("Number of Points: " + QString::number(cloud_->size()));
	if (showPoints) Cloud_Display(*cloud_, "cloud_", 2, true, true, true);
}

void threeD_Inspection::FrameBased_CoarseRegistration()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ModelCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr spheres_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	cInspection->objectName = "Spheres";
	cInspection->ReadPCLPointCloud(*ModelCloud);
	for (int i = 0; i < ModelCloud->size(); i++)
	{
		ModelCloud->points[i].r = 255;
		ModelCloud->points[i].g = 255;
		ModelCloud->points[i].b = 255;
	}

	if (ModelCloud->size() == 0)
	{
		ui.textEdit->append("Model Cloud is empty");
		return;
	}

	if (segmented_cloud->size() == 0)
	{
		ui.textEdit->append("Segmented Cloud is empty");
		return;
	}

	int minpoint = ui.spinBox_minP_Spheres->value();
	int maxpoint = ui.spinBox_maxP_Spheres->value();
	int minPDetected = ui.spinBox_minPDetected_Spheres->value();
	double threshDist = ui.SpinBox_Frame_DistThresh->text().toDouble();
	double radiusMin = ui.SpinBox_Frame_RadiusMin->text().toDouble();
	double radiusMax = ui.SpinBox_Frame_RadiusMax->text().toDouble();

	for (int k = 0; k <= Segmented_Indicates->points[Segmented_Indicates->size() - 1].x; k++)
	{
		if (recognised_cloud->size()) recognised_cloud->clear();
		for (int i = 0; i < (int)segmented_cloud->size(); i++)
		{
			if (Segmented_Indicates->points[i].x == k)
			{
				pcl::PointXYZRGB point;
				point.x = segmented_cloud->points[i].x;
				point.y = segmented_cloud->points[i].y;
				point.z = segmented_cloud->points[i].z;
				recognised_cloud->push_back(point);
			}
		}
		ui.textEdit->append("Number of Points: " + QString::number(recognised_cloud->size()));
		if (recognised_cloud->size() == 0 || recognised_cloud->size() > maxpoint || recognised_cloud->size() < minpoint) continue;

		double coff1[4], coff2[4];
		int SphereDetect = cInspection->Sphere_Detection(*recognised_cloud, threshDist, radiusMin, radiusMax, minPDetected, coff1, coff2);
		if (SphereDetect == 0) continue;
		pcl::PointXYZRGB point;
		point.x = coff1[0];
		point.y = coff1[1];
		point.z = coff1[2];
		/*ui.textEdit->append("X=" + QString::number(coff1[0]) + " Y=" + QString::number(coff1[1]) + " Z=" + QString::number(coff1[2]));
		ui.textEdit->append("Radius: " + QString::number(coff1[3]) + "\n");*/
		spheres_cloud->push_back(point);
		if (SphereDetect == 1) continue;
		point.x = coff2[0];
		point.y = coff2[1];
		point.z = coff2[2];
		/*ui.textEdit->append("X=" + QString::number(coff2[0]) + " Y=" + QString::number(coff2[1]) + " Z=" + QString::number(coff2[2]));
		ui.textEdit->append("Radius: " + QString::number(coff2[3]) + "\n");*/
		spheres_cloud->push_back(point);
	}

	ui.textEdit->append("Ball Detected: " + QString::number(spheres_cloud->size()));
	if (spheres_cloud->size() < 6) return;

	double matchingThresh = ui.SpinBox_Frame_MatchingThresh->text().toDouble();
	if (cInspection->Sphere_Marker_Matching(*spheres_cloud, *ModelCloud, *source_, *source_, matchingThresh))
	{
		ui.textEdit->append("Ball Matching score is too low!");
		return;
	}
	pcl::copyPointCloud(*source_, *cloud_);
	cloud_->operator+=(*target_);
	Cloud_Display(*cloud_, "cloud_", 2, true, true, true);
	Cloud_Display(*ModelCloud, "ModelCloud", 10, false, false, false);
	ui.textEdit->append("Ball Matching was successful!");
} 

void threeD_Inspection::AutoFrameBased_CoarseRegistration()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ModelCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr spheres_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	cInspection->objectName = "Spheres";
	cInspection->ReadPCLPointCloud(*ModelCloud);
	for (int i = 0; i < ModelCloud->size(); i++)
	{
		ModelCloud->points[i].r = 255;
		ModelCloud->points[i].g = 255;
		ModelCloud->points[i].b = 255;
	}

	if (ui.checkBox_FrameScan_ModeDriven->isChecked())
	{
		Eigen::Matrix4f FramMatr = Eigen::Matrix4f::Identity();
		for (int row = 0; row < 3; row++)
		{
			FramMatr(row, 0) = LoadframeTrans_cloud->points[(viewIndex - 1) * 4 + row].x;
			FramMatr(row, 1) = LoadframeTrans_cloud->points[(viewIndex - 1) * 4 + row].y;
			FramMatr(row, 2) = LoadframeTrans_cloud->points[(viewIndex - 1) * 4 + row].z;
		}
		FramMatr(0, 3) = LoadframeTrans_cloud->points[(viewIndex - 1) * 4 + 3].x;
		FramMatr(1, 3) = LoadframeTrans_cloud->points[(viewIndex - 1) * 4 + 3].y;
		FramMatr(2, 3) = LoadframeTrans_cloud->points[(viewIndex - 1) * 4 + 3].z;

		pcl::transformPointCloud(*source_, *source_, FramMatr);
		pcl::copyPointCloud(*source_, *cloud_);
		cloud_->operator+=(*target_);
		if (showPoints == true) Cloud_Display(*cloud_, "cloud_", 2, true, true, true);
		if (showPoints == true) Cloud_Display(*ModelCloud, "ModelCloud", 10, false, false, false);
		return;
	}

	if (!segmented_cloud->size())
	{
		ui.textEdit->append("Segmented Cloud is empty");
		return;
	}

	int minpoint = ui.spinBox_minP_Spheres->value();
	int maxpoint = ui.spinBox_maxP_Spheres->value();
	int minPDetected = ui.spinBox_minPDetected_Spheres->value();
	double threshDist = ui.SpinBox_Frame_DistThresh->text().toDouble();
	double radiusMin = ui.SpinBox_Frame_RadiusMin->text().toDouble();
	double radiusMax = ui.SpinBox_Frame_RadiusMax->text().toDouble();

	for (int k = 0; k <= Segmented_Indicates->points[Segmented_Indicates->size() - 1].x; k++)
	{
		if (recognised_cloud->size()) recognised_cloud->clear();
		for (int i = 0; i < (int)segmented_cloud->size(); i++)
		{
			if (Segmented_Indicates->points[i].x == k)
			{
				pcl::PointXYZRGB point;
				point.x = segmented_cloud->points[i].x;
				point.y = segmented_cloud->points[i].y;
				point.z = segmented_cloud->points[i].z;
				recognised_cloud->push_back(point);
			}
		}
		ui.textEdit->append("Number of Points: " + QString::number(recognised_cloud->size()));
		if (recognised_cloud->size() == 0 || recognised_cloud->size() > maxpoint || recognised_cloud->size() < minpoint) continue;

		double coff1[4], coff2[4];
		int SphereDetect = cInspection->Sphere_Detection(*recognised_cloud, threshDist, radiusMin, radiusMax, minPDetected, coff1, coff2);
		if (SphereDetect == 0) continue;
		pcl::PointXYZRGB point;
		point.x = coff1[0];
		point.y = coff1[1];
		point.z = coff1[2];
		/*ui.textEdit->append("X=" + QString::number(coff1[0]) + " Y=" + QString::number(coff1[1]) + " Z=" + QString::number(coff1[2]));
		ui.textEdit->append("Radius: " + QString::number(coff1[3]) + "\n");*/
		spheres_cloud->push_back(point);
		if (SphereDetect == 1) continue;
		point.x = coff2[0];
		point.y = coff2[1];
		point.z = coff2[2];
		/*ui.textEdit->append("X=" + QString::number(coff2[0]) + " Y=" + QString::number(coff2[1]) + " Z=" + QString::number(coff2[2]));
		ui.textEdit->append("Radius: " + QString::number(coff2[3]) + "\n");*/
		spheres_cloud->push_back(point);
	}

	ui.textEdit->append("Ball Detected: " + QString::number(spheres_cloud->size()));

	double matchingThresh = ui.SpinBox_Frame_MatchingThresh->text().toDouble();
	if (cInspection->Sphere_Marker_Matching(*spheres_cloud, *ModelCloud, *source_, *source_, matchingThresh))
	{
		ui.textEdit->append("Calibration supported!");
		Eigen::Matrix4f FramMatr = Eigen::Matrix4f::Identity();
		for (int row = 0; row < 3; row++)
		{
			FramMatr(row, 0) = LoadframeTrans_cloud->points[(viewIndex - 1) * 4 + row].x;
			FramMatr(row, 1) = LoadframeTrans_cloud->points[(viewIndex - 1) * 4 + row].y;
			FramMatr(row, 2) = LoadframeTrans_cloud->points[(viewIndex - 1) * 4 + row].z;
		}
		FramMatr(0, 3) = LoadframeTrans_cloud->points[(viewIndex - 1) * 4 + 3].x;
		FramMatr(1, 3) = LoadframeTrans_cloud->points[(viewIndex - 1) * 4 + 3].y;
		FramMatr(2, 3) = LoadframeTrans_cloud->points[(viewIndex - 1) * 4 + 3].z;
		pcl::transformPointCloud(*source_, *source_, FramMatr);
	}
	pcl::copyPointCloud(*source_, *cloud_);
	cloud_->operator+=(*target_);
	if (showPoints == true) Cloud_Display(*cloud_, "cloud_", 2, true, true, true);
	if (showPoints == true) Cloud_Display(*ModelCloud, "ModelCloud", 10, false, false, false);
	ui.textEdit->append("Ball Matching was successful!");
}

void threeD_Inspection::ICPPress()
{	
	if (source_->size() == 0 || target_->size() == 0)
	{
		ui.textEdit->append("source or target is empty!");
		return;
	}
	ui.textEdit->append("ICP target_: " + QString::number(target_->size()));
	ui.textEdit->append("ICP source_: " + QString::number(source_->size()));

	double toler = ui.doubleBox_Regis_toler->text().toDouble();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_overlap(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_overlap(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (cInspection->PCL_Overlap_Detection(*source_, *target_, *source_overlap, toler))
	{
		ui.textEdit->append("Overlap point cloud is empty!");
		return;
	}
	if (cInspection->PCL_Overlap_Detection(*target_, *source_, *target_overlap, toler))
	{
		ui.textEdit->append("Overlap point cloud is empty!");
		return;
	}

	if (source_overlap->size() < 50 || target_overlap->size() < 50)
	{
		ui.textEdit->append("Overlap area is too small!");
		return;
	}

	ui.textEdit->append("ICP target_overlap: " + QString::number(target_overlap->size()));
	ui.textEdit->append("ICP source_overlap: " + QString::number(source_overlap->size()));

	cInspection->ICP_Registration(*source_overlap, *target_overlap, *source_, *source_, 100);
	RegisMatr = cInspection->matr * RegisMatr;

	pcl::copyPointCloud(*source_, *cloud_);
	cloud_->operator+=(*target_);

	Cloud_Display(*cloud_, "cloud_", 2, true, true, true);
	ui.textEdit->append("ICP registration finish!");
}

void threeD_Inspection::Frame_ICPPress()
{
	if (source_->size() == 0 || target_->size() == 0)
	{
		ui.textEdit->append("source or target is empty!");
		return;
	}
	ui.textEdit->append("ICP target_: " + QString::number(target_->size()));
	ui.textEdit->append("ICP source_: " + QString::number(source_->size()));

	double toler = ui.doubleBox_Frame_Overlap->text().toDouble();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_overlap(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_overlap(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (cInspection->PCL_Overlap_Detection(*source_, *target_, *source_overlap, toler))
	{
		ui.textEdit->append("Overlap point cloud is empty!");
		return;
	}
	if (cInspection->PCL_Overlap_Detection(*target_, *source_, *target_overlap, toler))
	{
		ui.textEdit->append("Overlap point cloud is empty!");
		return;
	}

	if (source_overlap->size() < 50 || target_overlap->size() < 50)
	{
		ui.textEdit->append("Overlap area is too small!");
		return;
	}

	ui.textEdit->append("ICP target_overlap: " + QString::number(target_overlap->size()));
	ui.textEdit->append("ICP source_overlap: " + QString::number(source_overlap->size()));

	cInspection->ICP_Registration(*source_overlap, *target_overlap, *source_, *source_, 100);
	RegisMatr = cInspection->matr * RegisMatr;

	pcl::copyPointCloud(*source_, *cloud_);
	cloud_->operator+=(*target_);

	Cloud_Display(*cloud_, "cloud_", 2, true, true, true);
	ui.textEdit->append("ICP registration finish!");
}

void threeD_Inspection::ModDri_ICPPress()
{
	if (source_->size() == 0 || target_->size() == 0)
	{
		ui.textEdit->append("source or target is empty!");
		return;
	}
	ui.textEdit->append("ICP target_: " + QString::number(target_->size()));
	ui.textEdit->append("ICP source_: " + QString::number(source_->size()));

	double toler = ui.doubleBox_ModDri_Overlap->text().toDouble();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_overlap(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_overlap(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (cInspection->PCL_Overlap_Detection(*source_, *target_, *source_overlap, toler))
	{
		ui.textEdit->append("Overlap point cloud is empty!");
		return;
	}
	if (cInspection->PCL_Overlap_Detection(*target_, *source_, *target_overlap, toler))
	{
		ui.textEdit->append("Overlap point cloud is empty!");
		return;
	}

	if (source_overlap->size() < 50 || target_overlap->size() < 50)
	{
		ui.textEdit->append("Overlap area is too small!");
		return;
	}

	ui.textEdit->append("ICP target_overlap: " + QString::number(target_overlap->size()));
	ui.textEdit->append("ICP source_overlap: " + QString::number(source_overlap->size()));

	cInspection->ICP_Registration(*source_overlap, *target_overlap, *source_, *source_, 100);
	RegisMatr = cInspection->matr * RegisMatr;

	pcl::copyPointCloud(*source_, *cloud_);
	cloud_->operator+=(*target_);

	Cloud_Display(*cloud_, "cloud_", 2, true, true, true);
	ui.textEdit->append("ICP registration finish!");
}

void threeD_Inspection::Registration_Error_Report()
{
	if (source_->size() == 0 || target_->size() == 0) return;
	pcl::PointCloud<pcl::PointXYZ>::Ptr error_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cInspection->Distances_CloudToCloud_PCL(*source_, *target_, *error_cloud);
	cInspection->Display_Error_Report(*source_, *error_cloud);
}

void threeD_Inspection::Regis_Model_Driven()
{
	if (cloud_->size() == 0)
	{
		ui.textEdit->append("Point cloud is empty");
		return;
	}

	double pass = ui.SpinBox_passRecog->text().toDouble();
	double OBB_toler = ui.doubleSpinBox_SubOBBTolerance->text().toDouble();
	cInspection->objectName = ui.comboBox_ObjRecog->currentText();
	if (model_cloud->size()) model_cloud->size();
	cInspection->ReadPCLPointCloud(*model_cloud);
	unsigned char recog_status = cInspection->model_Driven_Registration(*cloud_, *cloud_, OBB_toler, pass);
	if (recog_status == 1) ui.textEdit->append("OBB size not fitted");
	if (recog_status == 2) ui.textEdit->append("Matching score ICP is to low");
	if (recog_status == 3) ui.textEdit->append("Cannot load data from virtual camera");
	if (recog_status == 0)
	{
		target_->operator+=(*cloud_);
		Cloud_Display(*cloud_, "cloud", 2, true, true, true);
		Cloud_Display(*model_cloud, "model", 2, true, false, false);
		Cloud_Display(*target_, "target", 2, false, false, false);
		//pcl::copyPointCloud(*target_, *cloud_);
		//Coordinate_Draw(false);
	}
}

void threeD_Inspection::Overlap_RemovalPress()
{
	if (cloud_->size() == 0) return;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Buf(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*cloud_, *cloud_Buf);
	double upSamPara_01 = ui.doubleBox_Upsampling_01->text().toDouble();
	double upSamPara_02 = ui.doubleBox_Upsampling_02->text().toDouble();

	MovingLeastSquares<PointXYZ, PointXYZ> mls;
	mls.setInputCloud(cloud_Buf);
	mls.setSearchRadius(2);
	mls.setPolynomialFit(true);
	mls.setPolynomialOrder(2);
	mls.setUpsamplingMethod(MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
	mls.setUpsamplingRadius(upSamPara_02);
	mls.setUpsamplingStepSize(upSamPara_01);
	PointCloud<PointXYZ>::Ptr cloud_smoothed(new PointCloud<PointXYZ>());
	mls.process(*cloud_smoothed);
	pcl::copyPointCloud(*cloud_smoothed, *cloud_);

	double ratio = ui.SpinBox_Downsample->text().toDouble();
	cInspection->DownSampling(*cloud_, *cloud_, ratio);

	ui.textEdit->append("Number of Points: " + QString::number(cloud_->size()));
	Cloud_Display(*cloud_, "Cloud", 2, true, true, true);
}

// -----------------------------------------------------0--------------------------------------------------------
// Test Mode
void threeD_Inspection::Pick_and_Place_TestPressed()
{
	//double robotTarget_X = ui.doubleSpinBox_PickX->text().toDouble();
	//double robotTarget_Y = ui.doubleSpinBox_PickY->text().toDouble();
	//double robotTarget_Z = ui.doubleSpinBox_PickZ->text().toDouble();
	//double robotTarget_Rx = ui.doubleSpinBox_PickRx->text().toDouble();
	//double robotTarget_Ry = ui.doubleSpinBox_PickRy->text().toDouble();
	//double robotTarget_Rz = ui.doubleSpinBox_PickRz->text().toDouble();

	//if (!ui.checkBox_Host->isChecked())
	//{
	//	ifstream fs;
	//	fs.open("//aidc-ntu3-pc/Users/Public/pointcloud.pcd", ios::binary);

	//	if (!fs.is_open() || fs.fail())
	//	{
	//		ui.textEdit->append("Can open file saved pose");
	//		return;
	//	}

	//	string line;
	//	vector<string> st;
	//	getline(fs, line);
	//	// Tokenize the line
	//	boost::trim(line);
	//	boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);
	//	if (st.size() == 3)
	//	{
	//		robotTarget_X += float(atof(st[0].c_str()));
	//		robotTarget_Y += float(atof(st[1].c_str()));
	//		robotTarget_Z += float(atof(st[2].c_str()));
	//	}

	//	fs.close();
	//}
	//else
	//{
	//	robotTarget_X += features->points[9].x;
	//	robotTarget_Y += features->points[9].y;
	//	robotTarget_Z += features->points[9].z;
	//}
	//
	//QString line = "X = \"" + QString::number(robotTarget_X, 'g', 6) + "\"" + " Y = \"" + QString::number(-5000.24512, 'g', 6) +
	//	"\"" + " Z = \"" + QString::number(robotTarget_Z, 'g', 6) + "\"" + " A = \"" + QString::number(robotTarget_Rx, 'g', 6) + "\"" +
	//	" B = \"" + QString::number(robotTarget_Ry, 'g', 6) + "\"" + " C = \"" + QString::number(robotTarget_Rz, 'g', 6) + "\"";
	//QString dataSend = ui.textEdit_Send_Kuka->toPlainText();
	//dataSend.replace(dataSend.indexOf("Current") + 8, dataSend.indexOf("Before") - dataSend.indexOf("Current") - 16, line);
	//ui.textEdit_Send_Kuka->clear();
	//ui.textEdit_Send_Kuka->append(dataSend);
}

void threeD_Inspection::FrameBall_DetectionPress()
{
	ui.textEdit->append("\n");

	if (!segmented_cloud->size())
	{
		ui.textEdit->append("Segmented Cloud is empty");
		return;
	}

	int minpoint = ui.spinBox_minP_Spheres->value();
	int maxpoint = ui.spinBox_maxP_Spheres->value();
	int minPDetected = ui.spinBox_minPDetected_Spheres->value();
	double threshDist = ui.SpinBox_Frame_DistThresh->text().toDouble();
	double radiusMin = ui.SpinBox_Frame_RadiusMin->text().toDouble();
	double radiusMax = ui.SpinBox_Frame_RadiusMax->text().toDouble();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr spheres_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	for (int k = 0; k <= Segmented_Indicates->points[Segmented_Indicates->size() - 1].x; k++)
	{
		if (recognised_cloud->size()) recognised_cloud->clear();
		for (int i = 0; i < (int)segmented_cloud->size(); i++)
		{
			if (Segmented_Indicates->points[i].x == k)
			{
				pcl::PointXYZRGB point;
				point.x = segmented_cloud->points[i].x;
				point.y = segmented_cloud->points[i].y;
				point.z = segmented_cloud->points[i].z;
				recognised_cloud->push_back(point);
			}
		}
		//ui.textEdit->append("Number of Points: " + QString::number(recognised_cloud->size()));
		if (recognised_cloud->size() == 0 || recognised_cloud->size() > maxpoint || recognised_cloud->size() < minpoint) continue;

		double coff1[4], coff2[4];
		int SphereDetect = cInspection->Sphere_Detection(*recognised_cloud, threshDist, radiusMin, radiusMax, minPDetected, coff1, coff2);
		if (SphereDetect == 0) continue;
		pcl::PointXYZRGB point;
		point.x = coff1[0];
		point.y = coff1[1];
		point.z = coff1[2];
		ui.textEdit->append("X=" + QString::number(coff1[0]) + " Y=" + QString::number(coff1[1]) + " Z=" + QString::number(coff1[2]));
		ui.textEdit->append("Radius: " + QString::number(coff1[3]) + "\n");
		spheres_cloud->push_back(point);
		if (SphereDetect == 1) continue;
		point.x = coff2[0];
		point.y = coff2[1];
		point.z = coff2[2];
		ui.textEdit->append("X=" + QString::number(coff2[0]) + " Y=" + QString::number(coff2[1]) + " Z=" + QString::number(coff2[2]));
		ui.textEdit->append("Radius: " + QString::number(coff2[3]) + "\n");
		spheres_cloud->push_back(point);
	}

	if (spheres_cloud->size() == 0)
	{
		ui.textEdit->append("No ball can be detected!");
		return;
	}
	ui.textEdit->append("Ball Detected: " + QString::number(spheres_cloud->size()));

	for (int i = 0; i < spheres_cloud->size(); i++)
	{
		spheres_cloud->points[i].r = 255;
		spheres_cloud->points[i].g = 255;
		spheres_cloud->points[i].b = 255;
	}
	Cloud_Display(*spheres_cloud, "spheres_cloud", 10, false, false, false);
}

void threeD_Inspection::Frame_Matching()
{
	ui.textEdit->append("\n");

	if (!segmented_cloud->size())
	{
		ui.textEdit->append("Segmented Cloud is empty");
		return;
	}

	int minpoint = ui.spinBox_minP_Spheres->value();
	int maxpoint = ui.spinBox_maxP_Spheres->value();
	int minPDetected = ui.spinBox_minPDetected_Spheres->value();
	double threshDist = ui.SpinBox_Frame_DistThresh->text().toDouble();
	double radiusMin = ui.SpinBox_Frame_RadiusMin->text().toDouble();
	double radiusMax = ui.SpinBox_Frame_RadiusMax->text().toDouble();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ModelCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr spheres_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	cInspection->objectName = "Spheres";
	cInspection->ReadPCLPointCloud(*ModelCloud);
	for (int i = 0; i < ModelCloud->size(); i++)
	{
		ModelCloud->points[i].r = 255;
		ModelCloud->points[i].g = 255;
		ModelCloud->points[i].b = 255;
	}

	for (int k = 0; k <= Segmented_Indicates->points[Segmented_Indicates->size() - 1].x; k++)
	{
		if (recognised_cloud->size()) recognised_cloud->clear();
		for (int i = 0; i < (int)segmented_cloud->size(); i++)
		{
			if (Segmented_Indicates->points[i].x == k)
			{
				pcl::PointXYZRGB point;
				point.x = segmented_cloud->points[i].x;
				point.y = segmented_cloud->points[i].y;
				point.z = segmented_cloud->points[i].z;
				recognised_cloud->push_back(point);
			}
		}
		if (recognised_cloud->size() == 0 || recognised_cloud->size() > maxpoint || recognised_cloud->size() < minpoint) continue;

		double coff1[4], coff2[4];
		int SphereDetect = cInspection->Sphere_Detection(*recognised_cloud, threshDist, radiusMin, radiusMax, minPDetected, coff1, coff2);
		if (SphereDetect == 0) continue;
		pcl::PointXYZRGB point;
		point.x = coff1[0];
		point.y = coff1[1];
		point.z = coff1[2];
		/*ui.textEdit->append("X=" + QString::number(coff1[0]) + " Y=" + QString::number(coff1[1]) + " Z=" + QString::number(coff1[2]));
		ui.textEdit->append("Radius: " + QString::number(coff1[3]) + "\n");*/
		spheres_cloud->push_back(point);
		if (SphereDetect == 1) continue;
		point.x = coff2[0];
		point.y = coff2[1];
		point.z = coff2[2];
		/*ui.textEdit->append("X=" + QString::number(coff2[0]) + " Y=" + QString::number(coff2[1]) + " Z=" + QString::number(coff2[2]));
		ui.textEdit->append("Radius: " + QString::number(coff2[3]) + "\n");*/
		spheres_cloud->push_back(point);
	}

	if (spheres_cloud->size() == 0)
	{
		ui.textEdit->append("No ball can be detected!");
		return;
	}
	ui.textEdit->append("Ball Detected: " + QString::number(spheres_cloud->size()));

	double matchingThresh = ui.SpinBox_Frame_MatchingThresh->text().toDouble();

	if (ui.checkBox_Frame_CaliFix->isChecked())
	{
		int modIndex = (viewIndex - 1) / 4;
		if (cInspection->Sphere_Marker_Matching_Fix(*spheres_cloud, *ModelCloud, *cloud_, *cloud_, modIndex, matchingThresh))
		{
			ui.textEdit->append("Matching score is too low!");
			return;
		}
	}
	else
	{
		if (cInspection->Sphere_Marker_Matching(*spheres_cloud, *ModelCloud, *cloud_, *cloud_, matchingThresh))
		{
			ui.textEdit->append("Matching score is too low!");
			return;
		}
	}
	
	for (int i = 0; i < spheres_cloud->size(); i++)
	{
		spheres_cloud->points[i].r = 255;
		spheres_cloud->points[i].g = 255;
		spheres_cloud->points[i].b = 255;
	}
	Cloud_Display(*cloud_, "cloud", 2, true, true, true);
	Cloud_Display(*ModelCloud, "ModelCloud", 10, false, false, false);
	ui.textEdit->append("Matching was successful!");
}

void threeD_Inspection::TestFrameCali_Start()
{
	Reset_Project();
	scanMethod = 3;
	frameScan = true;
	rotaTable_Move();
	ui.comboBox_FrameSize_Obj->setCurrentText("tall");
}

void threeD_Inspection::TestFrame_Pause()
{
	robotPause = true;
}

void threeD_Inspection::TestFrame_Continue()
{
	if (viewIndex > frameTrans_cloud->size() / 4)
	{
		for (int row = 0; row < 3; row++)
		{
			frameTrans_cloud->push_back(pcl::PointXYZ(cInspection->matr(row, 0), cInspection->matr(row, 1), cInspection->matr(row, 2)));
		}
		frameTrans_cloud->push_back(pcl::PointXYZ(cInspection->matr(0, 3), cInspection->matr(1, 3), cInspection->matr(2, 3)));
	}

	robotPause = false;
}

void threeD_Inspection::TestFrame_SaveTransf()
{
	ui.textEdit->append("frameTrans_cloud Saved: " + QString::number(frameTrans_cloud->size()));

	ofstream tallFile, medianFile, shortFile;
	tallFile.open("ScanPose\\tallFrameRegis.txt");
	medianFile.open("ScanPose\\medianFrameRegis.txt");
	shortFile.open("ScanPose\\shortFrameRegis.txt");

	for (int i = 0; i < frameTrans_cloud->size(); i++)
	{
		if (i != 0) tallFile << "\n";
		tallFile << frameTrans_cloud->points[i].x << " " << frameTrans_cloud->points[i].y << " " << frameTrans_cloud->points[i].z;
		int numView = i / 4;
		if ((numView + 1) % 4 == 1 || (numView + 1) % 4 == 2 || (numView + 1) % 4 == 3)
		{
			if (i != 0) medianFile << "\n";
			medianFile << frameTrans_cloud->points[i].x << " " << frameTrans_cloud->points[i].y << " " << frameTrans_cloud->points[i].z;
		}
		if ((numView + 1) % 4 == 1 || (numView + 1) % 4 == 2)
		{
			if (i != 0) shortFile << "\n";
			shortFile << frameTrans_cloud->points[i].x << " " << frameTrans_cloud->points[i].y << " " << frameTrans_cloud->points[i].z;
		}
	}
	shortFile.close();
	medianFile.close();
	tallFile.close();
}

void threeD_Inspection::OnlineFrameCalibration()
{
	ui.textEdit->append("\n");
	ui.textEdit->append("viewIndex: " + QString::number(viewIndex));
	// Dowload Cloud
	showPoints = false;
	DownloadPressed();
	showPoints = true;
	if (cloud_->size() == 0) TestFrame_Pause();

	// Denoise
	double denoisePara = ui.SpinBox_Frame_Denoise->text().toDouble();
	cInspection->Denoise_Statistical(*cloud_, *cloud_, denoisePara);
	ui.textEdit->append("After Denoise: " + QString::number(cloud_->size()));
	if (cloud_->size() == 0) TestFrame_Pause();

	// Segmentation
	if (segmented_cloud->size())segmented_cloud->clear();
	double tolerSeg = ui.SpinBox_FrameEucli_Seg->text().toDouble();
	int minPSeg = ui.spinBox_FrameminP_seg->value();
	if (cloud_->size()) cInspection->Euclidean_Cluster_Extraction(*cloud_, *segmented_cloud, *Segmented_Indicates, *center_features, tolerSeg, minPSeg);
	else { ui.textEdit->append("Input cloud is empty for segmentation process"); TestFrame_Pause(); return; }
	ui.textEdit->append("Number of clusters: " + QString::number(Segmented_Indicates->points[Segmented_Indicates->size() - 1].x + 1));

	if (!segmented_cloud->size())
	{
		ui.textEdit->append("Segmented Cloud is empty");
		TestFrame_Pause();
		return;
	}

	int minpoint = ui.spinBox_minP_Spheres->value();
	int maxpoint = ui.spinBox_maxP_Spheres->value();
	int minPDetected = ui.spinBox_minPDetected_Spheres->value();
	double threshDist = ui.SpinBox_Frame_DistThresh->text().toDouble();
	double radiusMin = ui.SpinBox_Frame_RadiusMin->text().toDouble();
	double radiusMax = ui.SpinBox_Frame_RadiusMax->text().toDouble();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ModelCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr spheres_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	cInspection->objectName = "Spheres";
	cInspection->ReadPCLPointCloud(*ModelCloud);
	for (int i = 0; i < ModelCloud->size(); i++)
	{
		ModelCloud->points[i].r = 255;
		ModelCloud->points[i].g = 255;
		ModelCloud->points[i].b = 255;
	}

	for (int k = 0; k <= Segmented_Indicates->points[Segmented_Indicates->size() - 1].x; k++)
	{
		if (recognised_cloud->size()) recognised_cloud->clear();
		for (int i = 0; i < (int)segmented_cloud->size(); i++)
		{
			if (Segmented_Indicates->points[i].x == k)
			{
				pcl::PointXYZRGB point;
				point.x = segmented_cloud->points[i].x;
				point.y = segmented_cloud->points[i].y;
				point.z = segmented_cloud->points[i].z;
				recognised_cloud->push_back(point);
			}
		}
		if (recognised_cloud->size() == 0 || recognised_cloud->size() > maxpoint || recognised_cloud->size() < minpoint) continue;

		double coff1[4], coff2[4];
		int SphereDetect = cInspection->Sphere_Detection(*recognised_cloud, threshDist, radiusMin, radiusMax, minPDetected, coff1, coff2);
		if (SphereDetect == 0) continue;
		pcl::PointXYZRGB point;
		point.x = coff1[0];
		point.y = coff1[1];
		point.z = coff1[2];
		//ui.textEdit->append("X=" + QString::number(coff1[0]) + " Y=" + QString::number(coff1[1]) + " Z=" + QString::number(coff1[2]));
		//ui.textEdit->append("Radius: " + QString::number(coff1[3]) + "\n");
		spheres_cloud->push_back(point);
		if (SphereDetect == 1) continue;
		point.x = coff2[0];
		point.y = coff2[1];
		point.z = coff2[2];
		/*ui.textEdit->append("X=" + QString::number(coff2[0]) + " Y=" + QString::number(coff2[1]) + " Z=" + QString::number(coff2[2]));
		ui.textEdit->append("Radius: " + QString::number(coff2[3]) + "\n");*/
		spheres_cloud->push_back(point);
	}

	ui.textEdit->append("Ball Detected: " + QString::number(spheres_cloud->size()));
	if (spheres_cloud->size() < 6)
	{
		ui.textEdit->append("Spheres less than 6, Stop!");
		TestFrame_Pause();
		return;
	}

	double matchingThresh = ui.SpinBox_Frame_MatchingThresh->text().toDouble();


	if (ui.checkBox_Frame_CaliFix->isChecked())
	{
		int modIndex = (viewIndex - 1) / 4;
		if (cInspection->Sphere_Marker_Matching_Fix(*spheres_cloud, *ModelCloud, *cloud_, *cloud_, modIndex, matchingThresh))
		{
			ui.textEdit->append("Matching score is too low!");
			TestFrame_Pause();
			return;
		}
	}
	else
	{
		if (cInspection->Sphere_Marker_Matching(*spheres_cloud, *ModelCloud, *cloud_, *cloud_, matchingThresh))
		{
			ui.textEdit->append("Matching score is too low!");
			TestFrame_Pause();
			return;
		}
	}

	Cloud_Display(*cloud_, "cloud_", 2, true, true, true);
	Cloud_Display(*ModelCloud, "ModelCloud", 10, false, false, false);
	for (int row = 0; row < 3; row++) frameTrans_cloud->push_back(pcl::PointXYZ(cInspection->matr(row, 0), cInspection->matr(row, 1), cInspection->matr(row, 2)));
	frameTrans_cloud->push_back(pcl::PointXYZ(cInspection->matr(0, 3), cInspection->matr(1, 3), cInspection->matr(2, 3)));
	cInspection->matr = Eigen::Matrix4f::Identity();
	ui.textEdit->append("Ball Matching was successful!");
}

// -----------------------------------------------------0--------------------------------------------------------
// Auto Mode

// Model Driven Scanning
void threeD_Inspection::AutoScanningPressed()
{
	Reset_Project();
	scanMethod = 1;
	viewIndex = 0;
	frameScan = false;
	robotScan = true;
	RegisMatr = Eigen::Matrix4f::Identity();
}

// Frame Based Scanning
void  threeD_Inspection::Frame_Based_Scan()
{
	Reset_Project();
	scanMethod = 2;
	frameScan = true;
	rotaTable_Move();

	QString regisFile;
	if (ui.comboBox_FrameSize_Obj->currentText() == "tall") regisFile = "ScanPose\\tallFrameRegis.txt";
	if (ui.comboBox_FrameSize_Obj->currentText() == "median") regisFile = "ScanPose\\medianFrameRegis.txt";
	if (ui.comboBox_FrameSize_Obj->currentText() == "short") regisFile = "ScanPose\\shortFrameRegis.txt";
	if (cInspection->loadCloud_txt(regisFile.toStdString(), *LoadframeTrans_cloud) == 0)
		ui.textEdit->append("Couldn't read file transform .txt file");
}

void threeD_Inspection::AutoMode_Pause()
{
	robotPause = true;
}

void threeD_Inspection::AutoMode_Continue()
{
	robotPause = false;
}

void threeD_Inspection::OnlineRobotPositiningRegitration()
{
	ui.textEdit->append("");
	ui.textEdit->append("Scan: " + QString::number(viewIndex));

	if (ui.checkBox_Scan_Off->isChecked())
	{
		cInspection->objectName = ui.comboBox_Scan_Obj->currentText();
		cInspection->ReadSCanPointCloud(*cloud_, viewIndex);
		delay(2000);
		Cloud_Display(*cloud_, "cloud_", 2, true, true, true);
		delay(1000);
	}
	else
	{
		showPoints = false;

		// Download
		DownloadPressed();
		if (cloud_->size() == 0) return;

		// Denoise
		double denoisePara = ui.SpinBox_ModDri_Denoise->text().toDouble();
		cInspection->Denoise_Statistical(*cloud_, *cloud_, denoisePara);
		ui.textEdit->append("After Denoise: " + QString::number(cloud_->size()));
		if (cloud_->size() == 0) return;

		// Denoise by Segmentation
		double tolerance = ui.SpinBox_ModDri_EucliSeg->text().toDouble();
		int minpoint = ui.spinBox_ModDri_minPSeg->value();
		cInspection->Denoise_Segment(*cloud_, *cloud_, tolerance, minpoint);

		Source_Cloud();
		if (!ui.checkBox_Scan_ICP->isChecked()) showPoints = true; AutoRegis_System_Positioning();
		if (ui.checkBox_Scan_ICP->isChecked())
		{
			if (viewIndex == 1)
			{
				if (ui.checkBox_ModDri_Save->isChecked()) { SaveSingleScan(); SaveRegisScan(); }
				Cloud_Display(*cloud_, "cloud_", 2, true, true, true);
				pcl::copyPointCloud(*cloud_, *cloudxyz);
			}
			else
			{
				ModDri_ICPPress();
				pcl::copyPointCloud(*source_, *cloud_); cloud_->operator+=(*target_);
				if (ui.checkBox_ModDri_Save->isChecked()) { SaveSingleScan(); SaveRegisScan(); }
				pcl::copyPointCloud(*cloud_, *cloudxyz);
			}
		}
		showPoints = false; Targe_Cloud(); showPoints = true;
	}
}

void threeD_Inspection::OnlineFrameBasedRegitration()
{
	ui.textEdit->append("");
	ui.textEdit->append("Scan: " + QString::number(viewIndex));

	showPoints = false;

	// Download point cloud
	DownloadPressed();
	if (cloud_->size() == 0) return;

	// Denoise
	double denoisePara = ui.SpinBox_Frame_Denoise->text().toDouble();
	cInspection->Denoise_Statistical(*cloud_, *cloud_, denoisePara);
	ui.textEdit->append("After Denoise: " + QString::number(cloud_->size()));
	if (cloud_->size() == 0) return;

	// Segmentation
	if (segmented_cloud->size())segmented_cloud->clear();
	double tolerSeg = ui.SpinBox_FrameEucli_Seg->text().toDouble();
	int minPSeg = ui.spinBox_FrameminP_seg->value();
	cInspection->Euclidean_Cluster_Extraction(*cloud_, *segmented_cloud, *Segmented_Indicates, *center_features, tolerSeg, minPSeg);
	ui.textEdit->append("Number of clusters: " + QString::number(Segmented_Indicates->points[Segmented_Indicates->size() - 1].x + 1));
	
	pcl::copyPointCloud(*segmented_cloud, *cloud_);
	Source_Cloud();

	if (!ui.checkBox_FrameScan_ICP->isChecked()) showPoints = true; AutoFrameBased_CoarseRegistration();

	// Denoise by Segmentation
	double tolerance = ui.SpinBox_FrameDenoise_Seg->text().toDouble();
	int minpoint = ui.spinBox_FrameDenoise_minPSeg->value();
	cInspection->Denoise_Segment(*source_, *source_, tolerance, minpoint);
	pcl::copyPointCloud(*source_, *cloud_); cloud_->operator+=(*target_);

	if (ui.checkBox_FrameScan_ICP->isChecked())
	{
		if (viewIndex == 1)
		{
			if (ui.checkBox_FrameScan_Save->isChecked()) { SaveSingleScan(); SaveRegisScan(); }
			Cloud_Display(*cloud_, "cloud_", 2, true, true, true);
			pcl::copyPointCloud(*cloud_, *cloudxyz);
		}
		else
		{
			Frame_ICPPress();
			pcl::copyPointCloud(*source_, *cloud_); cloud_->operator+=(*target_);
			if (ui.checkBox_FrameScan_Save->isChecked()) { SaveSingleScan(); SaveRegisScan(); }
			pcl::copyPointCloud(*cloud_, *cloudxyz);
		}
	}
	showPoints = false; Targe_Cloud(); showPoints = true;
}

// -----------------------------------------------------0--------------------------------------------------------
// Commnicate with Kuka Robot through TCP-IP
void threeD_Inspection::bt_ListenKukaButtonPressed()
{
	ui.textEdit_Recieved_Kuka->append("HostAddress: " + QString::number(59152));
	if (!server_Kuka->listen(QHostAddress::Any, 59152))
	{
		ui.textEdit_Recieved_Kuka->append("Server for Kuka Robot could not start!");
	}
	else
	{
		ui.textEdit_Recieved_Kuka->append("Server for Kuka Robot started!");

		kukaForm = ui.textEdit_Send_Kuka->toPlainText();
		QString moveStatus = "\"" + QString::number(0) + "\"";
		kukaForm.replace(kukaForm.indexOf("M=") + 2, 3, moveStatus);
		ui.textEdit_Send_Kuka->clear();
		ui.textEdit_Send_Kuka->append(kukaForm);
	}
}
 
void threeD_Inspection::writeData_Kuka()
{
	QString data = ui.textEdit_Send_Kuka->toPlainText();
	socket_Kuka->write(data.toUtf8().constData());
	socket_Kuka->waitForBytesWritten();
}

void threeD_Inspection::myConnection_Kuka()
{
	ui.textEdit_Recieved_Kuka->clear();
	ui.textEdit_Recieved_Kuka->append("New connection with Kuka start!");
	socket_Kuka = server_Kuka->nextPendingConnection();
	socket_Kuka->waitForReadyRead();
	// Read data
	connect(socket_Kuka, SIGNAL(readyRead()), SLOT(readData_Auto_Kuka()));
	QByteArray data = socket_Kuka->readAll();
	QString DataAsString(data);
	ui.textEdit_Recieved_Kuka->append("Data Received from Kuka:");
	ui.textEdit_Recieved_Kuka->append(DataAsString);
	//write data
	socket_Kuka->write(kukaForm.toUtf8().constData());
	socket_Kuka->waitForBytesWritten();
}

void threeD_Inspection::readData_Auto_Kuka()
{
	if (ui.comboBox_Kuka->currentText() == "Auto")
	{
		// Read data
		QByteArray data_receiv = socket_Kuka->readAll();
		QString DataAsString(data_receiv);
		ui.textEdit_Recieved_Kuka->clear();
		ui.textEdit_Recieved_Kuka->append("Data Received from Kuka:");
		ui.textEdit_Recieved_Kuka->append(DataAsString);
		// write data
		if ((robotScan == true || frameScan == true) & robotPause == false)
		{
			QString PoseFile;
			if (robotScan == true) PoseFile = "ScanPose\\robotPose.txt";
			else if (frameScan == true)
			{
				if (ui.comboBox_FrameSize_Obj->currentText() == "tall") PoseFile = "ScanPose\\tallFramePose.txt";
				if (ui.comboBox_FrameSize_Obj->currentText() == "median") PoseFile = "ScanPose\\medianFramePose.txt";
				if (ui.comboBox_FrameSize_Obj->currentText() == "short") PoseFile = "ScanPose\\shortFramePose.txt";
			}
			QFile inputFile(PoseFile);
			
			int count = 0;
			bool status = false;
			if (inputFile.open(QIODevice::ReadOnly))
			{
				ui.textEdit_Recieved_Kuka->append("inputFile");
				QTextStream in(&inputFile);
				while (!in.atEnd())
				{
					QString line = in.readLine();
					if (count == viewIndex)
					{
						QString dataSend = ui.textEdit_Send_Kuka->toPlainText();
						dataSend.replace(dataSend.indexOf("Current") + 8, dataSend.indexOf("Before") - dataSend.indexOf("Current") - 16, line);
						ui.textEdit_Send_Kuka->clear();
						ui.textEdit_Send_Kuka->append(dataSend);
						QString data = ui.textEdit_Send_Kuka->toPlainText();
						socket_Kuka->write(data.toUtf8().constData());
						socket_Kuka->waitForBytesWritten();
						status = true;
						dataToProbe = "capture";
					}
					count++;
				}
				if (status == false)
				{
					QString dataSend = ui.textEdit_Send_Kuka->toPlainText();
					socket_Kuka->write(dataSend.toUtf8().constData());
					socket_Kuka->waitForBytesWritten();
					status = true;
					dataToProbe = "stop";
					if (scanMethod == 3) TestFrame_SaveTransf();
				}
				inputFile.close();
				viewIndex++;
				robotScan = false;
				frameScan = false;
				writeToProbe = true;
			}
		}
		else
		{
			if (moveOrigin)
			{
				QFile inputFile("OriginKuka.txt");
				if (inputFile.open(QIODevice::ReadOnly))
				{
					QTextStream in(&inputFile);
					while (!in.atEnd())
					{
						QString line = in.readLine();
						QString dataSend = ui.textEdit_Send_Kuka->toPlainText();
						dataSend.replace(dataSend.indexOf("Current") + 8, dataSend.indexOf("Before") - dataSend.indexOf("Current") - 16, line);
						ui.textEdit_Send_Kuka->clear();
						ui.textEdit_Send_Kuka->append(dataSend);
						QString data = ui.textEdit_Send_Kuka->toPlainText();
						socket_Kuka->write(data.toUtf8().constData());
						socket_Kuka->waitForBytesWritten();
					}
					inputFile.close();
				}
				moveOrigin = false;
			}
			else if (robotMove)
			{
				QString dataSend = ui.textEdit_Send_Kuka->toPlainText();
				socket_Kuka->write(dataSend.toUtf8().constData());
				socket_Kuka->waitForBytesWritten();
				robotMove = false;
			}
			else
			{
				socket_Kuka->write(kukaForm.toUtf8().constData());
				socket_Kuka->waitForBytesWritten();
				if (writeToProbe == true)
				{
					socket_Probe->write(dataToProbe.toUtf8().constData());
					socket_Probe->waitForBytesWritten();
					writeToProbe = false;
				}
			}
		}

	}
}

void threeD_Inspection::readData_Kuka()
{
	if (ui.comboBox_Kuka->currentText() == "Manually")
	{
		QByteArray data = socket_Kuka->readAll();
		QString DataAsString(data);
		ui.textEdit_Recieved_Kuka->clear();
		ui.textEdit_Recieved_Kuka->append("Data Received from Kuka:");
		ui.textEdit_Recieved_Kuka->append(DataAsString);
	}
}

void threeD_Inspection::Origin_KukaPressed()
{
	moveOrigin = true;
}

void threeD_Inspection::Move_KukaPressed()
{
	robotMove = true;
}

// -----------------------------------------------------0--------------------------------------------------------
// Commnicate with Probe through TCP-IP
void threeD_Inspection::bt_ListenProbeButtonPressed()
{
	ui.textEdit_Recieved_Probe->append("HostAddress: " + QString::number(59150));
	if (!server_Probe->listen(QHostAddress::Any, 59150))
	{
		ui.textEdit_Recieved_Probe->append("Server for Probe could not start!");
	}
	else
	{
		ui.textEdit_Recieved_Probe->append("Server for Probe started!");
	}
}

void threeD_Inspection::myConnection_Probe()
{
	ui.textEdit_Recieved_Probe->append("New connection with Probe start!");
	socket_Probe = server_Probe->nextPendingConnection();
	socket_Probe->waitForReadyRead();
	// Read data
	connect(socket_Probe, SIGNAL(readyRead()), SLOT(readData_Auto_Probe()));
	QByteArray data = socket_Probe->readAll();
	QString DataAsString(data);
	ui.textEdit_Recieved_Probe->append(DataAsString);
}

void threeD_Inspection::readData_Auto_Probe()
{
	// Read data
	QByteArray data = socket_Probe->readAll();
	QString DataAsString(data);
	ui.textEdit_Recieved_Probe->append(DataAsString);
	if (data == "CaptureDone") 
	{
		if (scanMethod == 1) // Big Blade Scanning
		{
			OnlineRobotPositiningRegitration();
			robotScan = true;
		}
		else if (scanMethod == 2) // Frame based scanning
		{
			rotaTable_Move();
			OnlineFrameBasedRegitration();
			frameScan = true;
		}
		else if (scanMethod == 3) // Frame Calibration
		{
			rotaTable_Move();
			OnlineFrameCalibration();
			frameScan = true;
		}
	}
}

void threeD_Inspection::writeData_Probe()
{
	QString dataToProbe = "Robot Kuka reaches the target!";
	socket_Probe->write(dataToProbe.toUtf8().constData());
	socket_Probe->waitForBytesWritten();
}

// -----------------------------------------------------0--------------------------------------------------------
// Rotational Table
void threeD_Inspection::rotaTable_Connect()
{
	rotationalTable->Open();
	rotationalTable->Move("HSPD=10000");
}

void threeD_Inspection::rotaTable_Disconnect()
{
	fnPerformaxComClose(rotationalTable->m_hUSBDevice);
}

void threeD_Inspection::rotaTable_Move_60_Degree()
{
	rotaPos++; int anglePose = rotaPos * 60000;
	string anlge; anlge.append("X");
	anlge.append(QString::number(anglePose).toUtf8().constData());
	char *cstr = new char[anlge.length() + 1]; strcpy(cstr, anlge.c_str());

	rotationalTable->Move(cstr);
	delete[] cstr;

	ui.textEdit->append("Rotate table Position: " + QString::number(rotaPos));
	ui.textEdit->append("Rotate table Angle: " + QString::number(anglePose / 1000));
}

void threeD_Inspection::rotaTab_ZeroPos()
{
	rotationalTable->Move("X0");
	delay(3000);
	rotaPos = 0;

	ui.textEdit->append("Rotate table Position: " + QString::number(rotaPos));
	ui.textEdit->append("Rotate table Angle: " + QString::number(0));
}

void threeD_Inspection::rotaTable_Move()
{
	QString objectSize = ui.comboBox_FrameSize_Obj->currentText();
	if (objectSize == "tall")
	{
		if (viewIndex == 0) { rotationalTable->Move("X0"); delay(3000);}
		if (viewIndex == 4) { rotationalTable->Move("X60000"); delay(3000);}
		if (viewIndex == 8) { rotationalTable->Move("X120000"); delay(3000);}
		if (viewIndex == 12) { rotationalTable->Move("X180000"); delay(3000);}
		if (viewIndex == 16) { rotationalTable->Move("X240000"); delay(3000);}
		if (viewIndex == 20) { rotationalTable->Move("X300000"); delay(3000);}
		if (viewIndex == 24) { rotationalTable->Move("X0"); delay(3000);}
	}
	if (objectSize == "median")
	{
		if (viewIndex == 0) { rotationalTable->Move("X0"); delay(3000);}
		if (viewIndex == 3) { rotationalTable->Move("X60000"); delay(3000);}
		if (viewIndex == 6) { rotationalTable->Move("X120000"); delay(3000);}
		if (viewIndex == 9) { rotationalTable->Move("X180000"); delay(3000);}
		if (viewIndex == 12) { rotationalTable->Move("X240000"); delay(3000);}
		if (viewIndex == 15) { rotationalTable->Move("X300000"); delay(3000);}
		if (viewIndex == 18) { rotationalTable->Move("X0"); delay(3000);}
	}
	if (objectSize == "short")
	{
		if (viewIndex == 0) { rotationalTable->Move("X0"); delay(3000); }
		if (viewIndex == 2) { rotationalTable->Move("X60000"); delay(3000); }
		if (viewIndex == 4) { rotationalTable->Move("X120000"); delay(3000); }
		if (viewIndex == 6) { rotationalTable->Move("X180000"); delay(3000); }
		if (viewIndex == 8) { rotationalTable->Move("X240000"); delay(3000); }
		if (viewIndex == 10) { rotationalTable->Move("X300000"); delay(3000); }
		if (viewIndex == 12) { rotationalTable->Move("X0"); delay(3000); }
	}
}

// -----------------------------------------------------0--------------------------------------------------------
// Tools and Examples
void threeD_Inspection::ICP_ExamplePressed()
{
	// ICP Example
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr ICP_Source(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr ICP_Target(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr ICP_Align(new pcl::PointCloud<pcl::PointXYZ>());

	// Input Data
	for (int i = 0; i < 100; i++)
	{
		pcl::PointXYZ point;

		point.x = 1024 * rand() / (RAND_MAX + 1.0f);
		point.y = 1024 * rand() / (RAND_MAX + 1.0f);
		point.z = 1024 * rand() / (RAND_MAX + 1.0f);
		ICP_Source->push_back(point);

		point.x += point.x*0.02;
		point.y += point.y*0.03;
		point.z += point.z*0.04;
		ICP_Target->push_back(point);
	}
	ui.textEdit->append("ICP_Target->size(): " + QString::number(ICP_Target->size()));
	Eigen::Affine3f transformMatrix = Eigen::Affine3f::Identity();
	transformMatrix.translation() << 30, 50, 100;
	transformMatrix.rotate(Eigen::AngleAxisf(M_PI/3, Eigen::Vector3f::UnitX()));
	transformMatrix.rotate(Eigen::AngleAxisf(M_PI/4, Eigen::Vector3f::UnitY()));
	transformMatrix.rotate(Eigen::AngleAxisf(M_PI/5, Eigen::Vector3f::UnitZ()));
	pcl::transformPointCloud(*ICP_Source, *ICP_Source, transformMatrix);
	
	// ICP Algorithm
	PointCloudT::Ptr ICP_transformed_cloud(new PointCloudT);
	IterativeClosestPoint<PointXYZ, PointXYZ> icp;
	icp.setInputCloud(ICP_Source);
	icp.setInputTarget(ICP_Target);
	icp.setMaximumIterations(10);

	//QThread::msleep(1000);
	icp.align(*ICP_Align);
	//Eigen::Matrix4f transformation = icp.getFinalTransformation();
	//pcl::transformPointCloud(*ICP_Source, *ICP_Source, transformation);
	ui.textEdit->append("Has converged: " + QString::number(icp.hasConverged()));
	QElapsedTimer timer;
	timer.start();
	ui.textEdit->append("Matching Score: " + QString::number(icp.getFitnessScore()));
	ui.textEdit->append("ICP Time: " + QString::number(timer.elapsed()) + " ms");

	// Display
	PointCloudT::Ptr source_cloud(new PointCloudT);
	PointCloudT::Ptr target_cloud(new PointCloudT);
	PointCloudT::Ptr aligned_cloud(new PointCloudT);
	pcl::copyPointCloud(*ICP_Source, *source_cloud);
	pcl::copyPointCloud(*ICP_Target, *target_cloud);
	pcl::copyPointCloud(*ICP_Align, *aligned_cloud);
	for (int i = 0; i < source_cloud->size(); i++)
	{
		source_cloud->points[i].r = 255;
		source_cloud->points[i].g = 0;
		source_cloud->points[i].b = 0;
	}
	for (int i = 0; i < target_cloud->size(); i++)
	{
		target_cloud->points[i].r = 0;
		target_cloud->points[i].g = 255;
		target_cloud->points[i].b = 0;
	}
	for (int i = 0; i < aligned_cloud->size(); i++)
	{
		aligned_cloud->points[i].r = 0;
		aligned_cloud->points[i].g = 0;
		aligned_cloud->points[i].b = 255;
	}

	ui.textEdit->append("source_cloud: " + QString::number(source_cloud->size()));
	Cloud_Display(*source_cloud, "source_cloud", 10, true, true, true);
	ui.textEdit->append("target_cloud: " + QString::number(target_cloud->size()));
	Cloud_Display(*target_cloud, "target_cloud", 10, false, false, false);
	ui.textEdit->append("aligned_cloud: " + QString::number(aligned_cloud->size()));
	Cloud_Display(*aligned_cloud, "aligned_cloud", 10, false, false, false);
}

void threeD_Inspection::SVD_ExamplePressed()
{
	// SVD Registration Example - Only used when two point clouds have the same number of points

	pcl::PointCloud<pcl::PointXYZ>::Ptr SVD_Source(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr SVD_Target(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr SVD_Align(new pcl::PointCloud<pcl::PointXYZ>());

	// Input Data
	for (int i = 0; i < 10; i++)
	{
		pcl::PointXYZ point;

		point.x = 1024 * rand() / (RAND_MAX + 1.0f);
		point.y = 1024 * rand() / (RAND_MAX + 1.0f);
		point.z = 1024 * rand() / (RAND_MAX + 1.0f);
		SVD_Target->push_back(point);

		point.x += point.x*0.02;
		point.y += point.y*0.03;
		point.z += point.z*0.04;
		SVD_Source->push_back(point);
	}

	Eigen::Affine3f transformMatrix = Eigen::Affine3f::Identity();
	transformMatrix.translation() << 30, 50, 100;
	//transformMatrix.rotate(Eigen::AngleAxisf(M_PI / 3, Eigen::Vector3f::UnitX()));
	//transformMatrix.rotate(Eigen::AngleAxisf(M_PI / 4, Eigen::Vector3f::UnitY()));
	//transformMatrix.rotate(Eigen::AngleAxisf(M_PI / 5, Eigen::Vector3f::UnitZ()));
	pcl::transformPointCloud(*SVD_Source, *SVD_Source, transformMatrix);

	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> TESVD;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 SVD_transformation;
	TESVD.estimateRigidTransformation(*SVD_Source, *SVD_Target, SVD_transformation);

	pcl::transformPointCloud(*SVD_Source, *SVD_Align, SVD_transformation);
	ui.textEdit->append("SVD_transformation[3, 0]: " + QString::number(SVD_transformation(3, 0)));
	ui.textEdit->append("SVD_transformation[0, 3]: " + QString::number(SVD_transformation(0, 3)));
	// Display
	PointCloudT::Ptr source_cloud(new PointCloudT);
	PointCloudT::Ptr target_cloud(new PointCloudT);
	PointCloudT::Ptr aligned_cloud(new PointCloudT);
	pcl::copyPointCloud(*SVD_Source, *source_cloud);
	pcl::copyPointCloud(*SVD_Target, *target_cloud);
	pcl::copyPointCloud(*SVD_Align, *aligned_cloud);
	for (int i = 0; i < source_cloud->size(); i++)
	{
		source_cloud->points[i].r = 255;
		source_cloud->points[i].g = 0;
		source_cloud->points[i].b = 0;
	}
	for (int i = 0; i < target_cloud->size(); i++)
	{
		target_cloud->points[i].r = 0;
		target_cloud->points[i].g = 255;
		target_cloud->points[i].b = 0;
	}
	for (int i = 0; i < aligned_cloud->size(); i++)
	{
		aligned_cloud->points[i].r = 0;
		aligned_cloud->points[i].g = 0;
		aligned_cloud->points[i].b = 255;
	}

	ui.textEdit->append("source_cloud: " + QString::number(source_cloud->size()));
	Cloud_Display(*source_cloud, "source_cloud", 10, true, true, true);
	ui.textEdit->append("target_cloud: " + QString::number(target_cloud->size()));
	Cloud_Display(*target_cloud, "target_cloud", 10, false, false, false);
	ui.textEdit->append("aligned_cloud: " + QString::number(aligned_cloud->size()));
	Cloud_Display(*aligned_cloud, "aligned_cloud", 10, false, false, false);
}

void threeD_Inspection::ToolTestPress()
{
	/*pcl::PointCloud<pcl::PointXYZ> test_cloud;
	test_cloud.height = 2;
	test_cloud.width = 5;
	test_cloud.points.resize(test_cloud.width * test_cloud.height);
	ui.textEdit->append("test_cloud: " + QString::number(test_cloud.size()));
	ui.textEdit->append("width: " + QString::number(test_cloud.width)); 
	
	for (int i = 0; i < test_cloud.height; i++)
	for (int j = 0; j < test_cloud.width; j++)
	{
		test_cloud(j, i).x = 10+i;
		test_cloud(j, i).y = 10*j;
		test_cloud(j, i).z = i+j;
	}

	ui.textEdit->append("test_cloud[0][0].x: " + QString::number(test_cloud(0, 0).x));
	ui.textEdit->append("test_cloud[3][0].x: " + QString::number(test_cloud(3, 0).x));
	ui.textEdit->append("test_cloud[1][1].x: " + QString::number(test_cloud(1, 1).x));
	ui.textEdit->append("test_cloud[4][1].x: " + QString::number(test_cloud(4, 1).x));
	ui.textEdit->append("test_cloud[17][1].x: " + QString::number(test_cloud(17, 1).x));
	ui.textEdit->append("test_cloud[1][3].x: " + QString::number(test_cloud(1, 3).x));

	test_cloud.clear();
	ui.textEdit->append("test_cloud[0][0].x: " + QString::number(test_cloud(0, 0).x));
	ui.textEdit->append("test_cloud[3][0].x: " + QString::number(test_cloud(3, 0).x));
	ui.textEdit->append("test_cloud[1][1].x: " + QString::number(test_cloud(1, 1).x));
	ui.textEdit->append("test_cloud[4][1].x: " + QString::number(test_cloud(4, 1).x));
	ui.textEdit->append("test_cloud[17][1].x: " + QString::number(test_cloud(17, 1).x));
	ui.textEdit->append("test_cloud[1][3].x: " + QString::number(test_cloud(1, 3).x));*/	
}

void threeD_Inspection::Sphere_Detection()
{
	if (!segmented_cloud->size())
	{
		ui.textEdit->append("Segmented Cloud is empty");
		return;
	}

	int minpoint = ui.spinBox_minP_Spheres->value(); 
	int maxpoint = ui.spinBox_maxP_Spheres->value();
	int minPDetected = ui.spinBox_minPDetected_Spheres->value();
	double threshDist = ui.SpinBox_Frame_DistThresh->text().toDouble();
	double radiusMin = ui.SpinBox_Frame_RadiusMin->text().toDouble();
	double radiusMax = ui.SpinBox_Frame_RadiusMax->text().toDouble(); 

	ui.textEdit->append("threshDist: " + QString::number(threshDist));
	ui.textEdit->append("radiusMin: " + QString::number(radiusMin));
	ui.textEdit->append("radiusMax: " + QString::number(radiusMax));

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ModelCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr spheres_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	cInspection->objectName = "Spheres";
	cInspection->ReadPCLPointCloud(*ModelCloud);
	for (int i = 0; i < ModelCloud->size(); i++)
	{
		ModelCloud->points[i].r = 255;
		ModelCloud->points[i].g = 0;
		ModelCloud->points[i].b = 0;
	}
	Cloud_Display(*ModelCloud, "ModelCloud", 10, false, false, false);

	for (int k = 0; k <= Segmented_Indicates->points[Segmented_Indicates->size() - 1].x; k++)
	{
		if (recognised_cloud->size()) recognised_cloud->clear();
		for (int i = 0; i < (int)segmented_cloud->size(); i++)
		{
			if (Segmented_Indicates->points[i].x == k)
			{
				pcl::PointXYZRGB point;
				point.x = segmented_cloud->points[i].x;
				point.y = segmented_cloud->points[i].y;
				point.z = segmented_cloud->points[i].z;
				recognised_cloud->push_back(point);
			}
		}
		ui.textEdit->append("Number of Points: " + QString::number(recognised_cloud->size()));
		if (recognised_cloud->size() == 0 || recognised_cloud->size() > maxpoint || recognised_cloud->size() < minpoint) continue;

		double coff1[4], coff2[4];
		int SphereDetect = cInspection->Sphere_Detection(*recognised_cloud, threshDist, radiusMin, radiusMax, minPDetected, coff1, coff2);
		if (SphereDetect == 0) continue;
		pcl::PointXYZRGB point;
		point.x = coff1[0];
		point.y = coff1[1];
		point.z = coff1[2];
		ui.textEdit->append("X=" + QString::number(coff1[0]) + " Y=" + QString::number(coff1[1]) + " Z=" + QString::number(coff1[2]));
		ui.textEdit->append("Radius: " + QString::number(coff1[3]) + "\n");
		spheres_cloud->push_back(point);
		if (SphereDetect == 1) continue;
		point.x = coff2[0];
		point.y = coff2[1];
		point.z = coff2[2];
		ui.textEdit->append("X=" + QString::number(coff2[0]) + " Y=" + QString::number(coff2[1]) + " Z=" + QString::number(coff2[2]));
		ui.textEdit->append("Radius: " + QString::number(coff2[3]) + "\n");
		spheres_cloud->push_back(point);
	}

	if (spheres_cloud->size() == 0)
	{
		ui.textEdit->append("No ball can be detected!");
		return;
	}

	for (int i = 0; i < spheres_cloud->size(); i++)
	{
		spheres_cloud->points[i].r = 0;
		spheres_cloud->points[i].g = 255;
		spheres_cloud->points[i].b = 0;
	}
	Cloud_Display(*spheres_cloud, "spheres_cloud", 10, false, false, false);
	return;
	double matchingThresh = ui.SpinBox_Frame_MatchingThresh->text().toDouble();
	if (cInspection->Sphere_Marker_Matching(*spheres_cloud, *ModelCloud, *spheres_cloud, *spheres_cloud, matchingThresh))
	{
		ui.textEdit->append("Matching score is too low!");
		return;
	}
	for (int i = 0; i < spheres_cloud->size(); i++)
	{
		spheres_cloud->points[i].r = 255;
		spheres_cloud->points[i].g = 255;
		spheres_cloud->points[i].b = 255;
	}
	Cloud_Display(*spheres_cloud, "spheres_cloud", 10, false, false, false);
}

void threeD_Inspection::Normals()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr Surface_Cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_Cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud_, *Surface_Cloud);
	int level = ui.doubleBox_NormalLevel->text().toDouble();
	double scale = ui.doubleBox_NormalScale->text().toDouble();
	for (int i = 0; i < Surface_Cloud->size() / level; i++) input_Cloud->push_back(Surface_Cloud->points[i * level]);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(input_Cloud);
	ne.setSearchSurface(Surface_Cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(3);
	ne.compute(*cloud_normals);

	/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr quali_Cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < cloud_normals->size(); i++)
	{
		pcl::PointXYZRGB point;
		point.x = cloud_->points[i].x;
		point.y = cloud_->points[i].y;
		point.z = cloud_->points[i].z;
		if (abs(cloud_normals->points[i].normal_z) > 0.7)
		{
			ui.textEdit->append("cloud_normals z: " + QString::number(abs(cloud_normals->points[i].normal_z)));
			quali_Cloud->push_back(point);
		}
	}
	Cloud_Display(*quali_Cloud, "quali_Cloud", 2, true, true, true);*/

	Cloud_Display(*cloud_, "cloud_", 2, true, true, true);

	for (int i = 0; i < cloud_normals->size(); i++)
	{
		cloud_normals->points[i].normal_x = -1 * cloud_normals->points[i].normal_x;
		cloud_normals->points[i].normal_y = -1 * cloud_normals->points[i].normal_y;
		cloud_normals->points[i].normal_z = -1 * cloud_normals->points[i].normal_z;
	}

	ui.textEdit->append("cloud_normals z: " + QString::number(cloud_normals->points[0].normal_z));
	ui.textEdit->append("cloud_normals Size: " + QString::number(cloud_normals->size()));

	viewer_->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(input_Cloud, cloud_normals, level, scale, "normals");
	ui.qvtkWidget->update();
}

// -----------------------------------------------------0--------------------------------------------------------
// Plot
void threeD_Inspection::Initial_2D_Graph()
{
	// Plotter or Charts
	ui.qvtkWidget_graph->GetInteractor()->Initialize();
	plotter.setViewInteractor(ui.qvtkWidget_graph->GetInteractor());
	ui.qvtkWidget_graph->SetRenderWindow(plotter.getRenderWindow());

	//setting some properties
	plotter.setTitle("SubOBB Descriptor Graph");
	plotter.setShowLegend(true);
	plotter.setXTitle("Index of Subdivided Box");
	plotter.setYTitle("Normalized Surface Area");
	plotter.setColorScheme(vtkColorSeries::BLUES);
	plotter.setXRange(0, 200);
	// Sample
	double *array_X = new double[100];
	double *array_Y = new double[100];
	for (int i = 0; i < 100; i++)
	{
		array_Y[i] = rand() / (3*RAND_MAX + 1.0f);
		array_X[i] = i*2;
	}
	plotter.addPlotData(array_X, array_Y, 100, "", vtkChart::BAR);
	ui.qvtkWidget_graph->update();
}

void threeD_Inspection::Line_2D_Graph()
{
	//setting some properties
	plotter.setTitle("SubOBB Region Growing Graph");
	plotter.setShowLegend(true);
	plotter.setXTitle("Number of Iteration");
	plotter.setYTitle("Normalized Cross Correlation");
	plotter.setColorScheme(vtkColorSeries::BLUES);
	plotter.setYRange(0, 1.1);
	
	double *array_X = new double[1];
	double *array_Y = new double[1];
	array_X[0] = 0;
	array_Y[0] = 1;
	plotter.addPlotData(array_X, array_Y, 1, "", vtkChart::BAR);
	ui.qvtkWidget_graph->update();
}

void threeD_Inspection::Update_Line_2D_Graph(pcl::PointCloud<pcl::PointXYZ> &input)
{
	double *array_X = new double[1];
	double *array_Y = new double[1];
	array_X[0] = input.size() - 1;
	array_Y[0] = input.points[input.size() - 1].z;
	plotter.setXRange(0, input.size());
	plotter.addPlotData(array_X, array_Y, 1, "", vtkChart::BAR);
	ui.qvtkWidget_graph->update();
}

// -----------------------------------------------------0--------------------------------------------------------
// Checkbox Recognition
void threeD_Inspection::Recog_SubOBB_Checkbox()
{
	ui.checkBox_OSubOBB_Recognition->setChecked(false);
	ui.checkBox_nonSeg_Recognition->setChecked(false);
	ui.checkBox_SubOBB_Recognition->setChecked(true);
}

void threeD_Inspection::Recog_OSubOBB_Checkbox()
{
	ui.checkBox_SubOBB_Recognition->setChecked(false);
	ui.checkBox_nonSeg_Recognition->setChecked(false);
	ui.checkBox_OSubOBB_Recognition->setChecked(true);
}

void threeD_Inspection::Recog_nonSegSubOBB_Checkbox()
{
	ui.checkBox_SubOBB_Recognition->setChecked(false);
	ui.checkBox_OSubOBB_Recognition->setChecked(false);
	ui.checkBox_nonSeg_Recognition->setChecked(true);
}
