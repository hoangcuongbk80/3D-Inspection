#ifndef CINSPECTION_H
#define CINSPECTION_H

#include <tchar.h>
#include <stdlib.h>
#include <cmath>
#include <stdio.h>

//---------------------------------------------o0o---------------------------------------------//
#include <ANN/ANN.h>
//---------------------------------------------o0o---------------------------------------------//
// OpenCV library
#include "cv.h"
#include "highgui.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
//---------------------------------------------o0o---------------------------------------------//
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
//---------------------------------------------o0o---------------------------------------------//
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
#include <pcl/common/centroid.h>

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
#include <pcl/registration/transformation_estimation_svd.h>

// filter
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

// Segmentation

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/geometry.h>

// VTK library
#include <vtkSmartPointer.h>
#include <vtkPointSource.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkPlane.h>
#include <vtkLine.h>
#include <vtkPolyLine.h>
#include <vtkPlaneSource.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkIdList.h>
#include <vtkKdTreePointLocator.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkMath.h>
#include <vtkCleanPolyData.h>
#include <vtkMaskPoints.h>
#include <vtkArrowSource.h>
#include <vtkLineSource.h>
#include <vtkAxesActor.h>
#include <vtkOBBTree.h>
#include <vtkPolygon.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>
#include <vtkCleanPolyData.h>
#include <vtkVersion.h>
#include <vtkLandmarkTransform.h>
#include <vtkTransformFilter.h>
#include <vtkIterativeClosestPointTransform.h>
#include <vtkMatrix4x4.h>
#include <vtkMatrixToLinearTransform.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkDataSetMapper.h>
#include <vtkDelaunay3D.h>
#include <vtkCenterOfMass.h>
#include <vtkScalarBarActor.h>
#include <vtkLookupTable.h>
//---------------------------------------------o0o---------------------------------------------//

// Boost
#include <boost/math/special_functions/round.hpp>
//---------------------------------------------o0o---------------------------------------------//

using namespace std;
//---------------------------------------------o0o---------------------------------------------//


class cinspection
{
public:
	cinspection();
	~cinspection();
// -----------------------------------------------------0--------------------------------------------------------
// In-Out Functions
	bool loadCloud_txt(const std::string &filename, pcl::PointCloud<pcl::PointXYZ> &cloud);

	bool saveCloud(const std::string &filename, pcl::PointCloud<pcl::PointXYZRGB> &cloud);

	void VTK_TO_PCL(vtkSmartPointer<vtkPoints> vtk_PointCloud, pcl::PointCloud<pcl::PointXYZRGB> &pcl_PointCloud);

	void PCL_TO_VTK(pcl::PointCloud<pcl::PointXYZRGB> &pcl_PointCloud, vtkSmartPointer<vtkPoints> vtk_PointCloud);

	void ReadVTKPointCloud_txt(vtkSmartPointer<vtkPoints> vtk_PointCloud);

	void ReadVTKDescriptor(int StartLine, int EndLine, vtkSmartPointer<vtkPoints> vtk_Descriptor);

	void ReaVirtualCameraParameters(int &NumOfSamplesX, double &ZThresh);

	void ReaVirtualCameraParameters_Opt(int &NumOfSamplesX, double &ZThresh);

	void ReaVirtualCameraParameters_Nonseg(int &NumOfSamplesX, double &ZThresh);

	void VTKpoint_To_VTKpoint(vtkSmartPointer<vtkPoints>  Input_source_points, vtkSmartPointer<vtkPoints>  &Output_source_points, double m);

	void ReadPCLPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &pcl_PointCloud);

	void ReadPCLPointCloud_Opt(pcl::PointCloud<pcl::PointXYZRGB> &pcl_PointCloud);

	void ReadPCLPointCloud_Nonseg(pcl::PointCloud<pcl::PointXYZRGB> &pcl_PointCloud);

	void ReadPCLDescriptor(int StartLine, int EndLine, pcl::PointCloud<pcl::PointXYZRGB> &Descriptor);

	void ReadPCLDescriptor_Opt(pcl::PointCloud<pcl::PointXYZRGB> &pcl_Descriptor);

	void ReadPCLDescriptor_Nonseg(pcl::PointCloud<pcl::PointXYZRGB> &pcl_Descriptor);

	void ReadSCanPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &pcl_PointCloud, int scan);

// -----------------------------------------------------0--------------------------------------------------------
// VTK Functions
	void VTK_OBB_Calculation(vtkSmartPointer<vtkPoints> points, double *OBB_Conner, double *OBB_Max, double *OBB_Mid, double *OBB_Min);

	void VTK_RealcoordinateToOBBcoordinateXY(vtkSmartPointer<vtkPoints> &input, vtkSmartPointer<vtkPoints> &output, double corner_OBB[3],
		double max_OBB[3], double mid_OBB[3], double min_OBB[3]);

	void VTKCenterOfMass(vtkPoints* points, double* center);

	void VTK_Translation(vtkSmartPointer<vtkPoints> source_points, double x, double y, double z, double Rx, double Ry,
		 double Rz, vtkSmartPointer<vtkPoints> &transformed_points);

	void VTK_kNearestMeanDistance(vtkSmartPointer<vtkPoints> input, int Nearest_K, double &thresh);

	bool kNearestMeanDist_PCL(pcl::PointCloud<pcl::PointXYZRGB> &input, int Nearest_K, double &meandist);

	void VTK_matching_scence_model(vtkSmartPointer<vtkPoints> ObjCloud, vtkSmartPointer<vtkPoints> ModelDescrip, double OBBmodelMid,
		 double OBBscenceMax, double adaptResol, double objNumOfSubOBBX, double objNumOfSubOBBY, double modelNumOfSubOBBX, 
		 double modelNumOfSubOBBY, double &StartX, double &StartY, double &BestRota, double &NCCmax, int countView);

	void VTK_OBB_Descriptor(vtkSmartPointer<vtkPoints> Input, vtkSmartPointer<vtkPoints> &Descriptor, int numOfSubOBBX, int numOfSubOBBY,
		 int numOfModel);

	void VTK_ExtractVitualPoints(vtkSmartPointer<vtkPoints> &vtk_PointClouds, vtkSmartPointer<vtkPoints> &ExtractedCloud,
		 vtkSmartPointer<vtkPoints> VirtualPoints, int NumOfSamplesX, double ratioZ, double startX, double NumOfSubOBBX, double startY,
		 double NumOfSubOBBY, double NumofOBB_model[3]);

	void VTK_ICP_Getmatrix(vtkSmartPointer<vtkPoints> source_points, vtkSmartPointer<vtkPoints> target_points,
		 vtkSmartPointer<vtkIterativeClosestPointTransform> &icp, int i);

	void VTK_ICP_Apply(vtkSmartPointer<vtkPoints> source_points, vtkSmartPointer<vtkPoints> &transformed_points,
		 vtkSmartPointer<vtkIterativeClosestPointTransform> icp);

	void VTK_MeanDistance_CloudToCloud(vtkSmartPointer<vtkPoints> source, vtkSmartPointer<vtkPoints> target, double &meandist);

	unsigned char VTK_OBB_Recognition(pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output,
		double toler, double pass);

// -----------------------------------------------------0--------------------------------------------------------
// PCL Functions
	bool PCL_MeanDistance_CloudToCloud(pcl::PointCloud<pcl::PointXYZRGB> &source, pcl::PointCloud<pcl::PointXYZRGB> &target,
		 double &meandist);

	void template_Matching(pcl::PointCloud<pcl::PointXYZRGB> &searched_Descrip, pcl::PointCloud<pcl::PointXYZRGB> &templ_Descrip,
		pcl::PointCloud<pcl::PointXYZ> &result, int numOfSubOBB_searched[3], int numOfSuOBB_temp[3]);

	void template_Matching_Nonseg(pcl::PointCloud<pcl::PointXYZRGB> &searched_Descrip, pcl::PointCloud<pcl::PointXYZRGB> &templ_Descrip,
		pcl::PointCloud<pcl::PointXYZ> &result, int numOfSubOBB_searched[3], int numOfSuOBB_temp[3]);

	bool PCL_Overlap_Detection(pcl::PointCloud<pcl::PointXYZRGB> &source, pcl::PointCloud<pcl::PointXYZRGB> &target,
		 pcl::PointCloud<pcl::PointXYZRGB> &output, double thresh);

	void DownSampling(pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output, double ratio);

	void Euclidean_Cluster_Extraction(pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &cloud_NoiseRemoval,
		 pcl::PointCloud<pcl::PointXYZRGB> &Segmented_Index, pcl::PointCloud<pcl::PointXYZRGB> &features, double tolerance, int minpoints);

	void colorCloudDistances(pcl::PointCloud<pcl::PointXYZRGB> &cloud_);

	void OBB_features(const pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output);

	void Cylinder_Detection(const pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &inliers,
		 pcl::PointCloud<pcl::PointXYZRGB> &outliners, double DistanceThreshold, double RadiusMin, double RadiusMax);

	int Sphere_Detection(pcl::PointCloud<pcl::PointXYZRGB> &input, double DistanceThresh, double radiusMin, double radiusMax, int minPoint, double *coff1, double *coff2);

	bool Sphere_Marker_Matching(pcl::PointCloud<pcl::PointXYZRGB> &source, pcl::PointCloud<pcl::PointXYZRGB> &target, 
		pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output, double thresh);

	bool Sphere_Marker_Matching_Fix(pcl::PointCloud<pcl::PointXYZRGB> &source, pcl::PointCloud<pcl::PointXYZRGB> &target,
		pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output, int mod_index, double thresh);

	bool PCL_NearestNeighbors_Search(pcl::PointCloud<pcl::PointXYZRGB> &source, pcl::PointCloud<pcl::PointXYZRGB> &target,
		pcl::PointCloud<pcl::PointXYZRGB> &output, int index, int k);

	void Denoise_Statistical(const pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output, 
		 double StddevMulThresh);

	void Denoise_Segment(const pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output,
		 double tolerance, int minpoints);

	void OBB_Inliers(pcl::PointCloud<pcl::PointXYZRGB> &input, const pcl::PointCloud<pcl::PointXYZRGB> &input_OBB,
		 pcl::PointCloud<pcl::PointXYZRGB> &output_Inliers, pcl::PointCloud<pcl::PointXYZRGB> &output_Outliers);

	void Distances_CloudToCloud(pcl::PointCloud<pcl::PointXYZRGB> &source, pcl::PointCloud<pcl::PointXYZRGB> &target,
		 pcl::PointCloud<pcl::PointXYZ> &dists_Cloud);

	void Distances_CloudToCloud_PCL(pcl::PointCloud<pcl::PointXYZRGB> &source, pcl::PointCloud<pcl::PointXYZRGB> &target,
		pcl::PointCloud<pcl::PointXYZ> &dists_Cloud);

	void Display_Error_Report(pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZ> &dists_Cloud);

	bool threePointToPlane(pcl::PointCloud<pcl::PointXYZ> &input, double &a, double &b, double &c, double &d);

	void OBB_Calculation(pcl::PointCloud<pcl::PointXYZRGB> &input, double *OBB_Conner, double *OBB_Max, double *OBB_Mid, double *OBB_Min);

	void RealcoordinateToOBBcoordinateXY(pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output,
		 double corner_OBB[3], double max_OBB[3], double mid_OBB[3], double min_OBB[3]);

	void TransferToOBB_CorrdinateXY(pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output);

	void OBB_Length(pcl::PointCloud<pcl::PointXYZRGB> &input, double *OBB_Length);

	bool numOfSubOBB_Estimation(pcl::PointCloud<pcl::PointXYZRGB> &object, pcl::PointCloud<pcl::PointXYZRGB> &model, int *numOfSubOBB);

	void OBB_Alignment(pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output);

	void OBB_Descriptor(pcl::PointCloud<pcl::PointXYZRGB> &Input, pcl::PointCloud<pcl::PointXYZRGB> &Descriptor, int numOfSubOBBX, 
		int numOfSubOBBY, double &subobbAreaMax);

	void OBB_Descriptor_Opt(pcl::PointCloud<pcl::PointXYZRGB> &Input, pcl::PointCloud<pcl::PointXYZRGB> &Descriptor, int numOfSub[3], double &subobbAreaMax);

	void OBB_Descriptor_Nonseg(pcl::PointCloud<pcl::PointXYZRGB> &Input, pcl::PointCloud<pcl::PointXYZRGB> &Descriptor, double &subobbAreaMax);

	void Opt_OBB_Descriptor(pcl::PointCloud<pcl::PointXYZRGB> &Input, pcl::PointCloud<pcl::PointXYZRGB> &Descriptor, int numOfSubOBB[3], double *maxFeatures);

	void Nonseg_OBB_Descriptor(pcl::PointCloud<pcl::PointXYZRGB> &Input, pcl::PointCloud<pcl::PointXYZRGB> &Descriptor, double ViewPoint[3], int inCreZ);

	void OBB_matching_scence_model(pcl::PointCloud<pcl::PointXYZRGB> &ObjCloud, pcl::PointCloud<pcl::PointXYZRGB> &ModelDescrip,
		 double OBBmodelMid, double OBBscenceMax, double subobbAreaMax_mod, double objNumOfSubOBBX, double objNumOfSubOBBY, double modelNumOfSubOBBX,
		 double modelNumOfSubOBBY, double &NCCmax, int countView);

	void SubOBB_matching_scence_model(pcl::PointCloud<pcl::PointXYZRGB> &ObjCloud, pcl::PointCloud<pcl::PointXYZRGB> &ModelDescrip,
	     double OBBmodelMid, double OBBscenceMax, double subobbAreaMax_mod, double objNumOfSubOBBX, double objNumOfSubOBBY, double modelNumOfSubOBBX,
		 double modelNumOfSubOBBY, double &StartX, double &StartY, double &BestRota, double &NCCmax, int countView);

	bool SubOBB_matching_scence_model_Opt(pcl::PointCloud<pcl::PointXYZRGB> &ObjectCloud, int *numOfSubOBB_object, pcl::PointCloud<pcl::PointXYZ> &PCLBestView,
		pcl::PointCloud<pcl::PointXYZ> &PCLBestRota, pcl::PointCloud<pcl::PointXYZ> &PCLBestNCC, pcl::PointCloud<pcl::PointXYZ> &PCLBestNumOfSubOBB);

	bool SubOBB_matching_model_scene_nonSeg(pcl::PointCloud<pcl::PointXYZRGB> &SceneCloud, int *numOfSubOBB_Scene, pcl::PointCloud<pcl::PointXYZ> 
		 &PCLBestView, pcl::PointCloud<pcl::PointXYZ> &PCLBestRota_mod, pcl::PointCloud<pcl::PointXYZ> &PCLBestNCC, pcl::PointCloud<pcl::PointXYZ> &PCLBestNumOfSubOBB);

	int RG_SubOBB_Matching(pcl::PointCloud<pcl::PointXYZRGB> &objCloud, pcl::PointCloud<pcl::PointXYZRGB> &model,
		pcl::PointCloud<pcl::PointXYZ> &PCLBestRota_mod, pcl::PointCloud<pcl::PointXYZ> &PCLBestView_mod, pcl::PointCloud<pcl::PointXYZ> &PCLBestNCC, 
		pcl::PointCloud<pcl::PointXYZ> &PCLBestNumOfSubOBB, double NCCThresh);

	void OBB_ExtractVitualPoints(pcl::PointCloud<pcl::PointXYZRGB> &pcl_PointClouds, pcl::PointCloud<pcl::PointXYZRGB> &ExtractedCloud,
		 pcl::PointCloud<pcl::PointXYZRGB> &VirtualPoints, int NumOfSamplesX, double ratioZ);

	void SubOBB_ExtractVitualPoints(pcl::PointCloud<pcl::PointXYZRGB> &pcl_PointClouds,
		 pcl::PointCloud<pcl::PointXYZRGB> &ExtractedCloud, pcl::PointCloud<pcl::PointXYZRGB> &VirtualPoints,
		 int NumOfSamplesX, double ratioZ, double startX, double NumOfSubOBBX, double startY, double NumOfSubOBBY, double NumofOBB_model[3]);

	bool SubOBB_ExtractVitualPoints_Opt(pcl::PointCloud<pcl::PointXYZRGB> &cloud_Mod, pcl::PointCloud<pcl::PointXYZRGB> &cloud_Extracted,
		double bestView[3], double bestRota[3], double numOfOBB_mod[3], int numOfOBB_obj[3], double matchingPoint[3], int NumOfSamplesX, double ZThresh);

	bool RG_SubOBB_StartFind(pcl::PointCloud<pcl::PointXYZRGB> &Input, pcl::PointCloud<pcl::PointXYZRGB> &output, int numOfSubOBB_scene[3]);

	void RG_SubOBB_ObjExtraction(pcl::PointCloud<pcl::PointXYZRGB> &Input, pcl::PointCloud<pcl::PointXYZRGB> &output,
		int numOfSubOBB_scene[3], int pickX, int pickY);

	void Nonseg_SubOBB_ObjExtraction(pcl::PointCloud<pcl::PointXYZRGB> &Input, pcl::PointCloud<pcl::PointXYZRGB> &output,
		int numOfSubOBB_Obj[3], int starPt[3]);

	void ICP_Registration(pcl::PointCloud<pcl::PointXYZRGB> &source_cloud, pcl::PointCloud<pcl::PointXYZRGB> &target_cloud,
		 pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output, int iter);

	void Inverse_ICP_Registration(pcl::PointCloud<pcl::PointXYZRGB> &source_cloud, pcl::PointCloud<pcl::PointXYZRGB> &target_cloud,
		pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output, int iter);

	unsigned char OBB_Recognition(pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output, double toler, double pass);

	unsigned char SubOBB_Recognition(pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output, double toler, double pass);

	unsigned char RG_SubOBB_Recognition(pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output, double toler, double pass);

	unsigned char model_Driven_Registration(pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output, double toler, double pass);

	unsigned char Optimized_SubOBB_Recognition(pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output, double NCCThresh,
		          double tolerUp, double tolerDown, double pass);

	unsigned char NonSeg_SubOBB_Recog(pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output, double pass);

	QString objectName;
	ofstream offlineFile;
	int desMethd[3];
	int reportCount;
	double noiseRatio;
	int NumOfSubOBBX_model;
	int NumOfSubOBBY_model;
	double SubOBB_Size;
	Eigen::Matrix4f matr;

private:
	
};

#endif // CINSPECTION_H
