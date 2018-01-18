#include "cinspection.h"

cinspection::cinspection()
{
	offlineFile.open("Report/Report_offline.txt");
	desMethd[0] = 0;
	desMethd[1] = 0;
	desMethd[2] = 0;
	reportCount = 0;
	NumOfSubOBBX_model = 15;
	NumOfSubOBBY_model = 15;
	noiseRatio = DBL_MAX;
	matr = Eigen::Matrix4f::Identity();
}

cinspection::~cinspection()
{
	offlineFile.close();
}

// -----------------------------------------------------0--------------------------------------------------------
// In-Out Functions
bool cinspection::loadCloud_txt(const std::string &filename, pcl::PointCloud<pcl::PointXYZ> &cloud)
{
	ifstream fs;
	fs.open(_TEXT(filename.c_str()), ios::binary);

	if (!fs.is_open() || fs.fail())
	{
		return 0;
	}

	string line;
	vector<string> st;
	while (!fs.eof())
	{
		getline(fs, line);
		// Tokenize the line
		boost::trim(line);
		boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);

		if (st.size() >= 3)
		{
			pcl::PointXYZ point;
			point.x = float(atof(st[0].c_str()));
			point.y = float(atof(st[1].c_str()));
			point.z = float(atof(st[2].c_str()));
			cloud.push_back(point);
		}
	}
	fs.close();
	return 1;
}

bool cinspection::saveCloud(const std::string &filename, pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
	ofstream saveFile;
	saveFile.open(_TEXT(filename.c_str()), ios::binary);
	if (!saveFile.is_open() || saveFile.fail())
	{
		return 0;
	}

	for (size_t i = 0; i < cloud.size(); i++)
	{
		saveFile << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << "\n";
	}
	saveFile.close();
	return 1;
}

void cinspection::ReadVTKPointCloud_txt(vtkSmartPointer<vtkPoints> vtk_PointCloud)
{
	ifstream fs;
	string name;
	name.append("CADdata\\");
	name.append(objectName.toUtf8().constData());
	name.append("_model.txt");
	fs.open(name);

	string line;
	vector<string> st;
	while (!fs.eof())
	{
		getline(fs, line);
		// Tokenize the line
		boost::trim(line);
		boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);

		if (st.size() == 3)
		{
			double point[3];
			point[0] = float(atof(st[0].c_str()));
			point[1] = float(atof(st[1].c_str()));
			point[2] = float(atof(st[2].c_str()));
			vtk_PointCloud->InsertNextPoint(point);
		}
	}
	fs.close();
}

void cinspection::VTK_TO_PCL(vtkSmartPointer<vtkPoints> vtk_PointCloud, pcl::PointCloud<pcl::PointXYZRGB> &pcl_PointCloud)
{
	for (int i = 0; i < vtk_PointCloud->GetNumberOfPoints(); i++)
	{
		double p[3];
		vtk_PointCloud->GetPoint(i, p);
		pcl::PointXYZRGB point;
		point.x = p[0];
		point.y = p[1];
		point.z = p[2];
		pcl_PointCloud.push_back(point);
	}
}

void cinspection::PCL_TO_VTK(pcl::PointCloud<pcl::PointXYZRGB> &pcl_PointCloud, vtkSmartPointer<vtkPoints> vtk_PointCloud)
{
	for (int i = 0; i < (int)pcl_PointCloud.size(); i++)
	{
		double point[3];
		point[0] = pcl_PointCloud.points[i].x;
		point[1] = pcl_PointCloud.points[i].y;
		point[2] = pcl_PointCloud.points[i].z;
		vtk_PointCloud->InsertNextPoint(point);
	}
}

void cinspection::ReaVirtualCameraParameters(int &NumOfSamplesX, double &ZThresh)
{
	ifstream fs;
	string name;
	name.append("CADdata\\");
	name.append(objectName.toUtf8().constData());
	name.append("_parameters.txt");
	fs.open(name);

	string line;
	vector<string> st;
	int count = 0;
	while (!fs.eof())
	{
		getline(fs, line);
		// Tokenize the line
		boost::trim(line);
		boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);
		if (count == 0) NumOfSamplesX = int(atof(st[0].c_str()));
		if (count == 1) ZThresh = double(atof(st[0].c_str()));
		if (count == 2) noiseRatio = double(atof(st[0].c_str()));
		if (count == 3) NumOfSubOBBX_model = int(atof(st[0].c_str()));
		if (count == 4) NumOfSubOBBY_model = int(atof(st[0].c_str()));
		count++;
	}
	fs.close();
}

void cinspection::ReaVirtualCameraParameters_Opt(int &NumOfSamplesX, double &ZThresh)
{
	ifstream fs;
	string name;
	name.append("CADdataOpt\\");
	name.append(objectName.toUtf8().constData());
	name.append("_parameters.txt");
	fs.open(name);

	string line;
	vector<string> st;
	int count = 0;
	while (!fs.eof())
	{
		getline(fs, line);
		// Tokenize the line
		boost::trim(line);
		boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);
		if (count == 0) NumOfSamplesX = int(atof(st[0].c_str()));
		if (count == 1) ZThresh = double(atof(st[0].c_str()));
		if (count == 2) noiseRatio = double(atof(st[0].c_str()));
		if (count == 3) NumOfSubOBBX_model = int(atof(st[0].c_str()));
		if (count == 4) NumOfSubOBBY_model = int(atof(st[0].c_str()));
		if (count == 5) SubOBB_Size = int(atof(st[0].c_str()));
		count++;
	}
	fs.close();
}

void cinspection::ReaVirtualCameraParameters_Nonseg(int &NumOfSamplesX, double &ZThresh)
{
	ifstream fs;
	string name;
	name.append("CADdataNonseg\\");
	name.append(objectName.toUtf8().constData());
	name.append("_parameters.txt");
	fs.open(name);

	string line;
	vector<string> st;
	int count = 0;
	while (!fs.eof())
	{
		getline(fs, line);
		// Tokenize the line
		boost::trim(line);
		boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);
		if (count == 0) NumOfSamplesX = int(atof(st[0].c_str()));
		if (count == 1) ZThresh = double(atof(st[0].c_str()));
		if (count == 2) noiseRatio = double(atof(st[0].c_str()));
		if (count == 3) NumOfSubOBBX_model = int(atof(st[0].c_str()));
		if (count == 4) NumOfSubOBBY_model = int(atof(st[0].c_str())); 
		if (count == 5) SubOBB_Size = int(atof(st[0].c_str()));
		count++;
	}
	fs.close();
}

void cinspection::VTKpoint_To_VTKpoint(vtkSmartPointer<vtkPoints>  Input_source_points, vtkSmartPointer<vtkPoints>  &Output_source_points, double m)
{
	for (vtkIdType i = 0; i < Input_source_points->GetNumberOfPoints(); i++)
	{
		double p[3];
		Input_source_points->GetPoint(i, p);
		if (p[2] < m)
		{
			Output_source_points->InsertNextPoint(p);
		}
	}
}

void cinspection::ReadVTKDescriptor(int StartLine, int EndLine, vtkSmartPointer<vtkPoints> vtk_Descriptor)
{
	//if (vtk_Descriptor->GetNumberOfPoints() != 0) vtk_Descriptor = vtkSmartPointer<vtkPoints>::New();
	ifstream fs;
	string name;
	name.append("CADdata\\");
	name.append(objectName.toUtf8().constData());
	name.append("_descriptor.txt");
	fs.open(name);

	string line;
	vector<string> st;
	int count = 0;
	while (!fs.eof())
	{
		getline(fs, line);
		// Tokenize the line
		boost::trim(line);
		boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);
		if (EndLine == count & st.size() > 3)
		{
			for (int i = 0; i < st.size(); i += 3)
			{
				double point[3];
				point[0] = float(atof(st[i].c_str()));
				point[1] = float(atof(st[i + 1].c_str()));
				point[2] = float(atof(st[i + 2].c_str()));
				vtk_Descriptor->InsertNextPoint(point);
			}

		}
		count++;
	}
	fs.close();

}

void cinspection::ReadPCLPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &pcl_PointCloud)
{
	ifstream fs;
	string name;
	name.append("CADdata\\");
	name.append(objectName.toUtf8().constData());
	name.append("_model.txt");
	fs.open(name);
	if (!fs.is_open() || fs.fail())
	{
		return;
	}

	string line;
	vector<string> st;
	while (!fs.eof())
	{
		getline(fs, line);
		// Tokenize the line
		boost::trim(line);
		boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);

		if (st.size() == 3)
		{
			pcl::PointXYZRGB point;
			point.x = float(atof(st[0].c_str()));
			point.y = float(atof(st[1].c_str()));
			point.z = float(atof(st[2].c_str()));
			point.r = 255;
			point.g = 255;
			point.b = 255;
			pcl_PointCloud.push_back(point);
		}
	}
	fs.close();
}

void cinspection::ReadPCLPointCloud_Opt(pcl::PointCloud<pcl::PointXYZRGB> &pcl_PointCloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz(new pcl::PointCloud<pcl::PointXYZ>());
	string name;
	name.append("CADdataOpt\\");
	name.append(objectName.toUtf8().constData());
	name.append("_model.pcd");

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(name, *cloudxyz) == -1) return;
	pcl::copyPointCloud(*cloudxyz, pcl_PointCloud);
}

void cinspection::ReadPCLPointCloud_Nonseg(pcl::PointCloud<pcl::PointXYZRGB> &pcl_PointCloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz(new pcl::PointCloud<pcl::PointXYZ>());
	string name;
	name.append("CADdataNonseg\\");
	name.append(objectName.toUtf8().constData());
	name.append("_model.pcd");

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(name, *cloudxyz) == -1) return;
	pcl::copyPointCloud(*cloudxyz, pcl_PointCloud);
}

void cinspection::ReadPCLDescriptor(int StartLine, int EndLine, pcl::PointCloud<pcl::PointXYZRGB> &pcl_Descriptor)
{
	ifstream fs;
	string name;
	name.append("CADdata\\");
	name.append(objectName.toUtf8().constData());
	name.append("_descriptor.txt");
	fs.open(name);

	string line;
	vector<string> st;
	int count = 0;
	while (!fs.eof())
	{
		getline(fs, line);
		// Tokenize the line
		boost::trim(line);
		boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);
		if (EndLine == count & st.size() > 3)
		{
			for (int i = 0; i < st.size(); i += 3)
			{
				pcl::PointXYZRGB point;
				point.x = float(atof(st[i].c_str()));
				point.y = float(atof(st[i + 1].c_str()));
				point.z = float(atof(st[i + 2].c_str()));
				pcl_Descriptor.push_back(point);
			}

		}
		count++;
	}
	fs.close();
}

void cinspection::ReadPCLDescriptor_Opt(pcl::PointCloud<pcl::PointXYZRGB> &pcl_Descriptor)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz(new pcl::PointCloud<pcl::PointXYZ>());
	string name;
	name.append("CADdataOpt\\");
	name.append(objectName.toUtf8().constData());
	name.append("_descriptor.pcd");

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(name, *cloudxyz) == -1) return;
	pcl::copyPointCloud(*cloudxyz, pcl_Descriptor);
}

void cinspection::ReadPCLDescriptor_Nonseg(pcl::PointCloud<pcl::PointXYZRGB> &pcl_Descriptor)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz(new pcl::PointCloud<pcl::PointXYZ>());
	string name;
	name.append("CADdataNonseg\\");
	name.append(objectName.toUtf8().constData());
	name.append("_descriptor.pcd");

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(name, *cloudxyz) == -1) return;
	pcl::copyPointCloud(*cloudxyz, pcl_Descriptor);
}

void cinspection::ReadSCanPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &pcl_PointCloud, int scan)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz(new pcl::PointCloud<pcl::PointXYZ>());
	string name;
	name.append("Scan\\");
	name.append(objectName.toUtf8().constData());
	QString view = QString::number(scan);
	name.append("\\");
	name.append(view.toUtf8().constData());
	name.append(".pcd");

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(name, *cloudxyz) == -1) return;
	pcl::copyPointCloud(*cloudxyz, pcl_PointCloud);
}

// -----------------------------------------------------0--------------------------------------------------------
// VTK Functions
void cinspection::VTK_OBB_Calculation(vtkSmartPointer<vtkPoints> points, double *OBB_Conner, double *OBB_Max, double *OBB_Mid, double *OBB_Min)
{
	OBB_Conner[0] = 0;
	OBB_Conner[1] = 0;
	OBB_Conner[2] = 0;
	OBB_Min[0] = 0;
	OBB_Min[1] = 0;
	OBB_Min[2] = 0;
	OBB_Mid[0] = 0;
	OBB_Mid[1] = 0;
	OBB_Mid[2] = 0;
	OBB_Max[0] = 0;
	OBB_Max[1] = 0;
	OBB_Max[2] = 0;

	// Create the tree
	vtkSmartPointer<vtkOBBTree> obbTree = vtkSmartPointer<vtkOBBTree>::New();
	double 	max_OBB[3], mid_OBB[3], min_OBB[3], size_OBB[3];
	obbTree->ComputeOBB(points, OBB_Conner, max_OBB, mid_OBB, min_OBB, size_OBB);
	for (int i = 0; i < 3; i++)
	{
		OBB_Max[i] = /*OBB_Conner[i] +*/ max_OBB[i];
		OBB_Mid[i] = /*OBB_Conner[i] +*/ mid_OBB[i];
		OBB_Min[i] = /*OBB_Conner[i] +*/ min_OBB[i];
	}
}

void cinspection::VTK_RealcoordinateToOBBcoordinateXY(vtkSmartPointer<vtkPoints> &input, vtkSmartPointer<vtkPoints> &output,
	double corner_OBB[3], double max_OBB[3], double mid_OBB[3], double min_OBB[3])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr SVD_Source(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr SVD_Target(new pcl::PointCloud<pcl::PointXYZ>());

	double lengthOBB_object[3];
	lengthOBB_object[0] = sqrt(max_OBB[0] * max_OBB[0] + max_OBB[1] * max_OBB[1] + max_OBB[2] * max_OBB[2]); // length of max
	lengthOBB_object[1] = sqrt(mid_OBB[0] * mid_OBB[0] + mid_OBB[1] * mid_OBB[1] + mid_OBB[2] * mid_OBB[2]); // length of mid
	lengthOBB_object[2] = sqrt(min_OBB[0] * min_OBB[0] + min_OBB[1] * min_OBB[1] + min_OBB[2] * min_OBB[2]); // length of min
	double point[4][3];
	for (int i = 0; i < 3; i++)
	{
		point[0][i] = corner_OBB[i];
		point[1][i] = corner_OBB[i] + (double) max_OBB[i] / lengthOBB_object[0];
		point[2][i] = corner_OBB[i] + (double) mid_OBB[i] / lengthOBB_object[1];
		point[3][i] = corner_OBB[i] + (double) min_OBB[i] / lengthOBB_object[2];
	}
	for (int i = 0; i < 3; i++) SVD_Source->push_back(pcl::PointXYZ(point[i][0], point[i][1], point[i][2]));

	SVD_Target->push_back(pcl::PointXYZ(0, 0, 0));
	SVD_Target->push_back(pcl::PointXYZ(1, 0, 0));
	SVD_Target->push_back(pcl::PointXYZ(0, 1, 0));

	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> TESVD;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 SVD_transformation;
	TESVD.estimateRigidTransformation(*SVD_Source, *SVD_Target, SVD_transformation);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SVD_Source_Cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SVD_Target_Cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	VTK_TO_PCL(input, *SVD_Source_Cloud);
	pcl::transformPointCloud(*SVD_Source_Cloud, *SVD_Target_Cloud, SVD_transformation);
	if (output->GetNumberOfPoints()) output = vtkSmartPointer<vtkPoints>::New();
	PCL_TO_VTK(*SVD_Target_Cloud, output);
	matr = SVD_transformation.replicate(4, 4);
}

void cinspection::VTKCenterOfMass(vtkPoints* points, double* center)
{
	center[0] = 0.0;
	center[1] = 0.0;
	center[2] = 0.0;

	for (vtkIdType i = 0; i < points->GetNumberOfPoints(); i++)
	{
		double point[3];
		points->GetPoint(i, point);

		center[0] += point[0];
		center[1] += point[1];
		center[2] += point[2];
	}

	double numberOfPoints = static_cast<double>(points->GetNumberOfPoints());
	center[0] = center[0] / numberOfPoints;
	center[1] = center[1] / numberOfPoints;
	center[2] = center[2] / numberOfPoints;
}

void cinspection::VTK_Translation(vtkSmartPointer<vtkPoints> source_points, double x, double y, double z, double Rx, double Ry, double Rz, vtkSmartPointer<vtkPoints> &transformed_points)
{
	vtkSmartPointer<vtkPolyData> source = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> source1 = vtkSmartPointer<vtkPolyData>::New();
	source1->SetPoints(source_points);

	vtkSmartPointer<vtkVertexGlyphFilter> glyphFilterSource = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	glyphFilterSource->SetInputData(source1);
	glyphFilterSource->Update();
	source->ShallowCopy(glyphFilterSource->GetOutput());

	// TRanslate and rotation
	vtkSmartPointer<vtkTransform> translation = vtkSmartPointer<vtkTransform>::New();
	translation->Translate(x, y, z);
	translation->RotateX(Rx);
	translation->RotateY(Ry);
	translation->RotateZ(Rz);

	vtkSmartPointer<vtkTransformFilter> transformFilter = vtkSmartPointer<vtkTransformFilter>::New();
#if VTK_MAJOR_VERSION <= 5
	transformFilter->SetInputConnection(source->GetProducerPort());
#else
	transformFilter->SetInputData(source);
#endif
	transformFilter->SetTransform(translation);
	transformFilter->Update();
	/*vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	#if VTK_MAJOR_VERSION <= 5
	transformFilter->SetInputConnection(source->GetProducerPort());
	#else
	transformFilter->SetInputData(source);
	#endif
	transformFilter->SetTransform(translation);
	transformFilter->Update();*/
	transformed_points = vtkSmartPointer<vtkPoints>::New();
	transformed_points = transformFilter->GetOutput()->GetPoints();
}

void cinspection::VTK_kNearestMeanDistance(vtkSmartPointer<vtkPoints> input, int Nearest_K, double &thresh)
{
	int PointNum_query = (int)input->GetNumberOfPoints();
	int PointNum_data = (int)input->GetNumberOfPoints();
	thresh = 0;
	//--------------------------------------------------
	// KNN METHOD for finding neighbors
	//--------------------------------------------------	

	int knn = Nearest_K;
	int dim = 3;						// dimension
	double	eps = 0;						// error bound
	int maxPts_query = PointNum_query;					// maximum number of data points
	int maxPts_data = PointNum_data;					// maximum number of data points

	int				ndataPts;					// actual number of data points
	int				nqueryPts;					// actual number of data points
	ANNpointArray	dataPts;				// data points
	ANNpointArray	queryPts;
	ANNpoint		queryPt;				// query point
	ANNidxArray		nnIdx;					// near neighbor indices
	ANNdistArray	dists;					// near neighbor distances
	ANNkd_tree*		kdTree;					// search structure

	queryPt = annAllocPt(dim);				// allocate query point
	dataPts = annAllocPts(maxPts_data, dim);	    // allocate data points
	queryPts = annAllocPts(maxPts_query, dim);	    // allocate data points
	nnIdx = new ANNidx[knn];				// allocate near neigh indices
	dists = new ANNdist[knn];				// allocate near neighbor dists

	ndataPts = 0;

	while (ndataPts < maxPts_data) {
		double p[3];
		input->GetPoint(ndataPts, p);
		dataPts[ndataPts][0] = p[0];
		dataPts[ndataPts][1] = p[1];
		dataPts[ndataPts][2] = p[2];
		ndataPts++;
	}

	nqueryPts = 0;
	while (nqueryPts < maxPts_query) {
		double q[3];
		input->GetPoint(nqueryPts, q);
		queryPts[nqueryPts][0] = q[0];
		queryPts[nqueryPts][1] = q[1];
		queryPts[nqueryPts][2] = q[2];
		nqueryPts++;
	}

	kdTree = new ANNkd_tree(	// build search structure
		dataPts,				// the data points
		ndataPts,				// number of points
		dim);					// dimension of space

	for (int searchP = 0; searchP < PointNum_query; searchP++)
	{
		queryPt = queryPts[searchP];	// Query the first point
		kdTree->annkSearch(			// search
			queryPt,				// query point
			knn,					// number of near neighbors
			nnIdx,					// nearest neighbors (returned)
			dists,					// squared distance (returned)
			eps);					// error bound
		int total = 0;
		for (int k = 1; k < Nearest_K; k++)
		{
			total += (double)sqrt(dists[k]);
		}
		thresh += (double)total / (Nearest_K - 1);
	}
	thresh = (double)thresh / PointNum_query;

	delete[] nnIdx;							// clean things up
	delete[] dists;
	delete kdTree;
	annClose();									// done with ANN
}

void cinspection::VTK_matching_scence_model(vtkSmartPointer<vtkPoints> ObjCloud, vtkSmartPointer<vtkPoints> ModelDescrip, double OBBmodelMid, double OBBscenceMax,
	 double adaptResol, double objNumOfSubOBBX, double objNumOfSubOBBY, double modelNumOfSubOBBX, double modelNumOfSubOBBY, double &StartX, double &StartY, double &BestRota, double &NCCmax, int countView)
{
	reportCount++;
	vtkSmartPointer<vtkPoints> Cloud = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints>  ObjDescrip = vtkSmartPointer<vtkPoints>::New();
	NCCmax = -1;

	cv::Mat modelimg(modelNumOfSubOBBY, modelNumOfSubOBBX, CV_8UC3, cv::Scalar(0, 0, 0));
	cv::Mat gray_modelimg;
	cv::cvtColor(modelimg, gray_modelimg, CV_BGR2GRAY);

	for (int i = 3; i<ModelDescrip->GetNumberOfPoints(); i++)
	{
		double p[3];
		ModelDescrip->GetPoint(i, p);
		gray_modelimg.at<uchar>(p[1], p[0]) = p[2] * 10000;
	}
	cv::normalize(gray_modelimg, gray_modelimg, 0, 255, cv::NORM_MINMAX, CV_8UC1);

	// Report and Debug
	/*string name;
	name.append("Report/model_extract_");
	QString num = QString::number(reportCount);
	name.append(num.toUtf8().constData());
	name.append(".jpg");
	cv::imwrite(name, gray_modelimg);*/

	// 8 corners of OBB can be as OBB origin, but here kz = 1, so possibilities is 4
	// OX and OY can be excahnged, so every corner we have 2 possibilites
	// Therefore, total posibilites is 8
	for (int RotaX = 0; RotaX < 360; RotaX += 180)
	for (int RotaZ = 0; RotaZ < 360; RotaZ += 90)
	{
		VTK_Translation(ObjCloud, 0, 0, 0, RotaX, 0, 0, Cloud);
		VTK_Translation(Cloud, 0, 0, 0, 0, 0, RotaZ, Cloud);
		VTK_OBB_Descriptor(Cloud, ObjDescrip, objNumOfSubOBBX, objNumOfSubOBBY, adaptResol);

		cv::Mat objimg(objNumOfSubOBBY, objNumOfSubOBBX, CV_8UC3, cv::Scalar(0, 0, 0));
		cv::Mat gray_objimg;
		cv::cvtColor(objimg, gray_objimg, CV_BGR2GRAY);
		for (int i = 0; i<ObjDescrip->GetNumberOfPoints(); i++)
		{
			double p[3];
			ObjDescrip->GetPoint(i, p);
			gray_objimg.at<uchar>(p[1], p[0]) = p[2] * 10000;
		}
		cv::normalize(gray_objimg, gray_objimg, 0, 255, cv::NORM_MINMAX, CV_8UC1);

		// Report and Debug
		/*string name;
		name.append("Report/object_");
		QString num = QString::number(reportCount);
		name.append(num.toUtf8().constData());
		num = QString::number(RotaX);
		name.append(num.toUtf8().constData());
		name.append("_");
		num = QString::number(RotaZ);
		name.append(num.toUtf8().constData());
		name.append(".jpg");
		cv::imwrite(name, gray_objimg);*/

		cv::Point minloc, maxloc;
		double minval, maxval;
		cv::Mat ftmp;// = cv::Mat(gray_modelimg.rows - gray_objimg.rows + 1, gray_modelimg.cols - gray_objimg.cols + 1, CV_32FC1 );
		cv::matchTemplate(gray_modelimg, gray_objimg, ftmp, CV_TM_CCOEFF_NORMED);//CV_TM_CCORR_NORMED CV_TM_CCOEFF_NORMED
		cv::minMaxLoc(ftmp, &minval, &maxval, &minloc, &maxloc, cv::Mat());
		if (NCCmax < maxval)
		{
			NCCmax = maxval;
			BestRota = RotaZ;
			StartX = maxloc.x;
			StartY = maxloc.y;
		}
		// Report and Debug
		//offlineFile << "NCC " << reportCount << "_" << RotaX << "_" << RotaZ << ": " << maxval << "\n";
	}
}

void cinspection::VTK_OBB_Descriptor(vtkSmartPointer<vtkPoints> Input, vtkSmartPointer<vtkPoints> &Descriptor, int numOfSubOBBX, int numOfSubOBBY, int numOfModel)
{
	int NumOfPoints = Input->GetNumberOfPoints();
	vtkSmartPointer<vtkPoints>  vtk_TransformedPoints = vtkSmartPointer<vtkPoints>::New();
	VTKpoint_To_VTKpoint(Input, vtk_TransformedPoints, DBL_MAX);
	int NumOfSubOBB = numOfSubOBBX*numOfSubOBBY;

	// Find Max Min X Y Z
	double MaxX = -1 * DBL_MAX;
	double MinX = DBL_MAX;
	double MaxY = -1 * DBL_MAX;
	double MinY = DBL_MAX;
	for (int i = 0; i < vtk_TransformedPoints->GetNumberOfPoints(); i++)
	{
		double p[3];
		vtk_TransformedPoints->GetPoint(i, p);
		if (p[0] > MaxX) MaxX = p[0];
		if (p[0] < MinX) MinX = p[0];
		if (p[1] > MaxY) MaxY = p[1];
		if (p[1] < MinY) MinY = p[1];
	}
	// Initializing for descriptor
	for (int OBBindex = 0; OBBindex < NumOfSubOBB; OBBindex++)
	{
		Descriptor->InsertPoint(OBBindex, 0, 0, 0);
	}
	for (int m = 0; m < vtk_TransformedPoints->GetNumberOfPoints(); m++)
	{
		double p[3];
		vtk_TransformedPoints->GetPoint(m, p);
		int deltaX = (MaxX - MinX) / numOfSubOBBX;
		int deltaY = (MaxY - MinY) / numOfSubOBBY;
		int index_X = (p[0] - MinX) / deltaX;
		int index_Y = (p[1] - MinY) / deltaY;
		int OBB_index = index_Y*numOfSubOBBX + index_X;
		double p_Descriptor[3];
		Descriptor->GetPoint(OBB_index, p_Descriptor);
		Descriptor->InsertPoint(OBB_index, index_X, index_Y, p_Descriptor[2] + 1);
	}

	if (vtk_TransformedPoints->GetNumberOfPoints() != 0) vtk_TransformedPoints = vtkSmartPointer<vtkPoints>::New();
	VTKpoint_To_VTKpoint(Descriptor, vtk_TransformedPoints, DBL_MAX);
	if (Descriptor->GetNumberOfPoints() != 0) Descriptor = vtkSmartPointer<vtkPoints>::New();
	for (int SubOBB = 0; SubOBB < NumOfSubOBB; SubOBB++)
	{
		double p_Descriptor[3];
		vtk_TransformedPoints->GetPoint(SubOBB, p_Descriptor);
		double Normalized_Descriptor = (double)p_Descriptor[2] / numOfModel;
		Descriptor->InsertNextPoint(p_Descriptor[0], p_Descriptor[1], Normalized_Descriptor);
	}
}

void cinspection::VTK_ExtractVitualPoints(vtkSmartPointer<vtkPoints> &vtk_PointClouds, vtkSmartPointer<vtkPoints> &ExtractedCloud,
	 vtkSmartPointer<vtkPoints> VirtualPoints, int NumOfSamplesX, double ratioZ, double startX, double NumOfSubOBBX, double startY, double NumOfSubOBBY, double NumofOBB_model[3])
{
	double MaxX = -1 * DBL_MAX;
	double MinX = DBL_MAX;
	double MaxY = -1 * DBL_MAX;
	double MinY = DBL_MAX;
	double MaxZ = -1 * DBL_MAX;
	double MinZ = DBL_MAX;

	for (int i = 0; i<vtk_PointClouds->GetNumberOfPoints(); i++)
	{
		double p[3];
		vtk_PointClouds->GetPoint(i, p);
		if (p[0]>MaxX) MaxX = p[0];
		if (p[0]<MinX) MinX = p[0];
		if (p[1]>MaxY) MaxY = p[1];
		if (p[1]<MinY) MinY = p[1];
		if (p[2]>MaxZ) MaxZ = p[2];
		if (p[2]<MinZ) MinZ = p[2];
	}

	int NumOfSamplesY = NumOfSamplesX*(double)(MaxY - MinY) / (MaxX - MinX);
	int NumOfSamples = NumOfSamplesX*NumOfSamplesY;

	vtkSmartPointer<vtkPoints>  vtk_MinMaxZValue = vtkSmartPointer<vtkPoints>::New();
	for (int OBBindex = 0; OBBindex < NumOfSamples; OBBindex++)
	{
		vtk_MinMaxZValue->InsertPoint(OBBindex, MaxZ, MinZ, 0);
	}
	// Find max of Z at each SubOBB
	for (int m = 0; m < vtk_PointClouds->GetNumberOfPoints(); m++)
	{
		double p[3];
		vtk_PointClouds->GetPoint(m, p);
		double deltaX = (double)(MaxX - MinX) / NumOfSamplesX;
		double deltaY = (double)(MaxY - MinY) / NumOfSamplesY;
		int index_X = (p[0] - MinX) / deltaX;
		int index_Y = (p[1] - MinY) / deltaY;
		int OBB_index = index_Y*NumOfSamplesX + index_X;
		double q[3];
		vtk_MinMaxZValue->GetPoint(OBB_index, q);
		if (p[2]>q[1]) vtk_MinMaxZValue->InsertPoint(OBB_index, q[0], p[2], 0);
		if (p[2]<q[0]) vtk_MinMaxZValue->InsertPoint(OBB_index, p[2], q[1], 0);
	}
	// Extract Points captured by Virtual Camera
	for (int m = 0; m<vtk_PointClouds->GetNumberOfPoints(); m++)
	{
		double p[3];
		vtk_PointClouds->GetPoint(m, p);
		double deltaX = (double)(MaxX - MinX) / NumOfSamplesX;
		double deltaY = (double)(MaxY - MinY) / NumOfSamplesY;
		int index_X = (p[0] - MinX) / deltaX;
		int index_Y = (p[1] - MinY) / deltaY;
		int OBB_index = index_Y*NumOfSamplesX + index_X;
		double q[3];
		vtk_MinMaxZValue->GetPoint(OBB_index, q);
		if (p[2] - q[0] > (q[1] - q[0])*ratioZ) VirtualPoints->InsertNextPoint(p);
	}
	// Extract Points similar with measured object part

	vtkSmartPointer<vtkPoints>  vtk_TransformedPoints = vtkSmartPointer<vtkPoints>::New();
	int NumOfSubOBB = NumofOBB_model[0] * NumofOBB_model[1];

	double center[3];
	VTKCenterOfMass(VirtualPoints, center);
	VTK_Translation(VirtualPoints, -center[0], -center[1], -center[2], 0, 0, 0, vtk_TransformedPoints);
	VTK_Translation(vtk_PointClouds, -center[0], -center[1], -center[2], 0, 0, 0, vtk_PointClouds);

	// Find Max Min X Y Z
	MaxX = -1 * DBL_MAX;
	MinX = DBL_MAX;
	MaxY = -1 * DBL_MAX;
	MinY = DBL_MAX;
	for (int i = 0; i<vtk_TransformedPoints->GetNumberOfPoints(); i++)
	{
		double p[3];
		vtk_TransformedPoints->GetPoint(i, p);
		if (p[0]>MaxX) MaxX = p[0];
		if (p[0]<MinX) MinX = p[0];
		if (p[1]>MaxY) MaxY = p[1];
		if (p[1]<MinY) MinY = p[1];
	}
	for (int m = 0; m<vtk_TransformedPoints->GetNumberOfPoints(); m++)
	{
		double p[3];
		vtk_TransformedPoints->GetPoint(m, p);
		double deltaX = (MaxX - MinX) / NumofOBB_model[0];
		double deltaY = (MaxY - MinY) / NumofOBB_model[1];
		int index_X = (p[0] - MinX) / deltaX;
		int index_Y = (p[1] - MinY) / deltaY;
		if (index_X >= startX && index_X < startX + NumOfSubOBBX && index_Y >= startY && index_Y < startY + NumOfSubOBBY)
			ExtractedCloud->InsertNextPoint(p);
	}
}

void cinspection::VTK_ICP_Getmatrix(vtkSmartPointer<vtkPoints> source_points, vtkSmartPointer<vtkPoints> target_points, vtkSmartPointer<vtkIterativeClosestPointTransform> &icp, int i)
{
	vtkSmartPointer<vtkPolyData> source = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> target = vtkSmartPointer<vtkPolyData>::New();

	vtkSmartPointer<vtkPolyData> source1 = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> target1 = vtkSmartPointer<vtkPolyData>::New();

	source1->SetPoints(source_points);
	target1->SetPoints(target_points);

	vtkSmartPointer<vtkVertexGlyphFilter> glyphFilterSource = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	glyphFilterSource->SetInputData(source1);
	glyphFilterSource->Update();

	source->ShallowCopy(glyphFilterSource->GetOutput());

	vtkSmartPointer<vtkVertexGlyphFilter> glyphFilterTarget = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	glyphFilterTarget->SetInputData(target1);
	glyphFilterTarget->Update();

	target->ShallowCopy(glyphFilterTarget->GetOutput());

	// Setup ICP transform
	/*vtkSmartPointer<vtkIterativeClosestPointTransform> icp = vtkSmartPointer<vtkIterativeClosestPointTransform>::New();*/

	icp->SetSource(source);
	icp->SetTarget(target);
	icp->GetLandmarkTransform()->SetModeToRigidBody();
	icp->SetMaximumNumberOfIterations(i);
	//icp->StartByMatchingCentroidsOn();
	icp->Modified();
	icp->Update();

	//vtkSmartPointer<vtkMatrix4x4> m = vtkSmartPointer<vtkMatrix4x4>::New();
	//m = icp->GetMatrix();

	/*vtkSmartPointer<vtkTransformFilter> finalTransformFilter = vtkSmartPointer<vtkTransformFilter>::New();

	#if VTK_MAJOR_VERSION <= 5
	finalTransformFilter->SetInput(source);
	#else
	finalTransformFilter->SetInputData(source);
	#endif
	finalTransformFilter->SetTransform(icp);
	finalTransformFilter->Update();
	transformed_points = finalTransformFilter->GetOutput()->GetPoints();*/
}

void cinspection::VTK_ICP_Apply(vtkSmartPointer<vtkPoints> source_points, vtkSmartPointer<vtkPoints> &transformed_points, vtkSmartPointer<vtkIterativeClosestPointTransform> icp)
{
	vtkSmartPointer<vtkPolyData> source = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> source1 = vtkSmartPointer<vtkPolyData>::New();
	source1->SetPoints(source_points);

	vtkSmartPointer<vtkVertexGlyphFilter> glyphFilterSource = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	glyphFilterSource->SetInputData(source1);
	glyphFilterSource->Update();
	source->ShallowCopy(glyphFilterSource->GetOutput());

	vtkSmartPointer<vtkTransformFilter> finalTransformFilter = vtkSmartPointer<vtkTransformFilter>::New();
#if VTK_MAJOR_VERSION <= 5
	finalTransformFilter->SetInput(source);
#else
	finalTransformFilter->SetInputData(source);
#endif
	finalTransformFilter->SetTransform(icp);
	finalTransformFilter->Update();
	if (transformed_points->GetNumberOfPoints() != 0) transformed_points = vtkSmartPointer<vtkPoints>::New();
	transformed_points = finalTransformFilter->GetOutput()->GetPoints();
}

void cinspection::VTK_MeanDistance_CloudToCloud(vtkSmartPointer<vtkPoints> source, vtkSmartPointer<vtkPoints> target, double &meandist)
{
	int PointNum_query = (int)source->GetNumberOfPoints();
	int PointNum_data = (int)target->GetNumberOfPoints();
	meandist = 0;
	//--------------------------------------------------
	// KNN METHOD for finding neighbors
	//--------------------------------------------------	

	int knn = 1;
	int dim = 3;						// dimension
	double	eps = 0;						// error bound
	int maxPts_query = PointNum_query;					// maximum number of data points
	int maxPts_data = PointNum_data;					// maximum number of data points

	int				ndataPts;					// actual number of data points
	int				nqueryPts;					// actual number of data points
	ANNpointArray	dataPts;				// data points
	ANNpointArray	queryPts;
	ANNpoint		queryPt;				// query point
	ANNidxArray		nnIdx;					// near neighbor indices
	ANNdistArray	dists;					// near neighbor distances
	ANNkd_tree*		kdTree;					// search structure

	queryPt = annAllocPt(dim);				// allocate query point
	dataPts = annAllocPts(maxPts_data, dim);	    // allocate data points
	queryPts = annAllocPts(maxPts_query, dim);	    // allocate data points
	nnIdx = new ANNidx[knn];				// allocate near neigh indices
	dists = new ANNdist[knn];				// allocate near neighbor dists

	ndataPts = 0;

	while (ndataPts < maxPts_data) {
		double p[3];
		target->GetPoint(ndataPts, p);
		dataPts[ndataPts][0] = p[0];
		dataPts[ndataPts][1] = p[1];
		dataPts[ndataPts][2] = p[2];
		ndataPts++;
	}

	nqueryPts = 0;
	while (nqueryPts < maxPts_query) {
		double q[3];
		source->GetPoint(nqueryPts, q);
		queryPts[nqueryPts][0] = q[0];
		queryPts[nqueryPts][1] = q[1];
		queryPts[nqueryPts][2] = q[2];
		nqueryPts++;
	}

	kdTree = new ANNkd_tree(	// build search structure
		dataPts,				// the data points
		ndataPts,				// number of points
		dim);					// dimension of space

	for (int searchP = 0; searchP < PointNum_query; searchP++)
	{
		queryPt = queryPts[searchP];	// Query the first point
		kdTree->annkSearch(			// search
			queryPt,				// query point
			knn,					// number of near neighbors
			nnIdx,					// nearest neighbors (returned)
			dists,					// squared distance (returned)
			eps);					// error bound
		meandist += sqrt(dists[0]);
	}
	meandist = (double)meandist / PointNum_query;

	delete[] nnIdx;							// clean things up
	delete[] dists;
	delete kdTree;
	annClose();									// done with ANN
}

unsigned char cinspection::VTK_OBB_Recognition(pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output, double toler, double pass)
{
	reportCount = 0;
	ofstream onlineFile;
	onlineFile.open("Report/Report_online.txt");
	QElapsedTimer timer;
	
	vtkSmartPointer<vtkPoints>  ModelDescriptor = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints>  ModelCloud = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints>  ObjectCloud = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> VirtualPoints = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> VTKBestView = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> VTKBestRota = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> VTKBestNCC = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> VTKnumOfSubOBB = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> VTKObjCloud = vtkSmartPointer<vtkPoints>::New();

	vtkSmartPointer<vtkMatrix4x4> m_obj = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4> matri = vtkSmartPointer<vtkMatrix4x4>::New();
	double center_obj[3];
	double center[3];
	double corner_OBB[3], max_OBB[3], mid_OBB[3], min_OBB[3];
	double NCCBest = -1 * DBL_MAX;

	// Get Object Cloud from segmentation
	vtkSmartPointer<vtkPoints> vtkPointCloud = vtkSmartPointer<vtkPoints>::New();
	PCL_TO_VTK(input, vtkPointCloud);
	PCL_TO_VTK(input, VTKObjCloud);
	VTK_OBB_Calculation(vtkPointCloud, corner_OBB, max_OBB, mid_OBB, min_OBB);
	double lengthOBB_object[3];
	lengthOBB_object[0] = sqrt(max_OBB[0] * max_OBB[0] + max_OBB[1] * max_OBB[1] + max_OBB[2] * max_OBB[2]); // length of max
	lengthOBB_object[1] = sqrt(mid_OBB[0] * mid_OBB[0] + mid_OBB[1] * mid_OBB[1] + mid_OBB[2] * mid_OBB[2]); // length of mid
	lengthOBB_object[2] = sqrt(min_OBB[0] * min_OBB[0] + min_OBB[1] * min_OBB[1] + min_OBB[2] * min_OBB[2]); // length of min

	//Transform object cloud to origin and real coordinate --> 2 purposes: easy to compute descriptor; easy to align its OBB with model's OBB
	if (ObjectCloud->GetNumberOfPoints() != 0) ObjectCloud = vtkSmartPointer<vtkPoints>::New();
	Eigen::Matrix4f matr1 = Eigen::Matrix4f::Identity();
	VTK_OBB_Calculation(vtkPointCloud, corner_OBB, max_OBB, mid_OBB, min_OBB);
	VTK_RealcoordinateToOBBcoordinateXY(vtkPointCloud, ObjectCloud, corner_OBB, max_OBB, mid_OBB, min_OBB);
	matr1 = matr.replicate(4, 4);
	VTKCenterOfMass(ObjectCloud, center_obj);
	VTK_Translation(ObjectCloud, -center_obj[0], -center_obj[1], -center_obj[2], 0, 0, 0, ObjectCloud);

	int countLine = 0;
	// Read Points from model
	ReadVTKPointCloud_txt(ModelCloud);
	// Read Descriptor of model
	ReadVTKDescriptor(countLine, countLine, ModelDescriptor);
	// Compute mean distance every point to its closest point

	double modelResol, objResol;
	timer.start();
	/*VTK_kNearestMeanDistance(ObjectCloud, 30, objResol);
	VTK_kNearestMeanDistance(ModelCloud, 30, modelResol);*/
	if(!kNearestMeanDist_PCL(input, 30, objResol)) return 3;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Model_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
	VTK_TO_PCL(ModelCloud, *Model_pcl);
	if(!kNearestMeanDist_PCL(*Model_pcl, 30, modelResol)) return 3;
	onlineFile << "VTK_kNearestMeanDistance: " << timer.elapsed() << " ms" << "\n";

	double adaptResol = ModelCloud->GetNumberOfPoints()*((double)modelResol / objResol);

	while (ModelDescriptor->GetNumberOfPoints() != 0)
	{
		double p_ModelOrientation[3];
		ModelDescriptor->GetPoint(0, p_ModelOrientation); //get Rx Ry Rz of model from txt file
		double lengthOBB_model[3];
		ModelDescriptor->GetPoint(1, lengthOBB_model); //get length of OBB of model with Max Mid Min from txt file
		double NumOfSubOBB_model[3];
		ModelDescriptor->GetPoint(2, NumOfSubOBB_model); //get Number of SubOBBs of model  along OX and OY from txt file

		if (abs(lengthOBB_object[0] - lengthOBB_model[0]) < toler*lengthOBB_model[0] &
			abs(lengthOBB_object[1] - lengthOBB_model[1]) < toler*lengthOBB_model[1])
		{
			//Estimate number of subOBBs of measured point cloud based on length of its OBB and number of subOBBs of OBB of model
			double NumOfSubOBBX = NumOfSubOBB_model[0] * (double)(lengthOBB_object[0] / lengthOBB_model[0]) + 0.5;//0.5 means if 1.7->2, if 1.3->1
			double NumOfSubOBBY = NumOfSubOBB_model[1] * (double)(lengthOBB_object[1] / lengthOBB_model[1]) + 0.5;//0.5 means if 1.7->2, if 1.3->1

			if (NumOfSubOBBX > NumOfSubOBB_model[0]) NumOfSubOBBX = NumOfSubOBB_model[0];
			if (NumOfSubOBBY > NumOfSubOBB_model[1]) NumOfSubOBBY = NumOfSubOBB_model[1];

			// Because start position for computing descriptor is random at 4 lower corners of OBB, we need to
			// rotate object cloud to test with different position to choose the best one.
			double NCCmax = -1;
			double BestRota = -1 * DBL_MAX;
			double StartX = -1;
			double StartY = -1;
			int countView = 0;
			timer.start();
			VTK_matching_scence_model(ObjectCloud, ModelDescriptor, lengthOBB_model[1], lengthOBB_object[0], adaptResol, NumOfSubOBBX,
				NumOfSubOBBY, NumOfSubOBB_model[0], NumOfSubOBB_model[1], StartX, StartY, BestRota, NCCmax, countView);
			onlineFile << "VTK_matching_scence_model: " << timer.elapsed() << " ms" << "\n";

			VTKBestRota->InsertNextPoint(StartX, StartY, BestRota);
			VTKBestView->InsertNextPoint(p_ModelOrientation);
			VTKBestNCC->InsertNextPoint(NumOfSubOBBX, NumOfSubOBBY, NCCmax);
			VTKnumOfSubOBB->InsertNextPoint(NumOfSubOBB_model);
			if (NCCmax >  NCCBest) NCCBest = NCCmax;
		}

		// Start new view from virtual camera
		countLine++;
		if (ModelDescriptor->GetNumberOfPoints() != 0) ModelDescriptor = vtkSmartPointer<vtkPoints>::New();
		// Read Descriptor of model
		ReadVTKDescriptor(countLine, countLine, ModelDescriptor);
	}
	if (VTKBestNCC->GetNumberOfPoints() == 0)
	{
		onlineFile.close();
		return 1;
	}

	double maxval = -1 * DBL_MAX;
	int maxIndex;
	if (vtkPointCloud->GetNumberOfPoints() != 0) vtkPointCloud = vtkSmartPointer<vtkPoints>::New();
	VTK_Translation(ObjectCloud, 0, 0, 0, 0, 0, 0, vtkPointCloud);
	vtkSmartPointer<vtkPoints> vtkCloud_Buffer = vtkSmartPointer<vtkPoints>::New();
	VTK_Translation(vtkPointCloud, 0, 0, 0, 0, 0, 0, vtkCloud_Buffer);
	while (true)
	{
		maxval = -1 * DBL_MAX;
		for (int numOfBestNCC = 0; numOfBestNCC < VTKBestNCC->GetNumberOfPoints(); numOfBestNCC++)
		{
			double p_BestNCC[3];
			VTKBestNCC->GetPoint(numOfBestNCC, p_BestNCC);
			if (p_BestNCC[2] > maxval)
			{
				maxval = p_BestNCC[2];
				maxIndex = numOfBestNCC;
			}
		}
		onlineFile << "Here 1: " << "\n";
		double NCC_Ratio = 0.5;
		if (abs(maxval) < abs(NCC_Ratio * NCCBest) || maxval == -1)
		{
			onlineFile << "Here final: " << "\n";
			onlineFile.close();
			return 2;
		}
		else
		{
			onlineFile << "Here 2: " << "\n";
			double p_BestView[3];
			VTKBestView->GetPoint(maxIndex, p_BestView);
			double p_BestRota[3];
			VTKBestRota->GetPoint(maxIndex, p_BestRota);
			double p_numOfSubOBB[3];
			VTKnumOfSubOBB->GetPoint(maxIndex, p_numOfSubOBB);
			double p_BestNCC[3];
			VTKBestNCC->GetPoint(maxIndex, p_BestNCC);
			VTKBestNCC->InsertPoint(maxIndex, 0, 0, -1);

			// Read parameters of 3D virtual camera
			int NumOfSamplesX;
			double ZThresh;
			ReaVirtualCameraParameters(NumOfSamplesX, ZThresh);

			// Extract points on model similar with measured points
			vtkSmartPointer<vtkPoints> ModelTran = vtkSmartPointer<vtkPoints>::New();
			vtkSmartPointer<vtkPoints> ExtractedCloud = vtkSmartPointer<vtkPoints>::New();
			if (VirtualPoints->GetNumberOfPoints() != 0) VirtualPoints = vtkSmartPointer<vtkPoints>::New();
			VTK_Translation(ModelCloud, 0, 0, 0, p_BestView[0], p_BestView[1], p_BestView[2], ModelTran);
			VTK_ExtractVitualPoints(ModelTran, ExtractedCloud, VirtualPoints, NumOfSamplesX, ZThresh, p_BestRota[0],
				p_BestNCC[0], p_BestRota[1], p_BestNCC[1], p_numOfSubOBB);

			VTK_OBB_Calculation(ExtractedCloud, corner_OBB, max_OBB, mid_OBB, min_OBB);
			VTK_RealcoordinateToOBBcoordinateXY(ExtractedCloud, ExtractedCloud, corner_OBB, max_OBB, mid_OBB, min_OBB);
			Eigen::Matrix4f matr2 = Eigen::Matrix4f::Identity();
			matr2 = matr.replicate(4, 4);

			if (Model_pcl->size()) Model_pcl->clear();
			VTK_TO_PCL(ModelTran, *Model_pcl);
			pcl::transformPointCloud(*Model_pcl, *Model_pcl, matr2);
			ModelTran->Reset();
			PCL_TO_VTK(*Model_pcl, ModelTran);

			//if (output.size()) output.clear();
			//VTK_TO_PCL(vtkPointCloud, output);
			//VTK_TO_PCL(ExtractedCloud, output);
			//return 1;

			VTKCenterOfMass(ExtractedCloud, center);
			// Coarse Registration Align OBB of extracted model point cloud with OBB of measured object point cloud  
			VTK_Translation(ExtractedCloud, -center[0], -center[1], -center[2], 0, 0, 0, ExtractedCloud);
			VTK_Translation(ModelTran, -center[0], -center[1], -center[2], 0, 0, 0, ModelTran);

			vtkSmartPointer<vtkPoints> vtkModel_Buffer = vtkSmartPointer<vtkPoints>::New();
			VTK_Translation(ModelTran, 0, 0, 0, 0, 0, 0, vtkModel_Buffer);
			// Because of kz = 1, object might be symmtrical along axis Ox or Oy, so need be checked by rotX 180 and rotY 180
			// If kx approximates ky, axis Ox and axis Oy will be ambiguous, so need be checked by rotZ by increment 90
			double increaZ = 90;
			if (abs(lengthOBB_object[0] - lengthOBB_object[1]) > 0.05*lengthOBB_object[0]) increaZ = 360;
			for (int rotZ = 0; rotZ < 360; rotZ += increaZ)
			for (int rotY = 0; rotY < 360; rotY +=180)
			for (int rotX = 0; rotX < 360; rotX += 180)
			{
				VTK_Translation(vtkCloud_Buffer, 0, 0, 0, rotX, rotY, rotZ, vtkPointCloud);
				VTK_Translation(vtkModel_Buffer, 0, 0, 0, 0, 0, 0, ModelTran);
				
				// Refine registration by ICP
				vtkSmartPointer<vtkIterativeClosestPointTransform> icp = vtkSmartPointer<vtkIterativeClosestPointTransform>::New();
				timer.start();
				VTK_ICP_Getmatrix(vtkPointCloud, ModelTran, icp, 100); //Can be considered to replace ModelTran by ExtractedCloud for reducing time
				onlineFile << "ICP: " << timer.elapsed() << " ms" << "\n";
				onlineFile << "NCC: " << maxval << "\n";
				// Transform whole model to measured point cloud
				vtkSmartPointer<vtkMatrix4x4> Inverse_m = vtkSmartPointer<vtkMatrix4x4>::New();
				vtkMatrix4x4::Invert(icp->GetMatrix(), Inverse_m);
				for (int i = 0; i<ModelTran->GetNumberOfPoints(); i++)
				{
					double p[3];
					ModelTran->GetPoint(i, p);
					double q[4] = { p[0], p[1], p[2], 1 };
					double q1[4];
					q1[0] = Inverse_m->MultiplyDoublePoint(q)[0];
					q1[1] = Inverse_m->MultiplyDoublePoint(q)[1];
					q1[2] = Inverse_m->MultiplyDoublePoint(q)[2];
					ModelTran->InsertPoint(i, q1[0], q1[1], q1[2]);
				}
							
				VTK_Translation(ModelTran, 0, 0, 0, -rotX, -rotY, -rotZ, ModelTran);
				//VTK_Translation(ModelTran, 0, 0, 0, 0, 0, -p_BestRota[2], ModelTran);
				VTK_Translation(ModelTran, center_obj[0], center_obj[1], center_obj[2], 0, 0, 0, ModelTran);

				Eigen::Matrix4f matr1_inver = Eigen::Matrix4f::Identity();

				matr1_inver = matr1.inverse();
				if (Model_pcl->size()) Model_pcl->clear();
				VTK_TO_PCL(ModelTran, *Model_pcl);
				onlineFile << "Number of Points Model_pcl: " << Model_pcl->size() << "\n";
				pcl::transformPointCloud(*Model_pcl, *Model_pcl, matr1_inver);
				onlineFile << "Number of Points Model_pcl: " << Model_pcl->size() << "\n";
				ModelTran->Reset();
				PCL_TO_VTK(*Model_pcl, ModelTran);

				/*if (output.size()) output.clear();
				VTK_TO_PCL(ModelTran, output);
				VTK_TO_PCL(vtkPointCloud, output);*/
				//return 1;
				
				double meandist;
				timer.start();
				//VTK_MeanDistance_CloudToCloud(VTKObjCloud, ModelTran, meandist);
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr ObjCloud_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr ModelTran_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
				VTK_TO_PCL(VTKObjCloud, *ObjCloud_pcl);
				VTK_TO_PCL(ModelTran, *ModelTran_pcl);
				if (!PCL_MeanDistance_CloudToCloud(*ObjCloud_pcl, *ModelTran_pcl, meandist)) continue;
				onlineFile << "PCL_MeanDistance_CloudToCloud: " << timer.elapsed() << " ms" << "\n";
				double thresh = pass;
				onlineFile.precision(8);
				onlineFile << "meandist: " << meandist << "\n";

				if (meandist < thresh)
				{
					VTK_TO_PCL(ModelTran, output);
					onlineFile.close();
					return 0;
				}
				else continue;
			}
		}
	}
	//onlineFile.close();
	//return 0;
}

// -----------------------------------------------------0--------------------------------------------------------
// PCL Functions
void cinspection::DownSampling(pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output, double ratio)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr DownsampleCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(input, *DownsampleCloud);

	// Downsample
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(DownsampleCloud);
	sor.setLeafSize(ratio, ratio, ratio);
	DownsampleCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	sor.filter(*DownsampleCloud);

	if ((int)output.size() != 0) output.clear();
	pcl::copyPointCloud(*DownsampleCloud, output);
}

void cinspection::template_Matching(pcl::PointCloud<pcl::PointXYZRGB> &searched_Descrip, pcl::PointCloud<pcl::PointXYZRGB> &templ_Descrip,
	 pcl::PointCloud<pcl::PointXYZ> &result, int numOfSubOBB_searched[3], int numOfSuOBB_temp[3])
{
	pcl::PointCloud<pcl::PointXYZ> Descrip_Temp;
	pcl::PointCloud<pcl::PointXYZ> Descrip_Sear;

	double templ_Sum = 0;
	//offlineFile << "numOfSuOBB_temp " << numOfSuOBB_temp[0] << " " << numOfSuOBB_temp[1] << "\n";
	//offlineFile << "numOfSubOBB_searched " << numOfSubOBB_searched[0] << " " << numOfSubOBB_searched[1] << "\n";

	Descrip_Temp.width = numOfSuOBB_temp[0];
	Descrip_Temp.height = numOfSuOBB_temp[1];
	Descrip_Temp.points.resize(Descrip_Temp.width * Descrip_Temp.height);
	//offlineFile << "\n";
	for (int j = 0; j < Descrip_Temp.height; j++)
	for (int i = 0; i < Descrip_Temp.width; i++)
	{
		int OBB_index = j*numOfSuOBB_temp[0] + i;
		Descrip_Temp(i, j).x = templ_Descrip.points[OBB_index].x / searched_Descrip.size();
		Descrip_Temp(i, j).y = templ_Descrip.points[OBB_index].y / searched_Descrip.size();
		Descrip_Temp(i, j).z = templ_Descrip.points[OBB_index].z / searched_Descrip.size();
		templ_Sum += Descrip_Temp(i, j).z * Descrip_Temp(i, j).z;
		//offlineFile << Descrip_Temp(i, j).z << " ";
	}
	//offlineFile << "\n";
	Descrip_Sear.width = numOfSubOBB_searched[0];
	Descrip_Sear.height = numOfSubOBB_searched[1];
	Descrip_Sear.points.resize(Descrip_Sear.width * Descrip_Sear.height);
	//offlineFile << "\n";
	for (int j = 0; j < Descrip_Sear.height; j++)
	for (int i = 0; i < Descrip_Sear.width; i++)
	{
		int OBB_index = j*numOfSubOBB_searched[0] + i + 1;
		Descrip_Sear(i, j).x = searched_Descrip.points[OBB_index].x / searched_Descrip.size();
		Descrip_Sear(i, j).y = searched_Descrip.points[OBB_index].y / searched_Descrip.size();
		Descrip_Sear(i, j).z = searched_Descrip.points[OBB_index].z / searched_Descrip.size();
		//offlineFile << Descrip_Sear(i, j).z << " ";
	}
	//offlineFile << "\n";
	//offlineFile << "\n";
	int MaxJ = Descrip_Sear.height - Descrip_Temp.height;
	int MaxI = Descrip_Sear.width - Descrip_Temp.width;
	pcl::PointXYZ point_NCCMax;
	point_NCCMax.z = DBL_MAX;
	
	for (int j = 0; j <= MaxJ; j++)
	for (int i = 0; i <= MaxI; i++)
	{
		double searched_Sum = 0;
		double TS = 0;
		double MaxK = j + Descrip_Temp.height;
		double MaxL = i + Descrip_Temp.width;
		for (int k = j; k < MaxK; k++)
		for (int l = i; l < MaxL; l++)
		{
			searched_Sum += Descrip_Sear(l, k).z * Descrip_Sear(l, k).z;
			TS += Descrip_Sear(l, k).z * Descrip_Temp(l - i, k - j).z;
		}
		pcl::PointXYZ point;
		point.x = i;
		point.y = j;
		point.z = TS / sqrt(searched_Sum * templ_Sum); //method=CV_TM_CCORR_NORMED
		//offlineFile << "point.z:" << point.z << "\n";
		//offlineFile << "TS: " << TS << "\n";
		//offlineFile << "searched_Sum: " << searched_Sum << "\n";
		//offlineFile << "templ_Sum: " << templ_Sum << "\n";
		result.push_back(point);
		if (point_NCCMax.z > point.z) pcl::copyPoint(point, point_NCCMax);
	}
	//offlineFile << "point_NCCMax:" << point_NCCMax.z << "\n";
	result.push_back(point_NCCMax);
}

void cinspection::template_Matching_Nonseg(pcl::PointCloud<pcl::PointXYZRGB> &searched_Descrip, pcl::PointCloud<pcl::PointXYZRGB> &templ_Descrip,
	pcl::PointCloud<pcl::PointXYZ> &result, int numOfSubOBB_searched[3], int numOfSuOBB_temp[3])
{
	pcl::PointCloud<pcl::PointXYZ> Descrip_Temp;
	pcl::PointCloud<pcl::PointXYZ> Descrip_Sear;

	double templ_Sum = 0;
	//offlineFile << "numOfSuOBB_temp " << numOfSuOBB_temp[0] << " " << numOfSuOBB_temp[1] << "\n";
	//offlineFile << "numOfSubOBB_searched " << numOfSubOBB_searched[0] << " " << numOfSubOBB_searched[1] << "\n";

	Descrip_Temp.width = numOfSuOBB_temp[0];
	Descrip_Temp.height = numOfSuOBB_temp[1];
	Descrip_Temp.points.resize(Descrip_Temp.width * Descrip_Temp.height);
	//offlineFile << "\n";
	for (int j = 0; j < Descrip_Temp.height; j++)
	for (int i = 0; i < Descrip_Temp.width; i++)
	{
		int OBB_index = j*numOfSuOBB_temp[0] + i;
		Descrip_Temp(i, j).x = templ_Descrip.points[OBB_index].x / searched_Descrip.size();
		Descrip_Temp(i, j).y = templ_Descrip.points[OBB_index].y / searched_Descrip.size();
		Descrip_Temp(i, j).z = templ_Descrip.points[OBB_index].z / searched_Descrip.size();
		templ_Sum += Descrip_Temp(i, j).z * Descrip_Temp(i, j).z;
		//offlineFile << Descrip_Temp(i, j).z << " ";
	}
	//offlineFile << "\n";
	Descrip_Sear.width = numOfSubOBB_searched[0];
	Descrip_Sear.height = numOfSubOBB_searched[1];
	Descrip_Sear.points.resize(Descrip_Sear.width * Descrip_Sear.height);
	//offlineFile << "\n";
	for (int j = 0; j < Descrip_Sear.height; j++)
	for (int i = 0; i < Descrip_Sear.width; i++)
	{
		int OBB_index = j*numOfSubOBB_searched[0] + i;
		Descrip_Sear(i, j).x = searched_Descrip.points[OBB_index].x / searched_Descrip.size();
		Descrip_Sear(i, j).y = searched_Descrip.points[OBB_index].y / searched_Descrip.size();
		Descrip_Sear(i, j).z = searched_Descrip.points[OBB_index].z / searched_Descrip.size();
		//offlineFile << Descrip_Sear(i, j).z << " ";
	}
	//offlineFile << "\n";
	//offlineFile << "\n";
	int MaxJ = Descrip_Sear.height - Descrip_Temp.height;
	int MaxI = Descrip_Sear.width - Descrip_Temp.width;
	pcl::PointXYZ point_NCCMax;
	point_NCCMax.z = -1 * DBL_MAX;
	//offlineFile << "MaxJ MaxI " << MaxJ << " " << MaxI << "\n";
	for (int j = 0; j <= MaxJ; j++)
	for (int i = 0; i <= MaxI; i++)
	{
		double searched_Sum = 0;
		double TS = 0;
		double MaxK = j + Descrip_Temp.height;
		double MaxL = i + Descrip_Temp.width;
		for (int k = j; k < MaxK; k++)
		for (int l = i; l < MaxL; l++)
		{
			if (Descrip_Temp(l - i, k - j).z == 0 & Descrip_Sear(l, k).z != 0) continue;
			searched_Sum += Descrip_Sear(l, k).z * Descrip_Sear(l, k).z;
			TS += Descrip_Sear(l, k).z * Descrip_Temp(l - i, k - j).z;
		}
		pcl::PointXYZ point;
		point.x = i;
		point.y = j;
		point.z = TS / sqrt(searched_Sum * templ_Sum); //method=CV_TM_CCORR_NORMED
		result.push_back(point);
		if (point_NCCMax.z < point.z) pcl::copyPoint(point, point_NCCMax);
	}
	//offlineFile << "point_NCCMax:" << point_NCCMax.x << " " << point_NCCMax.y << " " << point_NCCMax.z << "\n";
	result.push_back(point_NCCMax);
}

void cinspection::Euclidean_Cluster_Extraction(pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &cloud_NoiseRemoval,
	 pcl::PointCloud<pcl::PointXYZRGB> &Segmented_Index, pcl::PointCloud<pcl::PointXYZRGB> &features, double tolerance, int minpoints)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr SegmentedCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(input, *SegmentedCloud);

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud(SegmentedCloud);
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(*cloud_filtered);

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(tolerance); // 2cm
	ec.setMinClusterSize(minpoints); //100
	ec.setMaxClusterSize(99000000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);

	int j = 0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_NoiseRemoved(new pcl::PointCloud<pcl::PointXYZ>);
	if ((int)Segmented_Index.size() != 0) Segmented_Index.clear();
	if ((int)features.size() != 0) features.clear();
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointXYZRGB total;
		total.x = 0;
		total.y = 0;
		total.z = 0;
		int numOfpoint = 0;

		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		{
			cloud_NoiseRemoved->points.push_back(cloud_filtered->points[*pit]);
			total.x += cloud_NoiseRemoved->points[cloud_NoiseRemoved->size() - 1].x;
			total.y += cloud_NoiseRemoved->points[cloud_NoiseRemoved->size() - 1].y;
			total.z += cloud_NoiseRemoved->points[cloud_NoiseRemoved->size() - 1].z;
			numOfpoint++;

			pcl::PointXYZRGB point;
			point.x = j;
			point.y = 0;
			point.z = 0;
			if (j == 0) //Red	#FF0000	(255,0,0)
			{
				point.r = 255;
				point.g = 0;
				point.b = 0;
			}
			else if (j == 1) //Lime	#00FF00	(0,255,0)
			{
				point.r = 0;
				point.g = 255;
				point.b = 0;
			}
			else if (j == 2) // Blue	#0000FF	(0,0,255)
			{
				point.r = 0;
				point.g = 0;
				point.b = 255;
			}
			else if (j == 3) // Yellow	#FFFF00	(255,255,0)
			{
				point.r = 255;
				point.g = 255;
				point.b = 0;
			}
			else if (j == 4) //Cyan	#00FFFF	(0,255,255)
			{
				point.r = 0;
				point.g = 255;
				point.b = 255;
			}
			else if (j == 5) // Magenta	#FF00FF	(255,0,255)
			{
				point.r = 255;
				point.g = 0;
				point.b = 255;
			}
			else if (j == 6) // Olive	#808000	(128,128,0)
			{
				point.r = 128;
				point.g = 128;
				point.b = 0;
			}
			else if (j == 7) // Teal	#008080	(0,128,128)
			{
				point.r = 0;
				point.g = 128;
				point.b = 128;
			}
			else if (j == 8) // Purple	#800080	(128,0,128)
			{
				point.r = 128;
				point.g = 0;
				point.b = 128;
			}
			else
			{
				if (j % 2 == 0)
				{
					point.r = 255 * j / (cluster_indices.size());
					point.g = 128;
					point.b = 50;
				}
				else
				{
					point.r = 0;
					point.g = 255 * j / (cluster_indices.size());
					point.b = 128;
				}

			}
			Segmented_Index.push_back(point);
		}
		total.x = (double)total.x / numOfpoint;
		total.y = (double)total.y / numOfpoint;
		total.z = (double)total.z / numOfpoint;
		total.r = 255;
		total.g = 255;
		total.b = 255;
		features.push_back(total);
		j++;
	}
	if ((int)cloud_NoiseRemoval.size() != 0) cloud_NoiseRemoval.clear();
	pcl::copyPointCloud(*cloud_NoiseRemoved, cloud_NoiseRemoval);
}

void cinspection::colorCloudDistances(pcl::PointCloud<pcl::PointXYZRGB> &cloud_)
{
	// Find the minimum and maximum values along the selected axis
	double min, max;
	// Set an initial value
	switch (2)
	{
	case 0:  // x
		min = cloud_.points[0].x;
		max = cloud_.points[0].x;
		break;
	case 1:  // y
		min = cloud_.points[0].y;
		max = cloud_.points[0].y;
		break;
	default:  // z
		min = cloud_.points[0].z;
		max = cloud_.points[0].z;
		break;
	}

	// Search for the minimum/maximum
	for (pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it = cloud_.begin(); cloud_it != cloud_.end(); ++cloud_it)
	{
		switch (2)
		{
		case 0:  // x
			if (min > cloud_it->x)
				min = cloud_it->x;

			if (max < cloud_it->x)
				max = cloud_it->x;
			break;
		case 1:  // y
			if (min > cloud_it->y)
				min = cloud_it->y;

			if (max < cloud_it->y)
				max = cloud_it->y;
			break;
		default:  // z
			if (min > cloud_it->z)
				min = cloud_it->z;

			if (max < cloud_it->z)
				max = cloud_it->z;
			break;
		}
	}

	// Compute LUT scaling to fit the full histogram spectrum
	double lut_scale = 255.0 / (max - min);  // max is 255, min is 0

	if (min == max)  // In case the cloud is flat on the chosen direction (x,y or z)
		lut_scale = 1.0;  // Avoid rounding error in boost

	for (pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it = cloud_.begin(); cloud_it != cloud_.end(); ++cloud_it)
	{
		int value;
		switch (2)
		{
		case 0:  // x
			value = boost::math::iround((cloud_it->x - min) * lut_scale);  // Round the number to the closest integer
			break;
		case 1:  // y
			value = boost::math::iround((cloud_it->y - min) * lut_scale);
			break;
		default:  // z
			value = boost::math::iround((cloud_it->z - min) * lut_scale);
			break;
		}

		// Apply color to the cloud
		switch (4)
		{
		case 0:
			// Blue (= min) -> Red (= max)
			cloud_it->r = value;
			cloud_it->g = 0;
			cloud_it->b = 255 - value;
			break;
		case 1:
			// Green (= min) -> Magenta (= max)
			cloud_it->r = value;
			cloud_it->g = 255 - value;
			cloud_it->b = value;
			break;
		case 2:
			// White (= min) -> Red (= max)
			cloud_it->r = 255;
			cloud_it->g = 255 - value;
			cloud_it->b = 255 - value;
			break;
		case 3:
			// Grey (< 128) / Red (> 128)
			if (value > 128)
			{
				cloud_it->r = 255;
				cloud_it->g = 0;
				cloud_it->b = 0;
			}
			else
			{
				cloud_it->r = 128;
				cloud_it->g = 128;
				cloud_it->b = 128;
			}
			break;
		default:
			// Blue -> Green -> Red (~ rainbow)
			cloud_it->r = value > 128 ? (value - 128) * 2 : 0;  // r[128] = 0, r[255] = 255
			cloud_it->g = value < 128 ? 2 * value : 255 - ((value - 128) * 2);  // g[0] = 0, g[128] = 255, g[255] = 0
			cloud_it->b = value < 128 ? 255 - (2 * value) : 0;  // b[0] = 255, b[128] = 0
		}
	}
}

void cinspection::OBB_features(const pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output)
{
	if ((int)output.size() != 0) output.clear();
	vtkSmartPointer<vtkPoints> vtkPointCloud = vtkSmartPointer<vtkPoints>::New();
	for (int i = 0; i < (int)input.size(); i++)
	{
		double point[3];
		point[0] = input.points[i].x;
		point[1] = input.points[i].y;
		point[2] = input.points[i].z;
		vtkPointCloud->InsertNextPoint(point);
	}

	// Create the tree
	vtkSmartPointer<vtkOBBTree> obbTree = vtkSmartPointer<vtkOBBTree>::New();
	double 	corner_OBB[3], max_OBB[3], mid_OBB[3], min_OBB[3], size_OBB[3];
	obbTree->ComputeOBB(vtkPointCloud, corner_OBB, max_OBB, mid_OBB, min_OBB, size_OBB);
	double p[8][3];
	for (int i = 0; i < 3; i++)
	{
		p[0][i] = corner_OBB[i];
		p[1][i] = corner_OBB[i] + max_OBB[i];
		p[2][i] = corner_OBB[i] + mid_OBB[i];
		p[3][i] = corner_OBB[i] + min_OBB[i];
		p[4][i] = corner_OBB[i] + max_OBB[i] + mid_OBB[i] + min_OBB[i];
		p[5][i] = corner_OBB[i] + mid_OBB[i] + min_OBB[i];
		p[6][i] = corner_OBB[i] + max_OBB[i] + min_OBB[i];
		p[7][i] = corner_OBB[i] + max_OBB[i] + mid_OBB[i];
	}

	for (int i = 0; i < 8; i++)
	{
		pcl::PointXYZRGB point;
		point.x = p[i][0];
		point.y = p[i][1];
		point.z = p[i][2];
		point.r = 255;
		point.g = 0;
		point.b = 0;
		output.push_back(point);
	}

	double X, Y, Z;
	pcl::PointXYZRGB point;
	for (int i = 1; i < 4; i++)
	{
		X = (output.points[0].x - output.points[i].x);
		Y = (output.points[0].y - output.points[i].y);
		Z = (output.points[0].z - output.points[i].z);
		if (i == 1) point.x = sqrt(X*X + Y*Y + Z*Z);
		if (i == 2) point.y = sqrt(X*X + Y*Y + Z*Z);
		if (i == 3) point.z = sqrt(X*X + Y*Y + Z*Z);
	}
	output.push_back(point);

	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	polydata->SetPoints(vtkPointCloud);
	// Compute the center of mass
	vtkSmartPointer<vtkCenterOfMass> centerOfMassFilter = vtkSmartPointer<vtkCenterOfMass>::New();
#if VTK_MAJOR_VERSION <= 5
	centerOfMassFilter->SetInput(polydata);
#else
	centerOfMassFilter->SetInputData(polydata);
#endif
	centerOfMassFilter->SetUseScalarsAsWeights(false);
	centerOfMassFilter->Update();
	double center[3];
	centerOfMassFilter->GetCenter(center);
	point.x = center[0];
	point.y = center[1];
	point.z = center[2];
	output.push_back(point);
}

void cinspection::Cylinder_Detection(const pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &inliers,
	 pcl::PointCloud<pcl::PointXYZRGB> &outliers, double DistanceThreshold, double RadiusMin, double RadiusMax)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentedCloud;
	segmentedCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::copyPointCloud(input, *segmentedCloud);

	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::ModelCoefficients::Ptr  coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr  inliers_cylinder(new pcl::PointIndices);
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	// Estimate point normals
	ne.setSearchMethod(tree);
	ne.setInputCloud(segmentedCloud);
	ne.setKSearch(20);
	ne.compute(*cloud_normals);
	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CYLINDER);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight(0.1);//0.1
	seg.setMaxIterations(100000);
	seg.setDistanceThreshold(DistanceThreshold);//0.1
	seg.setRadiusLimits(RadiusMin, RadiusMax);//6
	seg.setInputCloud(segmentedCloud);
	seg.setInputNormals(cloud_normals);

	// Obtain the cylinder inliers and coefficients
	seg.segment(*inliers_cylinder, *coefficients_cylinder);
	// Write the cylinder inliers to disk
	extract.setInputCloud(segmentedCloud);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);
	extract.filter(inliers);
	extract.setNegative(true);
	extract.filter(outliers);
}

int cinspection::Sphere_Detection(pcl::PointCloud<pcl::PointXYZRGB> &input, double DistanceThresh, double radiusMin, double radiusMax, int minPoint, double *coff1, double *coff2 )
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inliers(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr outliers(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentedCloud;
	segmentedCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::copyPointCloud(input, *segmentedCloud);

	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::ModelCoefficients::Ptr  coefficients_sphere(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr  inliers_sphere(new pcl::PointIndices);
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	// Estimate point normals
	ne.setSearchMethod(tree);
	ne.setInputCloud(segmentedCloud);
	ne.setKSearch(20);
	ne.compute(*cloud_normals);
	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_SPHERE);
	seg.setMethodType(pcl::SAC_RANSAC);
	//seg.setNormalDistanceWeight(0.5);//0.1
	seg.setMaxIterations(10000);
	seg.setDistanceThreshold(DistanceThresh);//0.1
	seg.setRadiusLimits(radiusMin, radiusMax);
	seg.setInputCloud(segmentedCloud);
	seg.setInputNormals(cloud_normals);

	// Obtain the cylinder inliers and coefficients
	seg.segment(*inliers_sphere, *coefficients_sphere);
	if (inliers_sphere->indices.size() < minPoint) return 0;
	// Write the sphere inliers to disk
	extract.setInputCloud(segmentedCloud);
	extract.setIndices(inliers_sphere);
	extract.setNegative(false);
	extract.filter(*inliers);
	extract.setNegative(true);
	extract.filter(*outliers);

	coff1[0] = coefficients_sphere->values[0];
	coff1[1] = coefficients_sphere->values[1];
	coff1[2] = coefficients_sphere->values[2];
	coff1[3] = coefficients_sphere->values[3];
	
	// Round 2
	inliers_sphere.reset(new pcl::PointIndices);
	cloud_normals.reset(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(outliers);
	ne.compute(*cloud_normals);
	seg.setInputCloud(outliers);
	seg.setInputNormals(cloud_normals);
	seg.segment(*inliers_sphere, *coefficients_sphere);
	if (inliers_sphere->indices.size() < minPoint) return 1;
	coff2[0] = coefficients_sphere->values[0];
	coff2[1] = coefficients_sphere->values[1];
	coff2[2] = coefficients_sphere->values[2];
	coff2[3] = coefficients_sphere->values[3];

	return 2;
}

bool cinspection::Sphere_Marker_Matching(pcl::PointCloud<pcl::PointXYZRGB> &source, pcl::PointCloud<pcl::PointXYZRGB> &target,
	pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output, double thresh)
{
	ofstream onlineFile;
	onlineFile.open("Report/Report_online.txt");
	if (source.size() < 6) return 1;

	pcl::PointCloud<pcl::PointXYZ>::Ptr SVD_Source(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr SVD_Source_Buf(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr SVD_Target(new pcl::PointCloud<pcl::PointXYZ>());

	onlineFile << "Matching Score: " << "\n";
	for (int i = 0; i < source.size(); i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_nearest_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		if (PCL_NearestNeighbors_Search(source, source, *source_nearest_cloud, i, 6)) return 1;

		for (int k = 0; k < target.size(); k++)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_nearest_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			if (PCL_NearestNeighbors_Search(target, target, *target_nearest_cloud, k, 6)) return 1;
			
			pcl::copyPointCloud(*source_nearest_cloud, *SVD_Source_Buf);
			pcl::copyPointCloud(*target_nearest_cloud, *SVD_Target);
			for (int i2 = 2; i2 < 6; i2++)
			for (int i3 = 2; i3 < 6; i3++)
			for (int i4 = 2; i4 < 6; i4++)
			for (int i5 = 2; i5 < 6; i5++)
			{
				if (SVD_Source->size()) SVD_Source->clear();
				SVD_Source->push_back(SVD_Source_Buf->points[0]);
				SVD_Source->push_back(SVD_Source_Buf->points[1]);
				if (i2 == i3 || i2 == i4 || i2 == i5 || i3 == i4 || i3 == i5 || i4 == i5) continue;
				SVD_Source->push_back(SVD_Source_Buf->points[i2]);
				SVD_Source->push_back(SVD_Source_Buf->points[i3]);
				SVD_Source->push_back(SVD_Source_Buf->points[i4]);
				SVD_Source->push_back(SVD_Source_Buf->points[i5]);

				pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> TESVD;
				pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 SVD_transformation;
				// Note: SVD only useful when source and target in order of corresponding
				TESVD.estimateRigidTransformation(*SVD_Source, *SVD_Target, SVD_transformation);
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr transfer_Cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
				pcl::transformPointCloud(*source_nearest_cloud, *transfer_Cloud, SVD_transformation);
				double dist;
				PCL_MeanDistance_CloudToCloud(*transfer_Cloud, *target_nearest_cloud, dist);
				if (dist < 3 * thresh) onlineFile << dist << "\n";
				if (dist < thresh)
				{
					pcl::transformPointCloud(input, output, SVD_transformation);
					matr = SVD_transformation.replicate(4, 4);
				    return 0;
				}
			}
		}
	}
	onlineFile.close();
	return 1;
}

bool cinspection::Sphere_Marker_Matching_Fix(pcl::PointCloud<pcl::PointXYZRGB> &source, pcl::PointCloud<pcl::PointXYZRGB> &target,
	pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output, int mod_index, double thresh)
{
	ofstream onlineFile;
	onlineFile.open("Report/Report_online.txt");
	if (source.size() < 6) return 1;

	pcl::PointCloud<pcl::PointXYZ>::Ptr SVD_Source(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr SVD_Source_Buf(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr SVD_Target(new pcl::PointCloud<pcl::PointXYZ>());

	onlineFile << "Matching Score: " << "\n";
	for (int i = 0; i < source.size(); i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_nearest_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		if (PCL_NearestNeighbors_Search(source, source, *source_nearest_cloud, i, 6)) return 1;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_nearest_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		if (PCL_NearestNeighbors_Search(target, target, *target_nearest_cloud, mod_index, 6)) return 1;

		pcl::copyPointCloud(*source_nearest_cloud, *SVD_Source_Buf);
		pcl::copyPointCloud(*target_nearest_cloud, *SVD_Target);
		for (int i2 = 2; i2 < 6; i2++)
		for (int i3 = 2; i3 < 6; i3++)
		for (int i4 = 2; i4 < 6; i4++)
		for (int i5 = 2; i5 < 6; i5++)
		{
			if (SVD_Source->size()) SVD_Source->clear();
			SVD_Source->push_back(SVD_Source_Buf->points[0]);
			SVD_Source->push_back(SVD_Source_Buf->points[1]);
			if (i2 == i3 || i2 == i4 || i2 == i5 || i3 == i4 || i3 == i5 || i4 == i5) continue;
			SVD_Source->push_back(SVD_Source_Buf->points[i2]);
			SVD_Source->push_back(SVD_Source_Buf->points[i3]);
			SVD_Source->push_back(SVD_Source_Buf->points[i4]);
			SVD_Source->push_back(SVD_Source_Buf->points[i5]);

			pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> TESVD;
			pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 SVD_transformation;
			// Note: SVD only useful when source and target in order of corresponding
			TESVD.estimateRigidTransformation(*SVD_Source, *SVD_Target, SVD_transformation);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transfer_Cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
			pcl::transformPointCloud(*source_nearest_cloud, *transfer_Cloud, SVD_transformation);
			double dist;
			PCL_MeanDistance_CloudToCloud(*transfer_Cloud, *target_nearest_cloud, dist);
			if (dist < 3 * thresh) onlineFile << dist << "\n";
			if (dist < thresh)
			{
				pcl::transformPointCloud(input, output, SVD_transformation);
				matr = SVD_transformation.replicate(4, 4);
				return 0;
			}
		}
	}
	onlineFile.close();
	return 1;
}

bool cinspection::PCL_NearestNeighbors_Search(pcl::PointCloud<pcl::PointXYZRGB> &source, pcl::PointCloud<pcl::PointXYZRGB> &target,
	 pcl::PointCloud<pcl::PointXYZRGB> &output, int index, int k)
{
	if (source.size() < 6) return 1;

	if (source.size() == 0 || target.size() < k) return 1;
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(target, *target_cloud);
	pcl::copyPointCloud(source, *source_cloud);

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(target_cloud);
	std::vector<int> pointIdxNKNSearch(k);
	std::vector<float> pointNKNSquaredDistance(k);
	if (kdtree.nearestKSearch(source_cloud->points[index], k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
	{
		for (int i = 0; i < pointIdxNKNSearch.size(); i++)
		{
			output_cloud->push_back(target_cloud->points[pointIdxNKNSearch[i]]);
		}
	}
	pcl::copyPointCloud(*output_cloud, output);
	return 0;
}

void cinspection::Denoise_Statistical(const pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output,
	 double StddevMulThresh)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud_input(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud_output(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(input, *Cloud_input);

	int MeanK = 20;
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> NoiseRemoval;
	NoiseRemoval.setInputCloud(Cloud_input);
	NoiseRemoval.setMeanK(MeanK);
	NoiseRemoval.setStddevMulThresh(StddevMulThresh);
	NoiseRemoval.filter(*Cloud_output);
	pcl::copyPointCloud(*Cloud_output, output);
}

void cinspection::Denoise_Segment(const pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output,
	 double tolerance, int minpoints)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr SegmentedCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(input, *SegmentedCloud);

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud(SegmentedCloud);
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(*cloud_filtered);

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(tolerance); // 2cm
	ec.setMinClusterSize(minpoints); //100
	ec.setMaxClusterSize(99000000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);

	int j = 0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_NoiseRemoved(new pcl::PointCloud<pcl::PointXYZ>);
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointXYZRGB total;
		total.x = 0;
		total.y = 0;
		total.z = 0;
		int numOfpoint = 0;

		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		{
			cloud_NoiseRemoved->points.push_back(cloud_filtered->points[*pit]);
		}

	}
	if ((int)output.size() != 0) output.clear();
	pcl::copyPointCloud(*cloud_NoiseRemoved, output);
}

void cinspection::OBB_Inliers(pcl::PointCloud<pcl::PointXYZRGB> &input, const pcl::PointCloud<pcl::PointXYZRGB> &input_OBB,
	pcl::PointCloud<pcl::PointXYZRGB> &output_Inliers, pcl::PointCloud<pcl::PointXYZRGB> &output_Outliers)
{
	vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
	for (int i = 0; i < 8; i++)
	{
		polygon->GetPoints()->InsertNextPoint(input_OBB.points[i].x, input_OBB.points[i].y, input_OBB.points[i].z);
	}
	double n[3];
	polygon->ComputeNormal(polygon->GetPoints()->GetNumberOfPoints(), static_cast<double*>(polygon->GetPoints()->GetData()->GetVoidPointer(0)), n);
	double bounds[6];
    polygon->GetPoints()->GetBounds(bounds);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudInquery (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(input, *cloudInquery);
	if ((int)output_Outliers.size() != 0) output_Outliers.clear();

	for (int i = 0; i < (int)cloudInquery->size(); i++)
	{
		double point[3];
		point[0] = cloudInquery->points[i].x;
		point[1] = cloudInquery->points[i].y;
		point[2] = cloudInquery->points[i].z;
		if(polygon->PointInPolygon(point, polygon->GetPoints()->GetNumberOfPoints(), static_cast<double*>(
			polygon->GetPoints()->GetData()->GetVoidPointer(0)), bounds, n))
		{
			output_Inliers.push_back(cloudInquery->points[i]);
		}
		else output_Outliers.push_back(cloudInquery->points[i]);
	}

}

void cinspection::Distances_CloudToCloud(pcl::PointCloud<pcl::PointXYZRGB> &source_Cloud, pcl::PointCloud<pcl::PointXYZRGB> &target_Cloud,
	pcl::PointCloud<pcl::PointXYZ> &dists_Cloud)
{
	vtkSmartPointer<vtkPoints>  source = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints>  target = vtkSmartPointer<vtkPoints>::New();

	PCL_TO_VTK(source_Cloud, source);
	PCL_TO_VTK(target_Cloud, target);
	int PointNum_query = (int)source->GetNumberOfPoints();
	int PointNum_data = (int)target->GetNumberOfPoints();
	double meandist = 0;
	double Max_dist = -1 * DBL_MAX;
	double Min_dist = DBL_MAX;
	//--------------------------------------------------
	// KNN METHOD for finding neighbors
	//--------------------------------------------------	

	int knn = 3;
	int dim = 3;						// dimension
	double	eps = 0;						// error bound
	int maxPts_query = PointNum_query;					// maximum number of data points
	int maxPts_data = PointNum_data;					// maximum number of data points

	int				ndataPts;					// actual number of data points
	int				nqueryPts;					// actual number of data points
	ANNpointArray	dataPts;				// data points
	ANNpointArray	queryPts;
	ANNpoint		queryPt;				// query point
	ANNidxArray		nnIdx;					// near neighbor indices
	ANNdistArray	dists;					// near neighbor distances
	ANNkd_tree*		kdTree;					// search structure

	queryPt = annAllocPt(dim);				// allocate query point
	dataPts = annAllocPts(maxPts_data, dim);	    // allocate data points
	queryPts = annAllocPts(maxPts_query, dim);	    // allocate data points
	nnIdx = new ANNidx[knn];				// allocate near neigh indices
	dists = new ANNdist[knn];				// allocate near neighbor dists

	ndataPts = 0;

	while (ndataPts < maxPts_data) {
		double p[3];
		target->GetPoint(ndataPts, p);
		dataPts[ndataPts][0] = p[0];
		dataPts[ndataPts][1] = p[1];
		dataPts[ndataPts][2] = p[2];
		ndataPts++;
	}

	nqueryPts = 0;
	while (nqueryPts < maxPts_query) {
		double q[3];
		source->GetPoint(nqueryPts, q);
		queryPts[nqueryPts][0] = q[0];
		queryPts[nqueryPts][1] = q[1];
		queryPts[nqueryPts][2] = q[2];
		nqueryPts++;
	}

	kdTree = new ANNkd_tree(	// build search structure
		dataPts,				// the data points
		ndataPts,				// number of points
		dim);					// dimension of space

	for (int searchP = 0; searchP < PointNum_query; searchP++)
	{
		queryPt = queryPts[searchP];	// Query the first point
		kdTree->annkSearch(			// search
			queryPt,				// query point
			knn,					// number of near neighbors
			nnIdx,					// nearest neighbors (returned)
			dists,					// squared distance (returned)
			eps);					// error bound

		pcl::PointXYZ point;
		pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		for (int k = 0; k < 3; k++)
		{
			point.x = dataPts[nnIdx[k]][0];
			point.y = dataPts[nnIdx[k]][1];
			point.z = dataPts[nnIdx[k]][2];
			plane_cloud->push_back(point);
		}
		double a, b, c, d;
		double dist;
		if (threePointToPlane(*plane_cloud, a, b, c, d))
		{
			point.x = queryPt[0];
			point.y = queryPt[1];
			point.z = queryPt[2];
			dist = pcl::pointToPlaneDistance(point, a, b, c, d);
		}
		else dist = sqrt(dists[0]);

		meandist += dist;
		point.x = dist;
		point.y = 0;
		point.z = 0;
		dists_Cloud.push_back(point);
		if (Max_dist < dist) Max_dist = dist;
		if (Min_dist > dist) Min_dist = dist;
	}
		
	meandist = (double)meandist / PointNum_query;
	
	pcl::PointXYZ point;
	point.x = meandist;
	point.y = Max_dist;
	point.z = Min_dist;
	dists_Cloud.push_back(point);

	delete[] nnIdx;							// clean things up
	delete[] dists;
	delete kdTree;
	annClose();									// done with ANN
}

void cinspection::Display_Error_Report(pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZ> &dists_Cloud)
{
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	PCL_TO_VTK(input, points);

	vtkSmartPointer<vtkActor>					actorDisplayPC = vtkSmartPointer<vtkActor>::New();
	vtkSmartPointer<vtkRenderer>				renDisplayPC = vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow>			renderwindowDisplayPC = vtkSmartPointer<vtkRenderWindow>::New();
	vtkSmartPointer<vtkRenderWindowInteractor>	irenDisplayPC = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	vtkInteractorStyleTrackballCamera			*styleDisplayPC = vtkInteractorStyleTrackballCamera::New();
	 
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	polydata->SetPoints(points);

	vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	glyphFilter->SetInputData(polydata);
	glyphFilter->Update();

	polydata->ShallowCopy(glyphFilter->GetOutput());
	glyphFilter->Update();

	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polydata);
	mapper->Update();

	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);

	double point_size = 2;
	actor->GetProperty()->SetPointSize(point_size);

	double bound_data[6];
	polydata->GetBounds(bound_data);

	vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	vtkSmartPointer<vtkLookupTable> colorLookupTable = vtkSmartPointer<vtkLookupTable>::New();
	vtkSmartPointer<vtkLookupTable> colorLookupTable_Bar = vtkSmartPointer<vtkLookupTable>::New();
	double MaxValue = dists_Cloud.points[dists_Cloud.size() - 1].y;
	colorLookupTable->SetTableRange(0, MaxValue);
	colorLookupTable->Build();
	colorLookupTable_Bar->SetTableRange(0, MaxValue);
	colorLookupTable_Bar->Build();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");

	polydata->GetPointData()->SetScalars(colors);

	for (vtkIdType i = 0; i < points->GetNumberOfPoints(); i++)
	{
		double p[3];
		points->GetPoint(i, p);
		double dcolor[3];
		unsigned char color[3];
		colorLookupTable->GetColor(MaxValue - dists_Cloud.points[i].x, dcolor);
		for (unsigned int j = 0; j < 3; j++)
		{
			color[j] = static_cast<unsigned char>(255.0 * dcolor[j]);
		}
		colors->InsertNextTupleValue(color);
	}

	for (int i = 0; i <= 255; i++)
	{
		double colo[4];
		colorLookupTable->GetTableValue(255 - i, colo);
		colorLookupTable_Bar->SetTableValue(i, colo[0], colo[1], colo[2], 1.0);
	}

	vtkSmartPointer<vtkScalarBarActor> scalarBar = vtkSmartPointer<vtkScalarBarActor>::New();
	scalarBar->SetTitle("Error (mm)");
	scalarBar->SetNumberOfLabels(12);
	scalarBar->SetWidth(0.1);
	scalarBar->SetHeight(0.8);
	scalarBar->SetPosition(0.05, 0.05);
	scalarBar->SetLookupTable(colorLookupTable_Bar);

	// Set camera view
	/*renDisplayPC->ResetCamera();
	vtkCamera* camera = renDisplayPC->GetActiveCamera();
	camera->SetFocalPoint((bound_data[1] + bound_data[0]) / 2, (bound_data[3] + bound_data[2]) / 2, 0.0);
	double d = camera->GetDistance();
	camera->SetPosition((bound_data[1] + bound_data[0]) / 2, (bound_data[3] + bound_data[2]) / 2, d/2);
	camera->Zoom(1.0);*/

	// Add Axis
	/*vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
	axes->SetTotalLength((abs)(bound_data[1] - bound_data[0]), (abs)(bound_data[3] - bound_data[2]), (abs)(bound_data[5] - bound_data[4]));
	axes->SetConeRadius(0.2);
	axes->SetNormalizedShaftLength(0.95, 0.95, 0.95);
	axes->SetNormalizedTipLength(0.05, 0.05, 0.05);
	renDisplayPC->AddViewProp(axes);*/

	// Remove precious point clouds and shapes
	renDisplayPC->RemoveAllViewProps();
	// Display	
	actorDisplayPC->GetProperty()->SetColor(1.0, 1.0, 0.0);
	renDisplayPC->AddActor(actorDisplayPC);
	renDisplayPC->SetBackground(0.0, 0.0, 0.0);
	renDisplayPC->AddViewProp(actor);
	renDisplayPC->AddActor2D(scalarBar);
	renderwindowDisplayPC->AddRenderer(renDisplayPC);
	renderwindowDisplayPC->Render();
	irenDisplayPC->SetRenderWindow(renderwindowDisplayPC);
	irenDisplayPC->SetInteractorStyle(styleDisplayPC);
	irenDisplayPC->Start();
}

bool cinspection::threePointToPlane(pcl::PointCloud<pcl::PointXYZ> &input, double &a, double &b, double &c, double &d)
{
	if (input.size() < 3) return false;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(input, *cloud);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.1);
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0) return false;
	else
	{
		a = coefficients->values[0];
		b = coefficients->values[1];
		c = coefficients->values[2];
		d = coefficients->values[3];
		return true;
	}
}

bool cinspection::PCL_MeanDistance_CloudToCloud(pcl::PointCloud<pcl::PointXYZRGB> &source, pcl::PointCloud<pcl::PointXYZRGB> &target, double &meandist)
{
	if (source.size() == 0 || target.size() == 0) return 0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(target, *target_cloud);
	pcl::copyPointCloud(source, *source_cloud);
	
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(target_cloud);
	std::vector<int> pointIdxNKNSearch(1);
	std::vector<float> pointNKNSquaredDistance(1);

	meandist = 0;
	for (int i = 0; i < source.size(); ++i)
	{
		if (kdtree.nearestKSearch(source_cloud->points[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			meandist += sqrt(pointNKNSquaredDistance[0]);
		}
	}

	//calculating the mean distance
	meandist = (double) meandist / source.size();
	return 1;
}

bool cinspection::PCL_Overlap_Detection(pcl::PointCloud<pcl::PointXYZRGB> &source, pcl::PointCloud<pcl::PointXYZRGB> &target,
	pcl::PointCloud<pcl::PointXYZRGB> &output, double thresh)
{
	if (source.size() == 0 || target.size() == 0) return 1;
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(target, *target_cloud);
	pcl::copyPointCloud(source, *source_cloud);

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(target_cloud);
	std::vector<int> pointIdxNKNSearch(1);
	std::vector<float> pointNKNSquaredDistance(1);
	if (output.size()) output.clear();
	for (int i = 0; i < source.size(); ++i)
	{
		if (kdtree.nearestKSearch(source_cloud->points[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			if (sqrt(pointNKNSquaredDistance[0]) < thresh) output.push_back(source.points[i]);
		}
	}
	if (output.size()) return 0;
	return 1;
}

bool cinspection::kNearestMeanDist_PCL(pcl::PointCloud<pcl::PointXYZRGB> &input, int Nearest_K, double &meandist)
{
	if (input.size() == 0 || input.size() == 0) return 0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(input, *input_cloud);

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(input_cloud);
	std::vector<int> pointIdxNKNSearch(Nearest_K);
	std::vector<float> pointNKNSquaredDistance(Nearest_K);

	meandist = 0;
	for (int i = 0; i < input.size(); ++i)
	{
		if (kdtree.nearestKSearch(input_cloud->points[i], Nearest_K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			double total = 0;
			for (size_t j = 1; j < pointIdxNKNSearch.size(); ++j)
			{
				total += sqrt(pointNKNSquaredDistance[j]);
			}
			meandist += (double) total/(Nearest_K - 1);
		}
	}

	//calculating the mean distance
	meandist = (double)meandist / input.size();
	return 1;
}

void cinspection::Distances_CloudToCloud_PCL(pcl::PointCloud<pcl::PointXYZRGB> &source, pcl::PointCloud<pcl::PointXYZRGB> &target,
	pcl::PointCloud<pcl::PointXYZ> &dists_Cloud)
{
	if (source.size() == 0 || target.size() == 0) return;
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(source, *source_cloud);
	pcl::copyPointCloud(target, *target_cloud);

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(target_cloud);
	std::vector<int> pointIdxNKNSearch(3);
	std::vector<float> pointNKNSquaredDistance(3);

	double meandist = 0;
	double dist;
	double Max_dist = -1 * DBL_MAX;
	double Min_dist = DBL_MAX;
	for (int i = 0; i < source.size(); ++i)
	{
		if (kdtree.nearestKSearch(source_cloud->points[i], 3, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			for (int k = 0; k < 3; k++)
			{
				plane_cloud->push_back(target_cloud->points[pointIdxNKNSearch[k]]);
			}
			double a, b, c, d;
			if (threePointToPlane(*plane_cloud, a, b, c, d))
			{
				dist = pcl::pointToPlaneDistance(source.points[i], a, b, c, d);
			}
			else dist = sqrt(pointNKNSquaredDistance[0]);

			if (dist > 1) dist = dist / 20;

			meandist += dist;
			pcl::PointXYZ point;
			point.x = dist;
			point.y = 0;
			point.z = 0;
			dists_Cloud.push_back(point);
			if (Max_dist < dist) Max_dist = dist;
			if (Min_dist > dist) Min_dist = dist;
		}
	}

	//calculating the mean distance
	meandist = (double)meandist / source.size();
	pcl::PointXYZ point;
	point.x = meandist;
	point.y = Max_dist;
	point.z = Min_dist;
	dists_Cloud.push_back(point);
	return;
}

void cinspection::OBB_Calculation(pcl::PointCloud<pcl::PointXYZRGB> &input, double *OBB_Conner, double *OBB_Max, double *OBB_Mid, double *OBB_Min)
{
	vtkSmartPointer<vtkPoints>  VTK_cloud = vtkSmartPointer<vtkPoints>::New();
	PCL_TO_VTK(input, VTK_cloud);

	OBB_Conner[0] = 0;
	OBB_Conner[1] = 0;
	OBB_Conner[2] = 0;
	OBB_Min[0] = 0;
	OBB_Min[1] = 0;
	OBB_Min[2] = 0;
	OBB_Mid[0] = 0;
	OBB_Mid[1] = 0;
	OBB_Mid[2] = 0;
	OBB_Max[0] = 0;
	OBB_Max[1] = 0;
	OBB_Max[2] = 0;

	// Create the tree
	vtkSmartPointer<vtkOBBTree> obbTree = vtkSmartPointer<vtkOBBTree>::New();
	double 	max_OBB[3], mid_OBB[3], min_OBB[3], size_OBB[3];
	obbTree->ComputeOBB(VTK_cloud, OBB_Conner, max_OBB, mid_OBB, min_OBB, size_OBB);
	for (int i = 0; i < 3; i++)
	{
		OBB_Max[i] = max_OBB[i];
		OBB_Mid[i] = mid_OBB[i];
		OBB_Min[i] = min_OBB[i];
	}
}

void cinspection::RealcoordinateToOBBcoordinateXY(pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output,
	double corner_OBB[3], double max_OBB[3], double mid_OBB[3], double min_OBB[3])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr SVD_Source(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr SVD_Target(new pcl::PointCloud<pcl::PointXYZ>());

	double lengthOBB_object[3];
	lengthOBB_object[0] = sqrt(max_OBB[0] * max_OBB[0] + max_OBB[1] * max_OBB[1] + max_OBB[2] * max_OBB[2]); // length of max
	lengthOBB_object[1] = sqrt(mid_OBB[0] * mid_OBB[0] + mid_OBB[1] * mid_OBB[1] + mid_OBB[2] * mid_OBB[2]); // length of mid
	lengthOBB_object[2] = sqrt(min_OBB[0] * min_OBB[0] + min_OBB[1] * min_OBB[1] + min_OBB[2] * min_OBB[2]); // length of min
	double point[4][3];
	for (int i = 0; i < 3; i++)
	{
		point[0][i] = corner_OBB[i];
		point[1][i] = corner_OBB[i] + (double)max_OBB[i] / lengthOBB_object[0];
		point[2][i] = corner_OBB[i] + (double)mid_OBB[i] / lengthOBB_object[1];
		point[3][i] = corner_OBB[i] + (double)min_OBB[i] / lengthOBB_object[2];
	}
	for (int i = 0; i < 3; i++) SVD_Source->push_back(pcl::PointXYZ(point[i][0], point[i][1], point[i][2]));

	SVD_Target->push_back(pcl::PointXYZ(0, 0, 0));
	SVD_Target->push_back(pcl::PointXYZ(1, 0, 0));
	SVD_Target->push_back(pcl::PointXYZ(0, 1, 0));

	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> TESVD;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 SVD_transformation;
	TESVD.estimateRigidTransformation(*SVD_Source, *SVD_Target, SVD_transformation);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SVD_Source_Cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::copyPointCloud(input, *SVD_Source_Cloud);
	if (output.size()) output.clear();
	pcl::transformPointCloud(*SVD_Source_Cloud, output, SVD_transformation);
	
	matr = Eigen::Matrix4f::Identity();
	matr = SVD_transformation.replicate(4, 4);
}

void cinspection::TransferToOBB_CorrdinateXY(pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output)
{
	Eigen::Matrix4f matr_transfer = Eigen::Matrix4f::Identity();
	Eigen::Vector4f center;
	Eigen::Affine3f transformMatrix = Eigen::Affine3f::Identity();
	double corner_OBB[3], max_OBB[3], mid_OBB[3], min_OBB[3];

	OBB_Calculation(input, corner_OBB, max_OBB, mid_OBB, min_OBB);
	RealcoordinateToOBBcoordinateXY(input, output, corner_OBB, max_OBB, mid_OBB, min_OBB);
	matr_transfer = matr.replicate(4, 4);
	pcl::compute3DCentroid(output, center);
	transformMatrix.translation() << -center(0, 0), -center(1, 0), -center(2, 0);
	pcl::transformPointCloud(output, output, transformMatrix);
	matr_transfer = transformMatrix.matrix() * matr_transfer;
	matr = matr_transfer.replicate(4, 4);
}

void cinspection::OBB_Length(pcl::PointCloud<pcl::PointXYZRGB> &input, double *OBB_Length)
{
	OBB_Length[0] = 0;
	OBB_Length[1] = 0;
	OBB_Length[2] = 0;
	double corner_OBB[3], max_OBB[3], mid_OBB[3], min_OBB[3];
	OBB_Calculation(input, corner_OBB, max_OBB, mid_OBB, min_OBB);
	OBB_Length[0] = sqrt(max_OBB[0] * max_OBB[0] + max_OBB[1] * max_OBB[1] + max_OBB[2] * max_OBB[2]); // length of max
	OBB_Length[1] = sqrt(mid_OBB[0] * mid_OBB[0] + mid_OBB[1] * mid_OBB[1] + mid_OBB[2] * mid_OBB[2]); // length of mid
	OBB_Length[2] = sqrt(min_OBB[0] * min_OBB[0] + min_OBB[1] * min_OBB[1] + min_OBB[2] * min_OBB[2]); // length of min
}

bool cinspection::numOfSubOBB_Estimation(pcl::PointCloud<pcl::PointXYZRGB> &object, pcl::PointCloud<pcl::PointXYZRGB> &model, int *numOfSubOBB)
{
	double OBB_Length_scene[3];
	double OBB_Length_model[3];
	OBB_Length(object, OBB_Length_scene);
	OBB_Length(model, OBB_Length_model);

	numOfSubOBB[0] = 0;
	numOfSubOBB[1] = 0;
	numOfSubOBB[2] = 0;

	//Estimate number of subOBBs of measured point cloud based on length of its OBB and number of subOBBs of OBB of model
	numOfSubOBB[0] = NumOfSubOBBX_model * ((double)OBB_Length_scene[0] / OBB_Length_model[0]) + 0.5;//0.5 means if 1.7->2, if 1.3->1
	numOfSubOBB[1] = NumOfSubOBBY_model * ((double)OBB_Length_scene[1] / OBB_Length_model[1]) + 0.5;//0.5 means if 1.7->2, if 1.3->1
	if (numOfSubOBB[0] == 0) numOfSubOBB[0] = 1;
	if (numOfSubOBB[1] == 0) numOfSubOBB[1] = 1;

	if (OBB_Length_scene[0] > OBB_Length_model[0] || OBB_Length_scene[1] > OBB_Length_model[1]) return 1;
	return 0;
}

void cinspection::OBB_Alignment(pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output)
{
	double center[3];
	double corner_OBB[3], max_OBB[3], mid_OBB[3], min_OBB[3];

	pcl::PointCloud<pcl::PointXYZ>::Ptr SVD_Source(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr SVD_Target(new pcl::PointCloud<pcl::PointXYZ>());

	OBB_Calculation(input, corner_OBB, max_OBB, mid_OBB, min_OBB);
	double lengthOBB_object[3];
	lengthOBB_object[0] = sqrt(max_OBB[0] * max_OBB[0] + max_OBB[1] * max_OBB[1] + max_OBB[2] * max_OBB[2]); // length of max
	lengthOBB_object[1] = sqrt(mid_OBB[0] * mid_OBB[0] + mid_OBB[1] * mid_OBB[1] + mid_OBB[2] * mid_OBB[2]); // length of mid
	lengthOBB_object[2] = sqrt(min_OBB[0] * min_OBB[0] + min_OBB[1] * min_OBB[1] + min_OBB[2] * min_OBB[2]); // length of min


	double point[4][3];
	for (int i = 0; i < 3; i++)
	{
		point[0][i] = corner_OBB[i];
		point[1][i] = corner_OBB[i] + (double)max_OBB[i] / lengthOBB_object[0];
		point[2][i] = corner_OBB[i] + (double)mid_OBB[i] / lengthOBB_object[1];
		point[3][i] = corner_OBB[i] + (double)min_OBB[i] / lengthOBB_object[2];
	}
	for (int i = 0; i < 3; i++) SVD_Source->push_back(pcl::PointXYZ(point[i][0], point[i][1], point[i][2]));

	SVD_Target->push_back(pcl::PointXYZ(0, 0, 0));
	SVD_Target->push_back(pcl::PointXYZ(1, 0, 0));
	SVD_Target->push_back(pcl::PointXYZ(0, 1, 0));

	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> TESVD;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 SVD_transformation;
	TESVD.estimateRigidTransformation(*SVD_Source, *SVD_Target, SVD_transformation);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SVD_Source_Cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::copyPointCloud(input, *SVD_Source_Cloud);
	if (output.size()) output.clear();
	pcl::transformPointCloud(*SVD_Source_Cloud, output, SVD_transformation);

	Eigen::Affine3f transformMatrix = Eigen::Affine3f::Identity();
	Eigen::Vector4f center_obj;
	pcl::compute3DCentroid(output, center_obj);
	transformMatrix.translation() << -center_obj(0, 0), -center_obj(1, 0), -center_obj(2, 0);
	pcl::transformPointCloud(output, output, transformMatrix);
}

void cinspection::OBB_Descriptor(pcl::PointCloud<pcl::PointXYZRGB> &Input, pcl::PointCloud<pcl::PointXYZRGB> &Descriptor, int numOfSubOBBX, 
	 int numOfSubOBBY, double &subobbAreaMax)
{
	int NumOfPoints = Input.size();
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_TransformedPoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(Input, *pcl_TransformedPoints);
	int NumOfSubOBB = numOfSubOBBX*numOfSubOBBY;

	// Find Max Min X Y Z
	double MaxX = -1 * DBL_MAX;
	double MinX = DBL_MAX;
	double MaxY = -1 * DBL_MAX;
	double MinY = DBL_MAX;
	for (int i = 0; i < pcl_TransformedPoints->size(); i++)
	{
		if (pcl_TransformedPoints->points[i].x > MaxX) MaxX = pcl_TransformedPoints->points[i].x;
		if (pcl_TransformedPoints->points[i].x < MinX) MinX = pcl_TransformedPoints->points[i].x;
		if (pcl_TransformedPoints->points[i].y > MaxY) MaxY = pcl_TransformedPoints->points[i].y;
		if (pcl_TransformedPoints->points[i].y < MinY) MinY = pcl_TransformedPoints->points[i].y;
	}
	// Initializing for descriptor
	if (Descriptor.size()) Descriptor.clear();
	for (int OBBindex = 0; OBBindex < NumOfSubOBB; OBBindex++)
	{
		pcl::PointXYZRGB point;
		point.x = 0;
		point.y = 0;
		point.z = 0;
		Descriptor.push_back(point);
	}
	double deltaX = (MaxX - MinX) / (double)numOfSubOBBX;
	double deltaY = (MaxY - MinY) / (double)numOfSubOBBY;
	if (deltaX == 0 || deltaY == 0) return;
	for (int m = 0; m < pcl_TransformedPoints->size(); m++)
	{
		int index_X = (pcl_TransformedPoints->points[m].x - MinX) / deltaX;
		int index_Y = (pcl_TransformedPoints->points[m].y - MinY) / deltaY;
		int OBB_index = index_Y*numOfSubOBBX + index_X;
		double p_Descriptor[3];
		if (OBB_index < NumOfSubOBB)
		{
			Descriptor.points[OBB_index].x = 0;
			Descriptor.points[OBB_index].y = 0;
			Descriptor.points[OBB_index].z += 1;
			/*Descriptor.points[OBB_index].r = index_X;
			Descriptor.points[OBB_index].g = index_Y;*/
		}
	}
	subobbAreaMax = 0;
	for (int i = 0; i < Descriptor.size(); i++)
	{
		if (Descriptor.points[i].z > subobbAreaMax) subobbAreaMax = Descriptor.points[i].z;
	}
}

void cinspection::OBB_Descriptor_Opt(pcl::PointCloud<pcl::PointXYZRGB> &Input, pcl::PointCloud<pcl::PointXYZRGB> &Descriptor, int numOfSub[3], double &subobbAreaMax)
{
	int NumOfPoints = Input.size();
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_TransformedPoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(Input, *pcl_TransformedPoints);

	// Find Max Min X Y Z
	double MaxX = -1 * DBL_MAX;
	double MinX = DBL_MAX;
	double MaxY = -1 * DBL_MAX;
	double MinY = DBL_MAX;
	for (int i = 0; i < pcl_TransformedPoints->size(); i++)
	{
		if (pcl_TransformedPoints->points[i].x > MaxX) MaxX = pcl_TransformedPoints->points[i].x;
		if (pcl_TransformedPoints->points[i].x < MinX) MinX = pcl_TransformedPoints->points[i].x;
		if (pcl_TransformedPoints->points[i].y > MaxY) MaxY = pcl_TransformedPoints->points[i].y;
		if (pcl_TransformedPoints->points[i].y < MinY) MinY = pcl_TransformedPoints->points[i].y;
	}
	int numOfSubOBBX = numOfSub[0];
	int numOfSubOBBY = numOfSub[1];
	int NumOfSubOBB = numOfSub[0] * numOfSub[1];

	// Initializing for descriptor
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CenterSubObb(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (Descriptor.size()) Descriptor.clear();
	for (int OBBindex = 0; OBBindex < NumOfSubOBB + 1; OBBindex++)
	{
		pcl::PointXYZRGB point;
		point.x = 0;
		point.y = 0;
		point.z = 0;
		Descriptor.push_back(point);
		if (OBBindex < NumOfSubOBB) CenterSubObb->push_back(point);
	}
	Descriptor.points[0].x = numOfSubOBBX;
	Descriptor.points[0].y = numOfSubOBBY;
	Descriptor.points[0].z = 1;

	if (SubOBB_Size == 0) return;

	double deltaX = (MaxX - MinX) / (double)numOfSubOBBX;
	double deltaY = (MaxY - MinY) / (double)numOfSubOBBY;
	// Area based descriptor
	for (int m = 0; m < pcl_TransformedPoints->size(); m++)
	{
		int index_X = (pcl_TransformedPoints->points[m].x - MinX) / deltaX;
		int index_Y = (pcl_TransformedPoints->points[m].y - MinY) / deltaY;
		int OBB_index = index_Y*numOfSubOBBX + index_X;
		double p_Descriptor[3];
		if (OBB_index < NumOfSubOBB)
		{
		
			Descriptor.points[OBB_index + 1].x = 0;
			Descriptor.points[OBB_index + 1].y = 0;
			Descriptor.points[OBB_index + 1].z += 1;
		
			if (desMethd[0] == 1 || desMethd[1] == 1)
			{
				CenterSubObb->points[OBB_index].x += pcl_TransformedPoints->points[m].x;
				CenterSubObb->points[OBB_index].y += pcl_TransformedPoints->points[m].y;
				CenterSubObb->points[OBB_index].z += pcl_TransformedPoints->points[m].z;
			}
		}
	}

	// Distant descriptor
	//offlineFile << "Center:";
	if (desMethd[0] == 1 || desMethd[1] == 1)
	{
		for (int i = 0; i < CenterSubObb->size(); i++)
		{
			if (Descriptor.points[i + 1].z == 0) continue;
			CenterSubObb->points[i].x /= Descriptor.points[i + 1].z;
			CenterSubObb->points[i].y /= Descriptor.points[i + 1].z;
			CenterSubObb->points[i].z /= Descriptor.points[i + 1].z;
			//offlineFile << CenterSubObb->points[i].x << " " << CenterSubObb->points[i].y << " " << CenterSubObb->points[i].z << " ";
		}
	}
	//offlineFile << "\n";

	// Normal vector descriptor
	pcl::PointCloud<pcl::PointXYZ>::Ptr Point_Cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Surface_Cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Indicates(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	if (desMethd[0] == 1)
	{
		pcl::copyPointCloud(Input, *Surface_Cloud);
		for (int i = 0; i < CenterSubObb->size(); i++)
		{
			if (Descriptor.points[i + 1].z < 3)
			{
				Indicates->push_back(pcl::PointXYZ(0, 0, 0));  continue;
			}
			Point_Cloud->push_back(pcl::PointXYZ(CenterSubObb->points[i].x, CenterSubObb->points[i].y, CenterSubObb->points[i].z));
			Indicates->push_back(pcl::PointXYZ(Point_Cloud->size() - 1, 1, 1));
		}

		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud(Point_Cloud);
		ne.setSearchSurface(Surface_Cloud);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		ne.setSearchMethod(tree);
		ne.setRadiusSearch(SubOBB_Size);
		ne.compute(*normals);
	}
	//offlineFile << "Descriptor: ";
	for (int i = 0; i < CenterSubObb->size(); i++)
	{
		int idex_X = i % numOfSubOBBX;
		int idex_Y = i / numOfSubOBBX;
		int normalCount = 0; int normalCenter = 0;
		for (int j = -1; j < 2; j += 2)
		for (int k = -1; k < 2; k += 2)
		{
			if (j == 0 & k == 0) continue;
			if (idex_X + j >= 0 & idex_Y + k >= 0 & idex_X + j < numOfSubOBBX & idex_Y + k < numOfSubOBBY)
			{
				int idex = (idex_Y + k) * numOfSubOBBX + (idex_X + j);
				// Distant descriptor
				if (desMethd[1] == 1)
				{
					if (Descriptor.points[i + 1].z == 0 || Descriptor.points[idex + 1].z == 0) continue;
					double sqdist = (CenterSubObb->points[i].x - CenterSubObb->points[idex].x) * (CenterSubObb->points[i].x - CenterSubObb->points[idex].x) +
						(CenterSubObb->points[i].y - CenterSubObb->points[idex].y) * (CenterSubObb->points[i].y - CenterSubObb->points[idex].y) +
						(CenterSubObb->points[i].z - CenterSubObb->points[idex].z) * (CenterSubObb->points[i].z - CenterSubObb->points[idex].z);
					Descriptor.points[i + 1].y += sqdist;
					normalCenter++;
				}
				// Normal vector descriptor
				if (desMethd[0] == 1)
				{
					if (Indicates->points[i].x == 0 & Indicates->points[i].y == 0 & Indicates->points[i].z == 0) continue;
					if (Indicates->points[idex].x == 0 & Indicates->points[idex].y == 0 & Indicates->points[idex].z == 0) continue;
					if (std::isnan(normals->points[Indicates->points[i].x].normal_x) || std::isnan(normals->points[Indicates->points[idex].x].normal_x)) continue;
					normalCount++;
					double dot_product = normals->points[Indicates->points[i].x].normal_x * normals->points[Indicates->points[idex].x].normal_x +
						normals->points[Indicates->points[i].x].normal_y * normals->points[Indicates->points[idex].x].normal_y +
						normals->points[Indicates->points[i].x].normal_z * normals->points[Indicates->points[idex].x].normal_z;
					double lengVector1 = normals->points[Indicates->points[i].x].normal_x * normals->points[Indicates->points[i].x].normal_x +
						normals->points[Indicates->points[i].x].normal_y * normals->points[Indicates->points[i].x].normal_y +
						normals->points[Indicates->points[i].x].normal_z * normals->points[Indicates->points[i].x].normal_z;
					double lengVector2 = normals->points[Indicates->points[idex].x].normal_x * normals->points[Indicates->points[idex].x].normal_x +
						normals->points[Indicates->points[idex].x].normal_y * normals->points[Indicates->points[idex].x].normal_y +
						normals->points[Indicates->points[idex].x].normal_z * normals->points[Indicates->points[idex].x].normal_z;
					double cosNorl = abs(dot_product / (sqrt(lengVector1) * sqrt(lengVector2)));
					Descriptor.points[i + 1].x += cosNorl;
				}
			}
		}
		if (normalCount != 0) Descriptor.points[i + 1].x /= (double)normalCount;
		if (normalCenter != 0) Descriptor.points[i + 1].y /= (double)normalCenter;
		//offlineFile << Descriptor.points[i + 1].x << " " << Descriptor.points[i + 1].y << " " << Descriptor.points[i + 1].z << " ";
	}
	//offlineFile << "\n";

	subobbAreaMax = 0;
	for (int i = 1; i < Descriptor.size(); i++)
	{
		if (Descriptor.points[i].z > subobbAreaMax) subobbAreaMax = Descriptor.points[i].z;
	}
}

void cinspection::OBB_Descriptor_Nonseg(pcl::PointCloud<pcl::PointXYZRGB> &Input, pcl::PointCloud<pcl::PointXYZRGB> &Descriptor, double &subobbAreaMax)
{
	int NumOfPoints = Input.size();
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_TransformedPoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(Input, *pcl_TransformedPoints);
	

	// Find Max Min X Y Z
	double MaxX = -1 * DBL_MAX;
	double MinX = DBL_MAX;
	double MaxY = -1 * DBL_MAX;
	double MinY = DBL_MAX;
	for (int i = 0; i < pcl_TransformedPoints->size(); i++)
	{
		if (pcl_TransformedPoints->points[i].x > MaxX) MaxX = pcl_TransformedPoints->points[i].x;
		if (pcl_TransformedPoints->points[i].x < MinX) MinX = pcl_TransformedPoints->points[i].x;
		if (pcl_TransformedPoints->points[i].y > MaxY) MaxY = pcl_TransformedPoints->points[i].y;
		if (pcl_TransformedPoints->points[i].y < MinY) MinY = pcl_TransformedPoints->points[i].y;
	}
	int numOfSubOBBX = (MaxX - MinX) / (double)SubOBB_Size + 0.5;
	int numOfSubOBBY = (MaxY - MinY) / (double)SubOBB_Size + 0.5;
	int NumOfSubOBB = numOfSubOBBX*numOfSubOBBY;

	// Initializing for descriptor
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CenterSubObb(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (Descriptor.size()) Descriptor.clear();
	for (int OBBindex = 0; OBBindex < NumOfSubOBB + 1; OBBindex++)
	{
		pcl::PointXYZRGB point;
		point.x = 0;
		point.y = 0;
		point.z = 0;
		Descriptor.push_back(point);
		if (OBBindex < NumOfSubOBB) CenterSubObb->push_back(point);
	}
	Descriptor.points[0].x = numOfSubOBBX;
	Descriptor.points[0].y = numOfSubOBBY;
	Descriptor.points[0].z = 1;

	if (SubOBB_Size == 0) return;

	// Area based descriptor
	for (int m = 0; m < pcl_TransformedPoints->size(); m++)
	{
		int index_X = (pcl_TransformedPoints->points[m].x - MinX) / SubOBB_Size;
		int index_Y = (pcl_TransformedPoints->points[m].y - MinY) / SubOBB_Size;
		int OBB_index = index_Y*numOfSubOBBX + index_X;
		double p_Descriptor[3];
		if (OBB_index < NumOfSubOBB)
		{
			Descriptor.points[OBB_index + 1].x = 0;
			Descriptor.points[OBB_index + 1].y = 0;
			Descriptor.points[OBB_index + 1].z += 1;
		
			if (desMethd[0] == 1 || desMethd[1] == 1)
			{
				CenterSubObb->points[OBB_index].x += pcl_TransformedPoints->points[m].x;
				CenterSubObb->points[OBB_index].y += pcl_TransformedPoints->points[m].y;
				CenterSubObb->points[OBB_index].z += pcl_TransformedPoints->points[m].z;
			}
		}
	}

	subobbAreaMax = 0;
	for (int i = 1; i < Descriptor.size(); i++)
	{
		if (Descriptor.points[i].z > subobbAreaMax) subobbAreaMax = Descriptor.points[i].z;
	}

	// Distant descriptor
	//offlineFile << "Center" << "\n";
	if (desMethd[1] == 1 || desMethd[0] == 1)
	{
		for (int i = 0; i < CenterSubObb->size(); i++)
		{
			if (Descriptor.points[i + 1].z == 0) continue;
			CenterSubObb->points[i].x /= Descriptor.points[i + 1].z;
			CenterSubObb->points[i].y /= Descriptor.points[i + 1].z;
			CenterSubObb->points[i].z /= Descriptor.points[i + 1].z;
			//offlineFile << CenterSubObb->points[i].x << " " << CenterSubObb->points[i].y << " " << CenterSubObb->points[i].z << " ";
		}
	}
	//offlineFile << "\n";

	// Normal vector descriptor
	pcl::PointCloud<pcl::PointXYZ>::Ptr Point_Cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Surface_Cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Indicates(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	if (desMethd[0] == 1)
	{
		pcl::copyPointCloud(Input, *Surface_Cloud);
		for (int i = 0; i < CenterSubObb->size(); i++)
		{
			if (Descriptor.points[i + 1].z < 10)
			{
				Indicates->push_back(pcl::PointXYZ(0, 0, 0));  continue;
			}
			Point_Cloud->push_back(pcl::PointXYZ(CenterSubObb->points[i].x, CenterSubObb->points[i].y, CenterSubObb->points[i].z));
			Indicates->push_back(pcl::PointXYZ(Point_Cloud->size() - 1, 1, 1));
		}

		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud(Point_Cloud);
		ne.setSearchSurface(Surface_Cloud);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		ne.setSearchMethod(tree);
		ne.setRadiusSearch(SubOBB_Size);
		ne.compute(*normals);
	}
	offlineFile << "VirCam Descriptor: ";
	for (int i = 0; i < CenterSubObb->size(); i++)
	{
		int idex_X = i % numOfSubOBBX;
		int idex_Y = i / numOfSubOBBX;
		int normalCount = 0; int normalCenter = 0;
		for (int j = -1; j < 2; j+=2)
		for (int k = -1; k < 2; k += 2)
		{
			if (j == 0 & k == 0) continue;
			if (idex_X + j >= 0 & idex_Y + k >= 0 & idex_X + j < numOfSubOBBX & idex_Y + k < numOfSubOBBY)
			{
				int idex = (idex_Y + k) * numOfSubOBBX + (idex_X + j);
				// Distant descriptor
				if (desMethd[1] == 1)
				{
					if (Descriptor.points[i + 1].z == 0 || Descriptor.points[idex + 1].z == 0) continue;
					double sqdist = (CenterSubObb->points[i].x - CenterSubObb->points[idex].x) * (CenterSubObb->points[i].x - CenterSubObb->points[idex].x) +
						(CenterSubObb->points[i].y - CenterSubObb->points[idex].y) * (CenterSubObb->points[i].y - CenterSubObb->points[idex].y) +
						(CenterSubObb->points[i].z - CenterSubObb->points[idex].z) * (CenterSubObb->points[i].z - CenterSubObb->points[idex].z);
					Descriptor.points[i + 1].y += sqdist;
					normalCenter++;
				}
				// Normal vector descriptor
				if (desMethd[0] == 1)
				{
					if (Indicates->points[i].x == 0 & Indicates->points[i].y == 0 & Indicates->points[i].z == 0) continue;
					if (Indicates->points[idex].x == 0 & Indicates->points[idex].y == 0 & Indicates->points[idex].z == 0) continue;
					if (std::isnan(normals->points[Indicates->points[i].x].normal_x) || std::isnan(normals->points[Indicates->points[idex].x].normal_x)) continue;
					normalCount++;
					double dot_product = normals->points[Indicates->points[i].x].normal_x * normals->points[Indicates->points[idex].x].normal_x +
						                 normals->points[Indicates->points[i].x].normal_y * normals->points[Indicates->points[idex].x].normal_y +
						                 normals->points[Indicates->points[i].x].normal_z * normals->points[Indicates->points[idex].x].normal_z;
					double lengVector1 = normals->points[Indicates->points[i].x].normal_x * normals->points[Indicates->points[i].x].normal_x +
						                 normals->points[Indicates->points[i].x].normal_y * normals->points[Indicates->points[i].x].normal_y +
						                 normals->points[Indicates->points[i].x].normal_z * normals->points[Indicates->points[i].x].normal_z;
					double lengVector2 = normals->points[Indicates->points[idex].x].normal_x * normals->points[Indicates->points[idex].x].normal_x +
						                 normals->points[Indicates->points[idex].x].normal_y * normals->points[Indicates->points[idex].x].normal_y +
						                 normals->points[Indicates->points[idex].x].normal_z * normals->points[Indicates->points[idex].x].normal_z;
					double cosNorl = abs(dot_product / (sqrt(lengVector1) * sqrt(lengVector2)));
					Descriptor.points[i + 1].x += cosNorl;
				}
			}
		}
		if (normalCount != 0) Descriptor.points[i + 1].x /= (double)normalCount;
		if (normalCenter != 0) Descriptor.points[i + 1].y /= (double)normalCenter;
		offlineFile << Descriptor.points[i + 1].x << " " << Descriptor.points[i + 1].y << " " << Descriptor.points[i + 1].z << " ";
	}
	offlineFile << "\n";
}

void cinspection::Opt_OBB_Descriptor(pcl::PointCloud<pcl::PointXYZRGB> &Input, pcl::PointCloud<pcl::PointXYZRGB> &Descriptor, int numOfSubOBB[3], double *maxFeatures)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Descriptor_Buf(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int RotaX = 0; RotaX < 360; RotaX += 180)
	for (int RotaZ = 0; RotaZ < 360; RotaZ += 90)
	{
		Eigen::Affine3f transformMatrix = Eigen::Affine3f::Identity();
		transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (RotaX / (double)180), Eigen::Vector3f::UnitX()));
		transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (RotaZ / (double)180), Eigen::Vector3f::UnitZ()));
		pcl::transformPointCloud(Input, *Cloud, transformMatrix);

		double subobbAreaMax_obj;
		OBB_Descriptor(*Cloud, *Descriptor_Buf, numOfSubOBB[0], numOfSubOBB[1], subobbAreaMax_obj);
		maxFeatures[0] = subobbAreaMax_obj;
		maxFeatures[1] = 0;
		maxFeatures[2] = 0;

		pcl::PointXYZRGB point;
		point.x = RotaX;
		point.y = 0;
		point.z = RotaZ;
		Descriptor.push_back(point);
		Descriptor.operator+=(*Descriptor_Buf);
	}
}

void cinspection::Nonseg_OBB_Descriptor(pcl::PointCloud<pcl::PointXYZRGB> &Input, pcl::PointCloud<pcl::PointXYZRGB> &Descriptor, double ViewPoint[3], int inCreZ)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud_Buf(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Descriptor_Buf(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int RotZ = 0; RotZ < 90; RotZ += inCreZ)
	{
		Eigen::Affine3f transformMatrix = Eigen::Affine3f::Identity();
		transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (RotZ / (double)180), Eigen::Vector3f::UnitZ()));
		pcl::transformPointCloud(Input, *Cloud_Buf, transformMatrix);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (int RotaX = 0; RotaX < 360; RotaX += 180)
		for (int RotaZ = 0; RotaZ < 360; RotaZ += 90)
		{
			transformMatrix = Eigen::Affine3f::Identity();
			transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (RotaX / (double)180), Eigen::Vector3f::UnitX()));
			transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (RotaZ / (double)180), Eigen::Vector3f::UnitZ()));
			pcl::transformPointCloud(*Cloud_Buf, *Cloud, transformMatrix);

			double subobbAreaMax_obj;
			OBB_Descriptor_Nonseg(*Cloud, *Descriptor_Buf, subobbAreaMax_obj);

			pcl::PointXYZRGB point;
			point.x = ViewPoint[0];
			point.y = ViewPoint[1];
			point.z = ViewPoint[2];
			Descriptor.push_back(point);

			point.x = RotaX;
			point.y = RotZ;
			point.z = RotaZ;
			Descriptor.push_back(point);

			point.x = subobbAreaMax_obj;
			point.y = 0;
			point.z = 0;
			Descriptor.push_back(point);

			Descriptor.operator+=(*Descriptor_Buf);
			point.x = -1;
			point.y = -1;
			point.z = -1;
			Descriptor.push_back(point);
		}
	}
	
}

void cinspection::OBB_matching_scence_model(pcl::PointCloud<pcl::PointXYZRGB> &ObjCloud, pcl::PointCloud<pcl::PointXYZRGB> &ModelDescrip, 
	double OBBmodelMid, double OBBscenceMax, double subobbAreaMax_mod, double objNumOfSubOBBX, double objNumOfSubOBBY, double modelNumOfSubOBBX,
	 double modelNumOfSubOBBY, double &NCCmax, int countView)
{
	reportCount++;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ObjDescrip(new pcl::PointCloud<pcl::PointXYZRGB>);
	NCCmax = -1;

	cv::Mat modelimg(modelNumOfSubOBBY, modelNumOfSubOBBX, CV_8UC3, cv::Scalar(0, 0, 0));
	cv::Mat gray_modelimg;
	cv::cvtColor(modelimg, gray_modelimg, CV_BGR2GRAY);
	
	for (int i = 3; i < ModelDescrip.size(); i++)
	{
		double p[3];
		p[0] = ModelDescrip.points[i].x;
		p[1] = ModelDescrip.points[i].y;
		p[2] = ModelDescrip.points[i].z;
		gray_modelimg.at<uchar>(p[1], p[0]) = p[2];
	}
	cv::normalize(gray_modelimg, gray_modelimg, 0, 255, cv::NORM_MINMAX, CV_8UC1);

	/*// Report and Debug
	string name;
	name.append("Report/model_extract_");
	QString num = QString::number(reportCount);
	name.append(num.toUtf8().constData());
	name.append(".jpg");
	cv::imwrite(name, gray_modelimg);*/

	// 8 corners of OBB can be as OBB origin, but here kz = 1, so possibilities is 4
	// OX and OY can be excahnged, so every corner we have 2 possibilites
	// Therefore, total posibilites is 8
	for (int RotaX = 0; RotaX < 360; RotaX += 180)
	for (int RotaZ = 0; RotaZ < 360; RotaZ += 90)
	{
		Eigen::Affine3f transformMatrix = Eigen::Affine3f::Identity();
		transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (RotaX / (double)180), Eigen::Vector3f::UnitX()));
		transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (RotaZ / (double)180), Eigen::Vector3f::UnitZ()));
		pcl::transformPointCloud(ObjCloud, *Cloud, transformMatrix);
		
		double subobbAreaMax_obj;
		OBB_Descriptor(*Cloud, *ObjDescrip, objNumOfSubOBBX, objNumOfSubOBBY, subobbAreaMax_obj);
		double adaptResol = subobbAreaMax_mod / double(subobbAreaMax_obj);
		cv::Mat objimg(objNumOfSubOBBY, objNumOfSubOBBX, CV_8UC3, cv::Scalar(0, 0, 0));
		cv::Mat gray_objimg;
		cv::cvtColor(objimg, gray_objimg, CV_BGR2GRAY);
		for (int i = 0; i < ObjDescrip->size(); i++)
		{
			double p[3];
			p[0] = ObjDescrip->points[i].x;
			p[1] = ObjDescrip->points[i].y;
			p[2] = ObjDescrip->points[i].z;
			gray_objimg.at<uchar>(p[1], p[0]) = p[2] * adaptResol;
		}
		cv::normalize(gray_objimg, gray_objimg, 0, 255, cv::NORM_MINMAX, CV_8UC1);

		/*// Report and Debug
		string name;
		name.append("Report/object_");
		QString num = QString::number(reportCount);
		name.append(num.toUtf8().constData());
		num = QString::number(Rota);
		name.append(num.toUtf8().constData());
		name.append(".jpg");
		cv::imwrite(name, gray_objimg);*/

		cv::Point minloc, maxloc;
		double minval, maxval;
		cv::Mat ftmp;// = cv::Mat(gray_modelimg.rows - gray_objimg.rows + 1, gray_modelimg.cols - gray_objimg.cols + 1, CV_32FC1 );
		cv::matchTemplate(gray_modelimg, gray_objimg, ftmp, CV_TM_CCOEFF_NORMED);//CV_TM_CCORR_NORMED CV_TM_CCOEFF_NORMED
		cv::minMaxLoc(ftmp, &minval, &maxval, &minloc, &maxloc, cv::Mat());
		if (NCCmax < maxval)
		{
			NCCmax = maxval;
		}
		// Report and Debug
		//offlineFile << "NCC " << reportCount << "_" << RotaX << "_" << RotaZ << ": " << maxval << "\n";
	}
}

void cinspection::SubOBB_matching_scence_model(pcl::PointCloud<pcl::PointXYZRGB> &ObjCloud, pcl::PointCloud<pcl::PointXYZRGB> &ModelDescrip,
	double OBBmodelMid, double OBBscenceMax, double subobbAreaMax_mod, double objNumOfSubOBBX, double objNumOfSubOBBY, double modelNumOfSubOBBX,
	double modelNumOfSubOBBY, double &StartX, double &StartY, double &BestRota, double &NCCmax, int countView)
{
	reportCount++;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ObjDescrip(new pcl::PointCloud<pcl::PointXYZRGB>);
	NCCmax = -1;

	cv::Mat modelimg(modelNumOfSubOBBY, modelNumOfSubOBBX, CV_8UC3, cv::Scalar(0, 0, 0));
	cv::Mat gray_modelimg;
	cv::cvtColor(modelimg, gray_modelimg, CV_BGR2GRAY);

	for (int i = 3; i < ModelDescrip.size(); i++)
	{
		double p[3];
		/*p[0] = ModelDescrip.points[i].x;
		p[1] = ModelDescrip.points[i].y;*/
		p[0] = (i-3) % (int)modelNumOfSubOBBX;
		p[1] = (i-3) / (int)modelNumOfSubOBBX;
		p[2] = ModelDescrip.points[i].z;
		//if (p[2] == 0) p[2] = 2 * subobbAreaMax_mod;
		gray_modelimg.at<uchar>(p[1], p[0]) = p[2];
	}
	cv::normalize(gray_modelimg, gray_modelimg, 0, 255, cv::NORM_MINMAX, CV_8UC1);

	/*// Report and Debug
	string name;
	name.append("Report/model_extract_");
	QString num = QString::number(reportCount);
	name.append(num.toUtf8().constData());
	name.append(".jpg");
	cv::imwrite(name, gray_modelimg);*/

	// 8 corners of OBB can be as OBB origin, but here kz = 1, so possibilities is 4
	// OX and OY can be exchanged, so every corner we have 2 possibilites
	// Therefore, total posibilites is 8
	for (int RotaX = 0; RotaX < 360; RotaX += 180)
	for (int RotaZ = 0; RotaZ < 360; RotaZ += 90)
	{
		Eigen::Affine3f transformMatrix = Eigen::Affine3f::Identity();
		transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (RotaX / (double)180), Eigen::Vector3f::UnitX()));
		transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (RotaZ / (double)180), Eigen::Vector3f::UnitZ()));
		pcl::transformPointCloud(ObjCloud, *Cloud, transformMatrix);
	
		double subobbAreaMax_obj;
		OBB_Descriptor(*Cloud, *ObjDescrip, objNumOfSubOBBX, objNumOfSubOBBY, subobbAreaMax_obj);
		double adaptResol = subobbAreaMax_mod / double(subobbAreaMax_obj);
		cv::Mat objimg(objNumOfSubOBBY, objNumOfSubOBBX, CV_8UC3, cv::Scalar(0, 0, 0));
		cv::Mat gray_objimg;
		cv::cvtColor(objimg, gray_objimg, CV_BGR2GRAY);
		for (int i = 0; i < ObjDescrip->size(); i++)
		{
			double p[3];
			/*p[0] = ObjDescrip->points[i].x;
			p[1] = ObjDescrip->points[i].y;*/
			p[0] = i % (int)objNumOfSubOBBX;
			p[1] = i / (int)objNumOfSubOBBX;
			p[2] = ObjDescrip->points[i].z;
			gray_objimg.at<uchar>(p[1], p[0]) = p[2] * adaptResol;
		}
		cv::normalize(gray_objimg, gray_objimg, 0, 255, cv::NORM_MINMAX, CV_8UC1);

		/*// Report and Debug
		string name;
		name.append("Report/object_");
		QString num = QString::number(reportCount);
		name.append(num.toUtf8().constData());
		num = QString::number(Rota);
		name.append(num.toUtf8().constData());
		name.append(".jpg");
		cv::imwrite(name, gray_objimg);*/

		cv::Point minloc, maxloc;
		double minval, maxval;
		cv::Mat ftmp;// = cv::Mat(gray_modelimg.rows - gray_objimg.rows + 1, gray_modelimg.cols - gray_objimg.cols + 1, CV_32FC1 );
		cv::matchTemplate(gray_modelimg, gray_objimg, ftmp, CV_TM_CCOEFF_NORMED);//CV_TM_CCORR_NORMED CV_TM_CCOEFF_NORMED
		cv::minMaxLoc(ftmp, &minval, &maxval, &minloc, &maxloc, cv::Mat());
		if (NCCmax < maxval)
		{
			NCCmax = maxval;
			BestRota = RotaZ;
			StartX = maxloc.x;
			StartY = maxloc.y;
		}
		// Report and Debug
		//offlineFile << "NCC " << reportCount << "_" << RotaX << "_" << RotaZ << ": " << maxval << "\n";
		//offlineFile << maxval << "\n";
	}
}

bool cinspection::SubOBB_matching_scence_model_Opt(pcl::PointCloud<pcl::PointXYZRGB> &ObjectCloud, int *numOfSubOBB_object, pcl::PointCloud<pcl::PointXYZ> &PCLBestView,
	pcl::PointCloud<pcl::PointXYZ> &PCLBestRota, pcl::PointCloud<pcl::PointXYZ> &PCLBestNCC, pcl::PointCloud<pcl::PointXYZ> &PCLBestNumOfSubOBB)
{
	if (ObjectCloud.size() == 0) return 1;
	if (PCLBestView.size()) PCLBestView.clear();
	if (PCLBestRota.size()) PCLBestRota.clear();
	if (PCLBestNCC.size()) PCLBestNCC.clear();
	if (PCLBestNumOfSubOBB.size()) PCLBestNumOfSubOBB.clear();

	double MaxFe_Obj[3];
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Descrip_Obj(new pcl::PointCloud<pcl::PointXYZRGB>); 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Descrip_Mod(new pcl::PointCloud<pcl::PointXYZRGB>);
	//OBB_Descriptor_Nonseg(ObjectCloud, *Descrip_Obj, FeaturesMax_Obj[0]);

	// Read Descriptor of model
	ReadPCLDescriptor_Opt(*Descrip_Mod);

	double BestView[3]; double BestRota[3]; double BestNCC[3]; double BestNumOfSubOBB[3]; BestNCC[2] = -1; int CountRota = 0;
	double maxNCC = -1; double ViePt[3]; double Rota[3]; double MaxFe_Mod[3]; int NumOfSubOBB_mod[3]; int Ire = 0;

	while (Ire < Descrip_Mod->size() - 1)
	{
		if (Ire == 0 || (Descrip_Mod->points[Ire].x == -1 & Descrip_Mod->points[Ire].y == -1 & Descrip_Mod->points[Ire].z == -1))
		{
			if (Ire == 0) Ire = -1;
			Ire++; ViePt[0] = Descrip_Mod->points[Ire].x; ViePt[1] = Descrip_Mod->points[Ire].y; ViePt[2] = Descrip_Mod->points[Ire].z;
			Ire++; Rota[0] = Descrip_Mod->points[Ire].x; Rota[1] = Descrip_Mod->points[Ire].y; Rota[2] = Descrip_Mod->points[Ire].z;
			Ire++; MaxFe_Mod[0] = Descrip_Mod->points[Ire].x; MaxFe_Mod[1] = Descrip_Mod->points[Ire].y; MaxFe_Mod[2] = Descrip_Mod->points[Ire].z;
			Ire++; NumOfSubOBB_mod[0] = Descrip_Mod->points[Ire].x; NumOfSubOBB_mod[1] = Descrip_Mod->points[Ire].y; NumOfSubOBB_mod[2] = Descrip_Mod->points[Ire].z; //
			//offlineFile << "\n";
			//offlineFile << "Ire: " << Ire << "\n";
			//offlineFile << "ViewPoint: " << ViePt[0] << " " << ViePt[1] << " " << ViePt[2] << "\n";
			//offlineFile << "Rota: " << Rota[0] << " " << Rota[1] << " " << Rota[2] << "\n";
			//offlineFile << "MaxFe_Mod: " << MaxFe_Mod[0] << " " << MaxFe_Mod[1] << " " << MaxFe_Mod[2] << "\n";
			//offlineFile << "NumOfSubOBB_mod: " << NumOfSubOBB_mod[0] << " " << NumOfSubOBB_mod[1] << " " << NumOfSubOBB_mod[2] << "\n";	
		}

		int totalOBB_mod = NumOfSubOBB_mod[0] * NumOfSubOBB_mod[1];
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr DesOneView_Mod(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (int i = 0; i < totalOBB_mod; i++)
		{
			pcl::PointXYZRGB point;
			Ire++;
			point.x = Descrip_Mod->points[Ire].x; point.y = Descrip_Mod->points[Ire].y; point.z = Descrip_Mod->points[Ire].z;
			DesOneView_Mod->push_back(point);
			//offlineFile << point.x << " " << point.y << " " << point.z << " ";
		}
		//offlineFile << "\n";
		Ire++;

		int numOfSubOBB_Obj[3]; numOfSubOBB_Obj[0] = numOfSubOBB_object[0]; numOfSubOBB_Obj[1] = numOfSubOBB_object[1]; numOfSubOBB_Obj[2] = 1;
		if (numOfSubOBB_Obj[0] > NumOfSubOBB_mod[0]) numOfSubOBB_Obj[0] = NumOfSubOBB_mod[0];
		if (numOfSubOBB_Obj[1] > NumOfSubOBB_mod[1]) numOfSubOBB_Obj[1] = NumOfSubOBB_mod[1];
		if (numOfSubOBB_Obj[0] < 1) numOfSubOBB_Obj[0] = 1;
		if (numOfSubOBB_Obj[1] < 1) numOfSubOBB_Obj[1] = 1;
		OBB_Descriptor_Opt(ObjectCloud, *Descrip_Obj, numOfSubOBB_Obj, MaxFe_Obj[0]);

		cv::Mat modelimg(NumOfSubOBB_mod[1], NumOfSubOBB_mod[0], CV_8UC3, cv::Scalar(0, 0, 0));
		cv::Mat gray_modelimg;
		cv::cvtColor(modelimg, gray_modelimg, CV_BGR2GRAY);
		for (int j = 0; j < DesOneView_Mod->size(); j++)
		{
			double p[3];
			p[0] = j % (int)NumOfSubOBB_mod[0];
			p[1] = j / (int)NumOfSubOBB_mod[0];
			if (desMethd[0] == 1) p[2] = 255 * DesOneView_Mod->points[j].x;
			else if (desMethd[1] == 1) p[2] = DesOneView_Mod->points[j].y;
			else if (desMethd[2] == 1) p[2] = DesOneView_Mod->points[j].z;
			gray_modelimg.at<uchar>(p[1], p[0]) = p[2];
		}
		cv::normalize(gray_modelimg, gray_modelimg, 0, 255, cv::NORM_MINMAX, CV_8UC1);

		double adaptResol = MaxFe_Mod[0] / double(MaxFe_Obj[0]);
		cv::Mat objimg(numOfSubOBB_Obj[1], numOfSubOBB_Obj[0], CV_8UC3, cv::Scalar(0, 0, 0));
		cv::Mat gray_objimg;
		cv::cvtColor(objimg, gray_objimg, CV_BGR2GRAY);
		for (int k = 1; k < Descrip_Obj->size(); k++)
		{
			double p[3];
			p[0] = (k-1) % (int)numOfSubOBB_Obj[0];
			p[1] = (k-1) / (int)numOfSubOBB_Obj[0];
			if (desMethd[0] == 1) p[2] = 255 * Descrip_Obj->points[k].x;
			else if (desMethd[1] == 1) p[2] = Descrip_Obj->points[k].y;
			else if (desMethd[2] == 1) p[2] = adaptResol * Descrip_Obj->points[k].z;
			gray_objimg.at<uchar>(p[1], p[0]) = p[2];
		}
		cv::normalize(gray_objimg, gray_objimg, 0, 255, cv::NORM_MINMAX, CV_8UC1);

		cv::Point minloc, maxloc;
		double minval, maxval;
		cv::Mat ftmp;// = cv::Mat(gray_modelimg.rows - gray_objimg.rows + 1, gray_modelimg.cols - gray_objimg.cols + 1, CV_32FC1 );
		cv::matchTemplate(gray_modelimg, gray_objimg, ftmp, CV_TM_CCOEFF_NORMED);//CV_TM_CCORR_NORMED   CV_TM_CCOEFF_NORMED
		cv::minMaxLoc(ftmp, &minval, &maxval, &minloc, &maxloc, cv::Mat());
		//offlineFile << "maxval_" << CountRota << " :" << maxval << "\n";

		if (BestNCC[2] < maxval)
		{
			BestNCC[0] = maxloc.x;
			BestNCC[1] = maxloc.y;
			BestNCC[2] = maxval;
			BestRota[0] = Rota[0];
			BestRota[1] = Rota[1];
			BestRota[2] = Rota[2];
			BestView[0] = ViePt[0];
			BestView[1] = ViePt[1];
			BestView[2] = ViePt[2];
			BestNumOfSubOBB[0] = NumOfSubOBB_mod[0];
			BestNumOfSubOBB[1] = NumOfSubOBB_mod[1];
			BestNumOfSubOBB[2] = 1;
		}
		if (maxNCC < BestNCC[2]) maxNCC = BestNCC[2];

		if (CountRota == 7)
		{
			PCLBestNCC.push_back(pcl::PointXYZ(BestNCC[0], BestNCC[1], BestNCC[2]));
			PCLBestView.push_back(pcl::PointXYZ(BestView[0], BestView[1], BestView[2]));
			PCLBestRota.push_back(pcl::PointXYZ(BestRota[0], BestRota[1], BestRota[2]));
			PCLBestNumOfSubOBB.push_back(pcl::PointXYZ(BestNumOfSubOBB[0], BestNumOfSubOBB[1], BestNumOfSubOBB[2]));
			//offlineFile << "BestNCC: " << BestNCC[2] << "\n";
			BestNCC[2] = -1;
			CountRota = 0;
		}
		else CountRota++;

		// Report and Debug
	}
	//offlineFile << "maxNCC: " << maxNCC << "\n";
	PCLBestNCC.push_back(pcl::PointXYZ(0, 0, maxNCC));
	if (PCLBestNCC.size() == 0) return 1;
	return 0;
}

bool cinspection::SubOBB_matching_model_scene_nonSeg(pcl::PointCloud<pcl::PointXYZRGB> &SceneCloud, int *numOfSubOBB_Scene, pcl::PointCloud<pcl::PointXYZ>
	&PCLBestView, pcl::PointCloud<pcl::PointXYZ> &PCLBestRota_mod, pcl::PointCloud<pcl::PointXYZ> &PCLBestNCC, pcl::PointCloud<pcl::PointXYZ> &PCLBestNumOfSubOBB)
{
	QElapsedTimer timer;
	if (SceneCloud.size() == 0) return 1;
	if (PCLBestView.size()) PCLBestView.clear();
	if (PCLBestRota_mod.size()) PCLBestRota_mod.clear();
	if (PCLBestNCC.size()) PCLBestNCC.clear();
	if (PCLBestNumOfSubOBB.size()) PCLBestNumOfSubOBB.clear();

	double FeaturesMax_Obj[3];
	double FeaturesMax_Mod[3];
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Descrip_Scene(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Descrip_Mod(new pcl::PointCloud<pcl::PointXYZRGB>);
	OBB_Descriptor(SceneCloud, *Descrip_Scene, numOfSubOBB_Scene[0], numOfSubOBB_Scene[1], FeaturesMax_Obj[0]);
	
	// Read Descriptor of model
	timer.start();
	ReadPCLDescriptor_Nonseg(*Descrip_Mod);

	pcl::PointXYZ maxNCC;
	maxNCC.z = -1;
	int numOfSubOBB_Sce[3];
	numOfSubOBB_Sce[0] = numOfSubOBB_Scene[0];
	numOfSubOBB_Sce[1] = numOfSubOBB_Scene[1];
	numOfSubOBB_Sce[2] = 1;
	
	double ViePt[3]; //ViewPoint
	double Rota[3]; //Rotation
	double MaxFe_Mod[3]; //Max model features 
	int NumOfSubOBB_mod[3];
	int Ire = 0;
	
	while (Ire < Descrip_Mod->size() - 1)
	{
		if (Ire == 0 || (Descrip_Mod->points[Ire].x == -1 & Descrip_Mod->points[Ire].y == -1 & Descrip_Mod->points[Ire].z == -1))
		{
			if (Ire == 0) Ire = -1;
			Ire++; ViePt[0] = Descrip_Mod->points[Ire].x; ViePt[1] = Descrip_Mod->points[Ire].y; ViePt[2] = Descrip_Mod->points[Ire].z;
			Ire++; Rota[0] = Descrip_Mod->points[Ire].x; Rota[1] = Descrip_Mod->points[Ire].y; Rota[2] = Descrip_Mod->points[Ire].z;
			Ire++; MaxFe_Mod[0] = Descrip_Mod->points[Ire].x; MaxFe_Mod[1] = Descrip_Mod->points[Ire].y; MaxFe_Mod[2] = Descrip_Mod->points[Ire].z;
			Ire++; NumOfSubOBB_mod[0] = Descrip_Mod->points[Ire].x; NumOfSubOBB_mod[1] = Descrip_Mod->points[Ire].y; NumOfSubOBB_mod[2] = Descrip_Mod->points[Ire].z; //
			//offlineFile << "\n";
			//offlineFile << "Ire: " << Ire << "\n";
			//offlineFile << "ViewPoint: " << ViePt[0] << " " << ViePt[1] << " " << ViePt[2] << "\n";
			//offlineFile << "Rota: " << Rota[0] << " " << Rota[1] << " " << Rota[2] << "\n";
			//offlineFile << "MaxFe_Mod: " << MaxFe_Mod[0] << " " << MaxFe_Mod[1] << " " << MaxFe_Mod[2] << "\n";
			//offlineFile << "NumOfSubOBB_mod: " << NumOfSubOBB_mod[0] << " " << NumOfSubOBB_mod[1] << " " << NumOfSubOBB_mod[2] << "\n";	
		}
		int totalOBB_mod = NumOfSubOBB_mod[0] * NumOfSubOBB_mod[1];
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Des_Single_Mod(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (int i = 0; i < totalOBB_mod; i++)
		{
			pcl::PointXYZRGB point;
			Ire++;
			point.x = Descrip_Mod->points[Ire].x; point.y = Descrip_Mod->points[Ire].y; point.z = Descrip_Mod->points[Ire].z;
			Des_Single_Mod->push_back(point);
			offlineFile << point.x << " " << point.y << " " << point.z << " ";
		}
		offlineFile << "\n";
		Ire++;

		pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
		template_Matching_Nonseg(*Descrip_Scene, *Des_Single_Mod, *result, numOfSubOBB_Sce, NumOfSubOBB_mod);
		PCLBestNCC.push_back(result->points[result->size() - 1]);
		PCLBestView.push_back(pcl::PointXYZ(ViePt[0], ViePt[1], ViePt[2]));
		PCLBestRota_mod.push_back(pcl::PointXYZ(Rota[0], Rota[1], Rota[2]));
		PCLBestNumOfSubOBB.push_back(pcl::PointXYZ(NumOfSubOBB_mod[0], NumOfSubOBB_mod[1], NumOfSubOBB_mod[2]));
		if (maxNCC.z < result->points[result->size() - 1].z)
		{
			pcl::copyPoint(result->points[result->size() - 1], maxNCC);
		}
	}
	//offlineFile << "maxNCC: " << maxNCC.x << " " << maxNCC.y << " " << maxNCC.z << "\n";
	//offlineFile << "Time: " << timer.elapsed() << " ms" << "\n";
	return 0;
}

void cinspection::OBB_ExtractVitualPoints(pcl::PointCloud<pcl::PointXYZRGB> &pcl_PointClouds, pcl::PointCloud<pcl::PointXYZRGB> &ExtractedCloud, 
	 pcl::PointCloud<pcl::PointXYZRGB> &VirtualPoints, int NumOfSamplesX, double ratioZ)
{
	double MaxX = -1 * DBL_MAX;
	double MinX = DBL_MAX;
	double MaxY = -1 * DBL_MAX;
	double MinY = DBL_MAX;
	double MaxZ = -1 * DBL_MAX;
	double MinZ = DBL_MAX;

	for (int i = 0; i < pcl_PointClouds.size(); i++)
	{
		if (pcl_PointClouds.points[i].x > MaxX) MaxX = pcl_PointClouds.points[i].x;
		if (pcl_PointClouds.points[i].x < MinX) MinX = pcl_PointClouds.points[i].x;
		if (pcl_PointClouds.points[i].y > MaxY) MaxY = pcl_PointClouds.points[i].y;
		if (pcl_PointClouds.points[i].y < MinY) MinY = pcl_PointClouds.points[i].y;
		if (pcl_PointClouds.points[i].z > MaxZ) MaxZ = pcl_PointClouds.points[i].z;
		if (pcl_PointClouds.points[i].z < MinZ) MinZ = pcl_PointClouds.points[i].z;
	}

	int NumOfSamplesY = NumOfSamplesX*(double)(MaxY - MinY) / (MaxX - MinX);
	int NumOfSamples = NumOfSamplesX*NumOfSamplesY;

	vtkSmartPointer<vtkPoints>  vtk_MinMaxZValue = vtkSmartPointer<vtkPoints>::New();
	for (int OBBindex = 0; OBBindex < NumOfSamples; OBBindex++)
	{
		vtk_MinMaxZValue->InsertPoint(OBBindex, MaxZ, MinZ, 0);
	}
	// Find max of Z at each SubOBB
	double deltaX = (double)(MaxX - MinX) / NumOfSamplesX;
	double deltaY = (double)(MaxY - MinY) / NumOfSamplesY;
	for (int m = 0; m < pcl_PointClouds.size(); m++)
	{
		double p[3];
		p[0] = pcl_PointClouds.points[m].x;
		p[1] = pcl_PointClouds.points[m].y;
		p[2] = pcl_PointClouds.points[m].z;
		int index_X = (p[0] - MinX) / deltaX;
		int index_Y = (p[1] - MinY) / deltaY;
		int OBB_index = index_Y*NumOfSamplesX + index_X;
		double q[3];
		vtk_MinMaxZValue->GetPoint(OBB_index, q);
		if (p[2] > q[1]) vtk_MinMaxZValue->InsertPoint(OBB_index, q[0], p[2], 0);
		if (p[2] < q[0]) vtk_MinMaxZValue->InsertPoint(OBB_index, p[2], q[1], 0);
	}
	// Extract Points captured by Virtual Camera
	if (VirtualPoints.size()) VirtualPoints.clear();
	for (int m = 0; m < pcl_PointClouds.size(); m++)
	{
		double p[3];
		p[0] = pcl_PointClouds.points[m].x;
		p[1] = pcl_PointClouds.points[m].y;
		p[2] = pcl_PointClouds.points[m].z;
		int index_X = (p[0] - MinX) / deltaX;
		int index_Y = (p[1] - MinY) / deltaY;
		int OBB_index = index_Y * NumOfSamplesX + index_X;
		double q[3];
		vtk_MinMaxZValue->GetPoint(OBB_index, q);
		if (p[2] - q[0] > (q[1] - q[0])*ratioZ) VirtualPoints.push_back(pcl_PointClouds.points[m]);
	}

	pcl::copyPointCloud(VirtualPoints, ExtractedCloud);
}

void cinspection::SubOBB_ExtractVitualPoints(pcl::PointCloud<pcl::PointXYZRGB> &pcl_PointClouds,
	 pcl::PointCloud<pcl::PointXYZRGB> &ExtractedCloud, pcl::PointCloud<pcl::PointXYZRGB> &VirtualPoints,
	 int NumOfSamplesX, double ratioZ, double startX, double NumOfSubOBBX, double startY, double NumOfSubOBBY, double NumofOBB_model[3])
{
	double MaxX = -1 * DBL_MAX;
	double MinX = DBL_MAX;
	double MaxY = -1 * DBL_MAX;
	double MinY = DBL_MAX;
	double MaxZ = -1 * DBL_MAX;
	double MinZ = DBL_MAX;

	for (int i = 0; i < pcl_PointClouds.size(); i++)
	{
		if (pcl_PointClouds.points[i].x > MaxX) MaxX = pcl_PointClouds.points[i].x;
		if (pcl_PointClouds.points[i].x < MinX) MinX = pcl_PointClouds.points[i].x;
		if (pcl_PointClouds.points[i].y > MaxY) MaxY = pcl_PointClouds.points[i].y;
		if (pcl_PointClouds.points[i].y < MinY) MinY = pcl_PointClouds.points[i].y;
		if (pcl_PointClouds.points[i].z > MaxZ) MaxZ = pcl_PointClouds.points[i].z;
		if (pcl_PointClouds.points[i].z < MinZ) MinZ = pcl_PointClouds.points[i].z;
	}

	int NumOfSamplesY = NumOfSamplesX*(double)(MaxY - MinY) / (MaxX - MinX);
	int NumOfSamples = NumOfSamplesX*NumOfSamplesY;

	vtkSmartPointer<vtkPoints>  vtk_MinMaxZValue = vtkSmartPointer<vtkPoints>::New();
	for (int OBBindex = 0; OBBindex < NumOfSamples; OBBindex++)
	{
		vtk_MinMaxZValue->InsertPoint(OBBindex, MaxZ, MinZ, 0);
	}
	double deltaX = (double)(MaxX - MinX) / NumOfSamplesX;
	double deltaY = (double)(MaxY - MinY) / NumOfSamplesY;
	// Find max of Z at each SubOBB
	for (int m = 0; m < pcl_PointClouds.size(); m++)
	{
		double p[3];
		p[0] = pcl_PointClouds.points[m].x;
		p[1] = pcl_PointClouds.points[m].y;
		p[2] = pcl_PointClouds.points[m].z;
		int index_X = (p[0] - MinX) / deltaX;
		int index_Y = (p[1] - MinY) / deltaY;
		int OBB_index = index_Y*NumOfSamplesX + index_X;
		double q[3];
		vtk_MinMaxZValue->GetPoint(OBB_index, q);
		if (p[2] > q[1]) vtk_MinMaxZValue->InsertPoint(OBB_index, q[0], p[2], 0);
		if (p[2] < q[0]) vtk_MinMaxZValue->InsertPoint(OBB_index, p[2], q[1], 0);
	}
	// Extract Points captured by Virtual Camera
	for (int m = 0; m < pcl_PointClouds.size(); m++)
	{
		double p[3];
		p[0] = pcl_PointClouds.points[m].x;
		p[1] = pcl_PointClouds.points[m].y;
		p[2] = pcl_PointClouds.points[m].z;
		int index_X = (p[0] - MinX) / deltaX;
		int index_Y = (p[1] - MinY) / deltaY;
		int OBB_index = index_Y * NumOfSamplesX + index_X;
		double q[3];
		vtk_MinMaxZValue->GetPoint(OBB_index, q);
		if (p[2] - q[0] > (q[1] - q[0])*ratioZ) VirtualPoints.push_back(pcl_PointClouds.points[m]);
	}

	// Denoise
	Denoise_Statistical(VirtualPoints, VirtualPoints, noiseRatio);

	// Extract Points similar with measured object part
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_TransformedPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	int NumOfSubOBB = NumofOBB_model[0] * NumofOBB_model[1];

	double corner_OBB[3], max_OBB[3], mid_OBB[3], min_OBB[3];
	Eigen::Matrix4f matr_extracted = Eigen::Matrix4f::Identity();
	OBB_Calculation(VirtualPoints, corner_OBB, max_OBB, mid_OBB, min_OBB);
	RealcoordinateToOBBcoordinateXY(VirtualPoints, VirtualPoints, corner_OBB, max_OBB, mid_OBB, min_OBB);
	matr_extracted = matr.replicate(4, 4);
	pcl::transformPointCloud(pcl_PointClouds, pcl_PointClouds, matr_extracted);

	Eigen::Affine3f transformMatrix = Eigen::Affine3f::Identity();
	Eigen::Vector4f center;
	pcl::compute3DCentroid(VirtualPoints, center);
	transformMatrix.translation() << -center(0, 0), -center(1, 0), -center(2, 0);
	pcl::transformPointCloud(VirtualPoints, *pcl_TransformedPoints, transformMatrix);
	pcl::transformPointCloud(pcl_PointClouds, pcl_PointClouds, transformMatrix);
	matr = transformMatrix.matrix() * matr;

	// Find Max Min X Y Z
	MaxX = -1 * DBL_MAX;
	MinX = DBL_MAX;
	MaxY = -1 * DBL_MAX;
	MinY = DBL_MAX;
	for (int i = 0; i < pcl_TransformedPoints->size(); i++)
	{
		if (pcl_TransformedPoints->points[i].x > MaxX) MaxX = pcl_TransformedPoints->points[i].x;
		if (pcl_TransformedPoints->points[i].x < MinX) MinX = pcl_TransformedPoints->points[i].x;
		if (pcl_TransformedPoints->points[i].y  > MaxY) MaxY = pcl_TransformedPoints->points[i].y;
		if (pcl_TransformedPoints->points[i].y  < MinY) MinY = pcl_TransformedPoints->points[i].y;
	}
	for (int m = 0; m < pcl_TransformedPoints->size(); m++)
	{
		double deltaX = (MaxX - MinX) / NumofOBB_model[0];
		double deltaY = (MaxY - MinY) / NumofOBB_model[1];
		int index_X = (pcl_TransformedPoints->points[m].x - MinX) / deltaX;
		int index_Y = (pcl_TransformedPoints->points[m].y - MinY) / deltaY;
		if (index_X >= startX && index_X < startX + NumOfSubOBBX && index_Y >= startY && index_Y < startY + NumOfSubOBBY)
			ExtractedCloud.push_back(pcl_TransformedPoints->points[m]);
	}
}

bool cinspection::SubOBB_ExtractVitualPoints_Opt(pcl::PointCloud<pcl::PointXYZRGB> &cloud_Mod, pcl::PointCloud<pcl::PointXYZRGB> &cloud_Extracted,
	double bestView[3], double bestRota[3], double numOfOBB_mod[3], int numOfOBB_obj[3], double matchingPoint[3], int NumOfSamplesX, double ZThresh)
{
	Eigen::Matrix4f matr_buf = Eigen::Matrix4f::Identity();
	Eigen::Affine3f transformMatrix = Eigen::Affine3f::Identity();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_Mod_Buf(new pcl::PointCloud<pcl::PointXYZRGB>);

	transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (bestView[0] / (double)180), Eigen::Vector3f::UnitX()));
	transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (bestView[1] / (double)180), Eigen::Vector3f::UnitY()));
	transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (bestView[2] / (double)180), Eigen::Vector3f::UnitZ()));
	pcl::transformPointCloud(cloud_Mod, *cloud_Mod_Buf, transformMatrix);
	matr_buf = transformMatrix.matrix();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_Vir(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_Ext(new pcl::PointCloud<pcl::PointXYZRGB>);
	OBB_ExtractVitualPoints(cloud_Mod, *cloud_Ext, *cloud_Vir, NumOfSamplesX, ZThresh);
	Denoise_Statistical(*cloud_Ext, *cloud_Ext, noiseRatio);
	TransferToOBB_CorrdinateXY(*cloud_Ext, *cloud_Ext);
	matr_buf = matr * matr_buf;

	transformMatrix = Eigen::Affine3f::Identity();
	transformMatrix.rotate(Eigen::AngleAxisf(M_PI * double(bestRota[0] / 180), Eigen::Vector3f::UnitX()));
	transformMatrix.rotate(Eigen::AngleAxisf(M_PI * double(bestRota[2] / 180), Eigen::Vector3f::UnitZ()));
	pcl::transformPointCloud(*cloud_Ext, *cloud_Ext, transformMatrix);
	matr_buf = transformMatrix.matrix() * matr_buf;

	// Extract Points similar with measured object part
	// Find Max Min X Y Z
	double MaxX = -1 * DBL_MAX;
	double MinX = DBL_MAX;
	double MaxY = -1 * DBL_MAX;
	double MinY = DBL_MAX;
	for (int i = 0; i < cloud_Ext->size(); i++)
	{
		if (cloud_Ext->points[i].x > MaxX) MaxX = cloud_Ext->points[i].x;
		if (cloud_Ext->points[i].x < MinX) MinX = cloud_Ext->points[i].x;
		if (cloud_Ext->points[i].y  > MaxY) MaxY = cloud_Ext->points[i].y;
		if (cloud_Ext->points[i].y  < MinY) MinY = cloud_Ext->points[i].y;
	}
	if (cloud_Extracted.size()) cloud_Extracted.clear();
	for (int m = 0; m < cloud_Ext->size(); m++)
	{
		double deltaX = (MaxX - MinX) / numOfOBB_mod[0];
		double deltaY = (MaxY - MinY) / numOfOBB_mod[1];
		int index_X = (cloud_Ext->points[m].x - MinX) / deltaX;
		int index_Y = (cloud_Ext->points[m].y - MinY) / deltaY;
		if (index_X >= matchingPoint[0] & index_X < matchingPoint[0] + numOfOBB_obj[0] & index_Y >= matchingPoint[1] & 
			index_Y < matchingPoint[1] + numOfOBB_obj[1])
			cloud_Extracted.push_back(cloud_Ext->points[m]);
	}
	matr = matr_buf.replicate(4, 4);
	return 0;
}

void cinspection::RG_SubOBB_ObjExtraction(pcl::PointCloud<pcl::PointXYZRGB> &Input, pcl::PointCloud<pcl::PointXYZRGB> &output,  
	int numOfSubOBB_scene[3], int pickX, int pickY)
{
	// Find Max Min X Y Z
	double MaxX = -1 * DBL_MAX;
	double MinX = DBL_MAX;
	double MaxY = -1 * DBL_MAX;
	double MinY = DBL_MAX;
	for (int i = 0; i < Input.size(); i++)
	{
		if (Input.points[i].x > MaxX) MaxX = Input.points[i].x;
		if (Input.points[i].x < MinX) MinX = Input.points[i].x;
		if (Input.points[i].y > MaxY) MaxY = Input.points[i].y;
		if (Input.points[i].y < MinY) MinY = Input.points[i].y;
	}

	double deltaX = (MaxX - MinX) / (double)numOfSubOBB_scene[0];
	double deltaY = (MaxY - MinY) / (double)numOfSubOBB_scene[1];
	if (deltaX == 0 || deltaY == 0) return;
	for (int m = 0; m < Input.size(); m++)
	{
		int index_X = (Input.points[m].x - MinX) / deltaX;
		int index_Y = (Input.points[m].y - MinY) / deltaY;
		int OBB_index = index_Y*numOfSubOBB_scene[0] + index_X;
		if (index_X == pickX & index_Y == pickY) output.push_back(Input.points[m]);
	}
}

void cinspection::Nonseg_SubOBB_ObjExtraction(pcl::PointCloud<pcl::PointXYZRGB> &Input, pcl::PointCloud<pcl::PointXYZRGB> &output,
	int numOfSubOBB_Obj[3], int starPt[3])
{
	// Find Max Min X Y Z
	double MaxX = -1 * DBL_MAX;
	double MinX = DBL_MAX;
	double MaxY = -1 * DBL_MAX;
	double MinY = DBL_MAX;
	for (int i = 0; i < Input.size(); i++)
	{
		if (Input.points[i].x > MaxX) MaxX = Input.points[i].x;
		if (Input.points[i].x < MinX) MinX = Input.points[i].x;
		if (Input.points[i].y > MaxY) MaxY = Input.points[i].y;
		if (Input.points[i].y < MinY) MinY = Input.points[i].y;
	}
	if (output.size()) output.clear();
	for (int m = 0; m < Input.size(); m++)
	{
		int index_X = (Input.points[m].x - MinX) / (double)SubOBB_Size;
		int index_Y = (Input.points[m].y - MinY) / (double)SubOBB_Size;
		int OBB_index = index_Y*numOfSubOBB_Obj[0] + index_X;
		if (index_X >= starPt[0] & index_X < starPt[0] + numOfSubOBB_Obj[0] & 
			index_Y >= starPt[1] & index_Y < starPt[1] + numOfSubOBB_Obj[1]) 
			output.push_back(Input.points[m]);
	}
}

void cinspection::ICP_Registration(pcl::PointCloud<pcl::PointXYZRGB> &source_cloud, pcl::PointCloud<pcl::PointXYZRGB> &target_cloud,
	pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output, int iter)
{
	vtkSmartPointer<vtkPoints>  source_points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints>  target_points = vtkSmartPointer<vtkPoints>::New();
	PCL_TO_VTK(source_cloud, source_points);
	PCL_TO_VTK(target_cloud, target_points);

	vtkSmartPointer<vtkPolyData> source = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> target = vtkSmartPointer<vtkPolyData>::New();

	vtkSmartPointer<vtkPolyData> source1 = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> target1 = vtkSmartPointer<vtkPolyData>::New();

	source1->SetPoints(source_points);
	target1->SetPoints(target_points);

	vtkSmartPointer<vtkVertexGlyphFilter> glyphFilterSource = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	glyphFilterSource->SetInputData(source1);
	glyphFilterSource->Update();

	source->ShallowCopy(glyphFilterSource->GetOutput());

	vtkSmartPointer<vtkVertexGlyphFilter> glyphFilterTarget = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	glyphFilterTarget->SetInputData(target1);
	glyphFilterTarget->Update();

	target->ShallowCopy(glyphFilterTarget->GetOutput());

	// Setup ICP transform
	vtkSmartPointer<vtkIterativeClosestPointTransform> icp = vtkSmartPointer<vtkIterativeClosestPointTransform>::New();
	icp->SetSource(source);
	icp->SetTarget(target);
	icp->GetLandmarkTransform()->SetModeToRigidBody();
	icp->SetMaximumNumberOfIterations(iter);
	//icp->StartByMatchingCentroidsOn();
	icp->Modified();
	icp->Update();
	//vtkSmartPointer<vtkMatrix4x4> m = vtkSmartPointer<vtkMatrix4x4>::New();
	//m = icp->GetMatrix();

	//Apply ICP
	vtkSmartPointer<vtkPoints>  VTK_input = vtkSmartPointer<vtkPoints>::New();
	PCL_TO_VTK(input, VTK_input);
	vtkSmartPointer<vtkPolyData> source_input = vtkSmartPointer<vtkPolyData>::New();
	source_input->SetPoints(VTK_input);

	vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter_input = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	glyphFilter_input->SetInputData(source_input);
	glyphFilter_input->Update();

	vtkSmartPointer<vtkPolyData> Poly_input = vtkSmartPointer<vtkPolyData>::New();
	Poly_input->ShallowCopy(glyphFilter_input->GetOutput());

	vtkSmartPointer<vtkTransformFilter> finalTransformFilter = vtkSmartPointer<vtkTransformFilter>::New();
    #if VTK_MAJOR_VERSION <= 5
	finalTransformFilter->SetInput(Poly_input);
	#else
	finalTransformFilter->SetInputData(Poly_input);
	#endif
	finalTransformFilter->SetTransform(icp);
	finalTransformFilter->Update();
	vtkSmartPointer<vtkPoints>  VTK_output = vtkSmartPointer<vtkPoints>::New();
	VTK_output = finalTransformFilter->GetOutput()->GetPoints();
	if (output.size() != 0) output.clear();
	VTK_TO_PCL(VTK_output, output);

	for (int i = 0; i < 4; i++)
	for (int j = 0; j < 4; j++)
	{
		matr(i, j) = icp->GetMatrix()->GetElement(i, j);
	}
}

void cinspection::Inverse_ICP_Registration(pcl::PointCloud<pcl::PointXYZRGB> &source_cloud, pcl::PointCloud<pcl::PointXYZRGB> &target_cloud,
	pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output, int iter)
{
	vtkSmartPointer<vtkPoints>  source_points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints>  target_points = vtkSmartPointer<vtkPoints>::New();
	PCL_TO_VTK(source_cloud, source_points);
	PCL_TO_VTK(target_cloud, target_points);

	vtkSmartPointer<vtkPolyData> source = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> target = vtkSmartPointer<vtkPolyData>::New();

	vtkSmartPointer<vtkPolyData> source1 = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> target1 = vtkSmartPointer<vtkPolyData>::New();

	source1->SetPoints(source_points);
	target1->SetPoints(target_points);

	vtkSmartPointer<vtkVertexGlyphFilter> glyphFilterSource = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	glyphFilterSource->SetInputData(source1);
	glyphFilterSource->Update();

	source->ShallowCopy(glyphFilterSource->GetOutput());

	vtkSmartPointer<vtkVertexGlyphFilter> glyphFilterTarget = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	glyphFilterTarget->SetInputData(target1);
	glyphFilterTarget->Update();

	target->ShallowCopy(glyphFilterTarget->GetOutput());

	// Setup ICP transform
	vtkSmartPointer<vtkIterativeClosestPointTransform> icp = vtkSmartPointer<vtkIterativeClosestPointTransform>::New();
	icp->SetSource(source);
	icp->SetTarget(target);
	icp->GetLandmarkTransform()->SetModeToRigidBody();
	icp->SetMaximumNumberOfIterations(iter);
	icp->Modified();
	icp->Update();

	//Apply ICP
	vtkSmartPointer<vtkPoints>  VTK_input = vtkSmartPointer<vtkPoints>::New();
	PCL_TO_VTK(input, VTK_input);

	vtkSmartPointer<vtkMatrix4x4> Inverse_m = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(icp->GetMatrix(), Inverse_m);
	for (int i = 0; i < VTK_input->GetNumberOfPoints(); i++)
	{
		double p[3];
		VTK_input->GetPoint(i, p);
		double q[4] = { p[0], p[1], p[2], 1 };
		double q1[4];
		q1[0] = Inverse_m->MultiplyDoublePoint(q)[0];
		q1[1] = Inverse_m->MultiplyDoublePoint(q)[1];
		q1[2] = Inverse_m->MultiplyDoublePoint(q)[2];
		VTK_input->InsertPoint(i, q1[0], q1[1], q1[2]);
	}

	if (output.size() != 0) output.clear();
	VTK_TO_PCL(VTK_input, output);
}

bool cinspection::RG_SubOBB_StartFind(pcl::PointCloud<pcl::PointXYZRGB> &Input, pcl::PointCloud<pcl::PointXYZRGB> &output, int numOfSubOBB_scene[3])
{
	if (output.size()) output.clear();
	double subobbAreaMax_scence;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Descriptor(new pcl::PointCloud<pcl::PointXYZRGB>);
	OBB_Descriptor(Input, *Descriptor, numOfSubOBB_scene[0], numOfSubOBB_scene[1], subobbAreaMax_scence);
	int count = 0;
	int winSize = NumOfSubOBBY_model / 2 - 1;
	if (winSize < 1) winSize = 1;
	for (int i = 0; i < numOfSubOBB_scene[0]; i++)
	for (int j = 0; j < numOfSubOBB_scene[1]; j++)
	{
		pcl::PointXYZRGB point;
		point.x = i;
		point.y = j;
		point.g = 0;
		point.z = Descriptor->points[j*numOfSubOBB_scene[0] + i].z;
		if (point.z == 0) continue;
		for (int l = -winSize; l < winSize + 1; l++)
		for (int k = -winSize; k < winSize + 1; k++)
		{
			if (k == 0 & l == 0) continue;
			int index = (j + k)*numOfSubOBB_scene[0] + (i + l);
			if (j + k > 0 & j + k < numOfSubOBB_scene[1] & i + l > 0 & i + l < numOfSubOBB_scene[0])
			{
				if (Descriptor->points[index].z > 0)
				{
					point.z += Descriptor->points[index].z;
					point.g++;
				}
			}
		}
		output.push_back(point);
	}
	return 1;
}

int cinspection::RG_SubOBB_Matching(pcl::PointCloud<pcl::PointXYZRGB> &objCloud, pcl::PointCloud<pcl::PointXYZRGB> &model,
	pcl::PointCloud<pcl::PointXYZ> &PCLBestRota_mod, pcl::PointCloud<pcl::PointXYZ> &PCLBestView_mod, pcl::PointCloud<pcl::PointXYZ> &PCLBestNCC,
	pcl::PointCloud<pcl::PointXYZ> &PCLBestNumOfSubOBB, double NCCThresh)
{
	if (objCloud.size() == 0) return 1;
	if (PCLBestView_mod.size()) PCLBestView_mod.clear();
	if (PCLBestRota_mod.size()) PCLBestRota_mod.clear();
	if (PCLBestNCC.size()) PCLBestNCC.clear();
	if (PCLBestNumOfSubOBB.size()) PCLBestNumOfSubOBB.clear();

	int numOfSubOBB_object[3];
	double NCCMax = -1;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_Obj(new pcl::PointCloud<pcl::PointXYZRGB>);
	TransferToOBB_CorrdinateXY(objCloud, *cloud_Obj);
	numOfSubOBB_Estimation(*cloud_Obj, model, numOfSubOBB_object);

	for (int RotaZ = 0; RotaZ < 90; RotaZ += 10)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_Obj_Buf(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr PCLBestView_Buf(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr PCLBestRota_Buf(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr PCLBestNCC_Buf(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr PCLBestNumOfSubOBB_Buf(new pcl::PointCloud<pcl::PointXYZ>);

		Eigen::Affine3f transformMatrix = Eigen::Affine3f::Identity();
		transformMatrix.rotate(Eigen::AngleAxisf(M_PI * double(RotaZ / 180), Eigen::Vector3f::UnitZ()));
		pcl::transformPointCloud(*cloud_Obj, *cloud_Obj_Buf, transformMatrix);
		if (SubOBB_matching_scence_model_Opt(*cloud_Obj_Buf, numOfSubOBB_object, *PCLBestView_Buf, *PCLBestRota_Buf, *PCLBestNCC_Buf,
			*PCLBestNumOfSubOBB_Buf)) continue;

		if (NCCMax < PCLBestNCC_Buf->points[PCLBestNCC_Buf->size() - 1].z)
		{
			NCCMax = PCLBestNCC_Buf->points[PCLBestNCC_Buf->size() - 1].z;
			pcl::copyPointCloud(*PCLBestView_Buf, PCLBestView_mod);
			pcl::copyPointCloud(*PCLBestRota_Buf, PCLBestRota_mod);
			pcl::copyPointCloud(*PCLBestNCC_Buf, PCLBestNCC);
			pcl::copyPointCloud(*PCLBestNumOfSubOBB_Buf, PCLBestNumOfSubOBB);
		}
		if (NCCMax > NCCThresh) return 0;
	}
	if (NCCMax < NCCThresh) return 2;
	return 0;
}

unsigned char cinspection::OBB_Recognition(pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output, double toler, double pass)
{
	reportCount = 0;
	ofstream onlineFile;
	onlineFile.open("Report/Report_online.txt");
	QElapsedTimer timer;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ObjectCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ModelCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ModelDescriptor(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr VirtualPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCLBestView(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCLBestRota(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCLBestNCC(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCLnumOfSubOBB(new pcl::PointCloud<pcl::PointXYZ>);

	Eigen::Matrix4f matr_mod = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f matr_obj = Eigen::Matrix4f::Identity();
	Eigen::Vector4f center_obj;
	Eigen::Vector4f center_mod;

	Eigen::Affine3f transformMatrix = Eigen::Affine3f::Identity();

	//double center_obj[3];
	double center[3];
	double corner_OBB[3], max_OBB[3], mid_OBB[3], min_OBB[3];
	double NCCBest = -1 * DBL_MAX;

	// Get Object Cloud from segmentation
	OBB_Calculation(input, corner_OBB, max_OBB, mid_OBB, min_OBB);
	double lengthOBB_object[3];
	lengthOBB_object[0] = sqrt(max_OBB[0] * max_OBB[0] + max_OBB[1] * max_OBB[1] + max_OBB[2] * max_OBB[2]); // length of max
	lengthOBB_object[1] = sqrt(mid_OBB[0] * mid_OBB[0] + mid_OBB[1] * mid_OBB[1] + mid_OBB[2] * mid_OBB[2]); // length of mid
	lengthOBB_object[2] = sqrt(min_OBB[0] * min_OBB[0] + min_OBB[1] * min_OBB[1] + min_OBB[2] * min_OBB[2]); // length of min

	//Transform object cloud to origin and real coordinate --> 2 purposes: easy to compute descriptor; easy to align its OBB with model's OBB
	//OBB_Calculation(input, corner_OBB, max_OBB, mid_OBB, min_OBB);
	RealcoordinateToOBBcoordinateXY(input, *ObjectCloud, corner_OBB, max_OBB, mid_OBB, min_OBB);
	matr_obj = matr.replicate(4, 4);
	pcl::compute3DCentroid(*ObjectCloud, center_obj);
	transformMatrix.translation() << -center_obj(0, 0), -center_obj(1, 0), -center_obj(2, 0);
	pcl::transformPointCloud(*ObjectCloud, *ObjectCloud, transformMatrix);
	
	int countLine = 0;
	// Read Points from model
	ReadPCLPointCloud(*ModelCloud);
	if (ModelCloud->size() == 0) return 3;
	// Read Descriptor of model
	ReadPCLDescriptor(countLine, countLine, *ModelDescriptor);
	// Compute mean distance every point to its closest point

	double modelResol, objResol;
	timer.start();

	while (ModelDescriptor->size() != 0)
	{
		double p_ModelOrientation[3];
		p_ModelOrientation[0] = ModelDescriptor->points[0].x; //get Rx Ry Rz of model from txt file
		p_ModelOrientation[1] = ModelDescriptor->points[0].y;
		p_ModelOrientation[2] = ModelDescriptor->points[0].z;
		double lengthOBB_model[3];
		lengthOBB_model[0] = ModelDescriptor->points[1].x; //get length of OBB of model with Max Mid Min from txt file
		lengthOBB_model[1] = ModelDescriptor->points[1].y;
		lengthOBB_model[2] = ModelDescriptor->points[1].z;
		double NumOfSubOBB_model[3];
		NumOfSubOBB_model[0] = ModelDescriptor->points[2].x; //get Number of SubOBBs of model  along OX and OY from txt file
		NumOfSubOBB_model[1] = ModelDescriptor->points[2].y;
		NumOfSubOBB_model[2] = ModelDescriptor->points[2].z;

		if (abs(lengthOBB_object[0] - lengthOBB_model[0]) < toler*lengthOBB_model[0] &
			abs(lengthOBB_object[1] - lengthOBB_model[1]) < toler*lengthOBB_model[1])
		{
			//Estimate number of subOBBs of measured point cloud based on length of its OBB and number of subOBBs of OBB of model
			int NumOfSubOBBX = NumOfSubOBB_model[0] * ((double)lengthOBB_object[0] / lengthOBB_model[0]) + 0.5;//0.5 means if 1.7->2, if 1.3->1
			int NumOfSubOBBY = NumOfSubOBB_model[1] * ((double)lengthOBB_object[1] / lengthOBB_model[1]) + 0.5;//0.5 means if 1.7->2, if 1.3->1
			if (NumOfSubOBBX > NumOfSubOBB_model[0]) NumOfSubOBBX = NumOfSubOBB_model[0];
			if (NumOfSubOBBY > NumOfSubOBB_model[1]) NumOfSubOBBY = NumOfSubOBB_model[1];
			if (NumOfSubOBBX == 0) NumOfSubOBBX = 1;
			if (NumOfSubOBBY == 0) NumOfSubOBBY = 1;

			/*onlineFile << "NumOfSubOBBX: " << NumOfSubOBBX << "\n";
			onlineFile << "NumOfSubOBBY: " << NumOfSubOBBY << "\n";*/

			// Because start position for computing descriptor is random at 4 lower corners of OBB, we need to
			// rotate object cloud to test with different position to choose the best one.
			double NCCmax = -1;
			int countView = 0;
			timer.start();
			OBB_matching_scence_model(*ObjectCloud, *ModelDescriptor, lengthOBB_model[1], lengthOBB_object[0], NumOfSubOBB_model[2], NumOfSubOBBX,
				NumOfSubOBBY, NumOfSubOBB_model[0], NumOfSubOBB_model[1], NCCmax, countView);
			onlineFile << "VTK_matching_scence_model: " << timer.elapsed() << " ms" << "\n";

			PCLBestView->push_back(pcl::PointXYZ(p_ModelOrientation[0], p_ModelOrientation[1], p_ModelOrientation[2]));
			PCLBestNCC->push_back(pcl::PointXYZ(NumOfSubOBBX, NumOfSubOBBY, NCCmax));
			PCLnumOfSubOBB->push_back(pcl::PointXYZ(NumOfSubOBB_model[0], NumOfSubOBB_model[1], NumOfSubOBB_model[2]));
            if (NCCmax >  NCCBest) NCCBest = NCCmax;
		}

		// Start new view from virtual camera
		countLine++;
		if (ModelDescriptor->size()) ModelDescriptor->clear();
		// Read Descriptor of model
		ReadPCLDescriptor(countLine, countLine, *ModelDescriptor);
	}
	if (PCLBestNCC->size() == 0)
	{
		onlineFile.close();
		return 1;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ObjectCloud_Buffer(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*ObjectCloud, *ObjectCloud_Buffer);

	double maxval = -1 * DBL_MAX;
	int maxIndex;
	while (true)
	{
		maxval = -1 * DBL_MAX;
		for (int numOfBestNCC = 0; numOfBestNCC < PCLBestNCC->size(); numOfBestNCC++)
		{
			if (PCLBestNCC->points[numOfBestNCC].z > maxval)
			{
				maxval = PCLBestNCC->points[numOfBestNCC].z;
				maxIndex = numOfBestNCC;
			}
		}
		double NCC_Ratio = 0.5;
		if (abs(maxval) < abs(NCC_Ratio * NCCBest) || maxval == -1)
		{
			onlineFile.close();
			return 2;
		}
		else
		{
			onlineFile << "\nNCC maxval: " << maxval << "\n";
			PCLBestNCC->points[maxIndex].z = -1;
			double p_numOfSubOBB[3];
			p_numOfSubOBB[0] = PCLnumOfSubOBB->points[maxIndex].x;
			p_numOfSubOBB[1] = PCLnumOfSubOBB->points[maxIndex].y;
			p_numOfSubOBB[2] = PCLnumOfSubOBB->points[maxIndex].z;
			// Read parameters of 3D virtual camera
			int NumOfSamplesX;
			double ZThresh;
			ReaVirtualCameraParameters(NumOfSamplesX, ZThresh);

			// Extract points on model similar with measured points
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr ModelTran(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr ExtractedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			if (VirtualPoints->size() != 0) VirtualPoints->clear();
			transformMatrix = Eigen::Affine3f::Identity();
			transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (PCLBestView->points[maxIndex].x / (double)180), Eigen::Vector3f::UnitX()));
			transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (PCLBestView->points[maxIndex].y / (double)180), Eigen::Vector3f::UnitY()));
			transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (PCLBestView->points[maxIndex].z / (double)180), Eigen::Vector3f::UnitZ()));
			pcl::transformPointCloud(*ModelCloud, *ModelTran, transformMatrix);

			OBB_ExtractVitualPoints(*ModelTran, *ExtractedCloud, *VirtualPoints, NumOfSamplesX, ZThresh);
			// Denoise
			Denoise_Statistical(*ExtractedCloud, *ExtractedCloud, noiseRatio);

			// Coarse Registration Align OBB of extracted model point cloud with OBB of measured object point cloud
			OBB_Calculation(*ExtractedCloud, corner_OBB, max_OBB, mid_OBB, min_OBB);
			RealcoordinateToOBBcoordinateXY(*ExtractedCloud, *ExtractedCloud, corner_OBB, max_OBB, mid_OBB, min_OBB);
			matr_mod = matr.replicate(4, 4);
			pcl::transformPointCloud(*ModelTran, *ModelTran, matr_mod);

			pcl::compute3DCentroid(*ExtractedCloud, center_mod);
			transformMatrix = Eigen::Affine3f::Identity();
			transformMatrix.translation() << -center_mod(0, 0), -center_mod(1, 0), -center_mod(2, 0);
			pcl::transformPointCloud(*ExtractedCloud, *ExtractedCloud, transformMatrix);
			pcl::transformPointCloud(*ModelTran, *ModelTran, transformMatrix);

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr ModelTran_Buffer(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::copyPointCloud(*ModelTran, *ModelTran_Buffer);
			// Because of kz = 1, object might be symmtrical along axis Ox or Oy, so need be checked by rotX 180 and rotY 180
			// If kx approximates ky, axis Ox and axis Oy will be ambiguous, so need be checked by rotZ by increment 90
			double increaZ = 90;
			if (abs(lengthOBB_object[0] - lengthOBB_object[1]) > 0.05*lengthOBB_object[0]) increaZ = 360;
			for (int rotZ = 0; rotZ < 360; rotZ += increaZ)
			for (int rotY = 0; rotY < 360; rotY += 180)
			for (int rotX = 0; rotX < 360; rotX += 180)
			{
				onlineFile << "rotZYX: " << rotZ << " " << rotY << " " << rotX << "\n";
				transformMatrix = Eigen::Affine3f::Identity();
				transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (rotX / (double)180), Eigen::Vector3f::UnitX()));
				transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (rotY / (double)180), Eigen::Vector3f::UnitY()));
				transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (rotZ / (double)180), Eigen::Vector3f::UnitZ()));
				pcl::transformPointCloud(*ObjectCloud_Buffer, *ObjectCloud, transformMatrix);
				pcl::copyPointCloud(*ModelTran_Buffer, *ModelTran);

				Inverse_ICP_Registration(*ObjectCloud, *ModelTran, *ModelTran, *ModelTran, 100);
				double meandist;
				PCL_MeanDistance_CloudToCloud(*ObjectCloud, *ModelTran, meandist);
				onlineFile << "meandist: " << meandist << "\n";
				if (meandist < pass)
				{
					pcl::transformPointCloud(*ModelTran, *ModelTran, transformMatrix.inverse());
					transformMatrix = Eigen::Affine3f::Identity();
					transformMatrix.translation() << center_obj(0, 0), center_obj(1, 0), center_obj(2, 0);
					pcl::transformPointCloud(*ModelTran, *ModelTran, transformMatrix);
					Eigen::Matrix4f inverse_matr_obj = Eigen::Matrix4f::Identity();
					inverse_matr_obj = matr_obj.inverse();
					pcl::transformPointCloud(*ModelTran, *ModelTran, inverse_matr_obj);

					pcl::copyPointCloud(*ModelTran, output);
					//output.operator+=(*ObjectCloud);
					onlineFile.close();
					return 0;
				}
			}
		}
	}
	onlineFile.close();
	return 0;
}

unsigned char cinspection::SubOBB_Recognition(pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output,
	double toler, double pass)
{
	reportCount = 0;
	ofstream onlineFile;
	onlineFile.open("Report/Report_online.txt");
	QElapsedTimer timer;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ObjectCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ModelCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ModelDescriptor(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr VirtualPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCLBestView(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCLBestRota(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCLBestNCC(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCLnumOfSubOBB(new pcl::PointCloud<pcl::PointXYZ>);

	Eigen::Matrix4f matr_mod = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f matr_obj = Eigen::Matrix4f::Identity();
	Eigen::Vector4f center_obj;
	Eigen::Vector4f center_mod;

	Eigen::Affine3f transformMatrix = Eigen::Affine3f::Identity();

	//double center_obj[3];
	double center[3];
	double corner_OBB[3], max_OBB[3], mid_OBB[3], min_OBB[3];
	double NCCBest = -1 * DBL_MAX;

	// Get Object Cloud from segmentation
	OBB_Calculation(input, corner_OBB, max_OBB, mid_OBB, min_OBB);
	double lengthOBB_object[3];
	lengthOBB_object[0] = sqrt(max_OBB[0] * max_OBB[0] + max_OBB[1] * max_OBB[1] + max_OBB[2] * max_OBB[2]); // length of max
	lengthOBB_object[1] = sqrt(mid_OBB[0] * mid_OBB[0] + mid_OBB[1] * mid_OBB[1] + mid_OBB[2] * mid_OBB[2]); // length of mid
	lengthOBB_object[2] = sqrt(min_OBB[0] * min_OBB[0] + min_OBB[1] * min_OBB[1] + min_OBB[2] * min_OBB[2]); // length of min

	//Transform object cloud to origin and real coordinate --> 2 purposes: easy to compute descriptor; easy to align its OBB with model's OBB
	//OBB_Calculation(input, corner_OBB, max_OBB, mid_OBB, min_OBB);
	RealcoordinateToOBBcoordinateXY(input, *ObjectCloud, corner_OBB, max_OBB, mid_OBB, min_OBB);
	matr_obj = matr.replicate(4, 4);
	pcl::compute3DCentroid(*ObjectCloud, center_obj);
	transformMatrix.translation() << -center_obj(0, 0), -center_obj(1, 0), -center_obj(2, 0);
	pcl::transformPointCloud(*ObjectCloud, *ObjectCloud, transformMatrix);

	int countLine = 0;
	// Read Points from model
	ReadPCLPointCloud(*ModelCloud);
	if (ModelCloud->size() == 0) return 3;
	// Read Descriptor of model
	ReadPCLDescriptor(countLine, countLine, *ModelDescriptor);
	// Compute mean distance every point to its closest point

	timer.start();

	while (ModelDescriptor->size() != 0)
	{
		double p_ModelOrientation[3];
		p_ModelOrientation[0] = ModelDescriptor->points[0].x; //get Rx Ry Rz of model from txt file
		p_ModelOrientation[1] = ModelDescriptor->points[0].y;
		p_ModelOrientation[2] = ModelDescriptor->points[0].z;
		double lengthOBB_model[3];
		lengthOBB_model[0] = ModelDescriptor->points[1].x; //get length of OBB of model with Max Mid Min from txt file
		lengthOBB_model[1] = ModelDescriptor->points[1].y;
		lengthOBB_model[2] = ModelDescriptor->points[1].z;
		double NumOfSubOBB_model[3];
		NumOfSubOBB_model[0] = ModelDescriptor->points[2].x; //get Number of SubOBBs of model  along OX and OY from txt file
		NumOfSubOBB_model[1] = ModelDescriptor->points[2].y;
		NumOfSubOBB_model[2] = ModelDescriptor->points[2].z;

		if (abs(lengthOBB_object[0] - lengthOBB_model[0]) < toler*lengthOBB_model[0] &
			abs(lengthOBB_object[1] - lengthOBB_model[1]) < toler*lengthOBB_model[1])
		{
			//Estimate number of subOBBs of measured point cloud based on length of its OBB and number of subOBBs of OBB of model
			int NumOfSubOBBX = NumOfSubOBB_model[0] * ((double)lengthOBB_object[0] / lengthOBB_model[0]) + 0.5;//0.5 means if 1.7->2, if 1.3->1
			int NumOfSubOBBY = NumOfSubOBB_model[1] * ((double)lengthOBB_object[1] / lengthOBB_model[1]) + 0.5;//0.5 means if 1.7->2, if 1.3->1
			if (NumOfSubOBBX > NumOfSubOBB_model[0]) NumOfSubOBBX = NumOfSubOBB_model[0];
			if (NumOfSubOBBY > NumOfSubOBB_model[1]) NumOfSubOBBY = NumOfSubOBB_model[1];
			if (NumOfSubOBBX == 0) NumOfSubOBBX = 1;
			if (NumOfSubOBBY == 0) NumOfSubOBBY = 1;
			onlineFile << "NumOfSubOBBX: " << NumOfSubOBBX << "\n";
			onlineFile << "NumOfSubOBBY: " << NumOfSubOBBY << "\n";

			// Because start position for computing descriptor is random at 4 lower corners of OBB, we need to
			// rotate object cloud to test with different position to choose the best one.
			double NCCmax = -1;
			double BestRota = -1 * DBL_MAX;
			double StartX = -1;
			double StartY = -1;
			int countView = 0;
			timer.start();
			SubOBB_matching_scence_model(*ObjectCloud, *ModelDescriptor, lengthOBB_model[1], lengthOBB_object[0], NumOfSubOBB_model[2], NumOfSubOBBX,
				NumOfSubOBBY, NumOfSubOBB_model[0], NumOfSubOBB_model[1], StartX, StartY, BestRota, NCCmax, countView);

			PCLBestRota->push_back(pcl::PointXYZ(StartX, StartY, BestRota));
			PCLBestView->push_back(pcl::PointXYZ(p_ModelOrientation[0], p_ModelOrientation[1], p_ModelOrientation[2]));
			PCLBestNCC->push_back(pcl::PointXYZ(NumOfSubOBBX, NumOfSubOBBY, NCCmax));
			PCLnumOfSubOBB->push_back(pcl::PointXYZ(NumOfSubOBB_model[0], NumOfSubOBB_model[1], NumOfSubOBB_model[2]));
			if (NCCmax >  NCCBest) NCCBest = NCCmax;
		}

		// Start new view from virtual camera
		countLine++;
		if (ModelDescriptor->size()) ModelDescriptor->clear();
		// Read Descriptor of model
		ReadPCLDescriptor(countLine, countLine, *ModelDescriptor);
	}
	if (PCLBestNCC->size() == 0)
	{
		onlineFile.close();
		return 1;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ObjectCloud_Buffer(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*ObjectCloud, *ObjectCloud_Buffer);

	double maxval = -1 * DBL_MAX;
	int maxIndex;
	while (true)
	{
		maxval = -1 * DBL_MAX;
		for (int numOfBestNCC = 0; numOfBestNCC < PCLBestNCC->size(); numOfBestNCC++)
		{
			if (PCLBestNCC->points[numOfBestNCC].z > maxval)
			{
				maxval = PCLBestNCC->points[numOfBestNCC].z;
				maxIndex = numOfBestNCC;
			}
		}
		double NCC_Ratio = 0.5;
		if (abs(maxval) < abs(NCC_Ratio * NCCBest) || maxval == -1)
		{
			onlineFile.close();
			return 2;
		}
		else
		{
			onlineFile << "\nNCC maxval: " << maxval << "\n";
			PCLBestNCC->points[maxIndex].z = -1;
			double p_numOfSubOBB[3];
			p_numOfSubOBB[0] = PCLnumOfSubOBB->points[maxIndex].x;
			p_numOfSubOBB[1] = PCLnumOfSubOBB->points[maxIndex].y;
			p_numOfSubOBB[2] = PCLnumOfSubOBB->points[maxIndex].z;
			// Read parameters of 3D virtual camera
			int NumOfSamplesX;
			double ZThresh;
			ReaVirtualCameraParameters(NumOfSamplesX, ZThresh);

			// Extract points on model similar with measured points
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr ModelTran(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr ExtractedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			if (VirtualPoints->size() != 0) VirtualPoints->clear();
			transformMatrix = Eigen::Affine3f::Identity();
			transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (PCLBestView->points[maxIndex].x / (double)180), Eigen::Vector3f::UnitX()));
			transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (PCLBestView->points[maxIndex].y / (double)180), Eigen::Vector3f::UnitY()));
			transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (PCLBestView->points[maxIndex].z / (double)180), Eigen::Vector3f::UnitZ()));
			pcl::transformPointCloud(*ModelCloud, *ModelTran, transformMatrix);

			SubOBB_ExtractVitualPoints(*ModelTran, *ExtractedCloud, *VirtualPoints, NumOfSamplesX, ZThresh, PCLBestRota->points[maxIndex].x,
				PCLBestNCC->points[maxIndex].x, PCLBestRota->points[maxIndex].y, PCLBestNCC->points[maxIndex].y, p_numOfSubOBB);

			// Coarse Registration Align OBB of extracted model point cloud with OBB of measured object point cloud
			OBB_Calculation(*ExtractedCloud, corner_OBB, max_OBB, mid_OBB, min_OBB);
			RealcoordinateToOBBcoordinateXY(*ExtractedCloud, *ExtractedCloud, corner_OBB, max_OBB, mid_OBB, min_OBB);
			matr_mod = matr.replicate(4, 4);
			pcl::transformPointCloud(*ModelTran, *ModelTran, matr_mod);

			pcl::compute3DCentroid(*ExtractedCloud, center_mod);
			transformMatrix = Eigen::Affine3f::Identity();
			transformMatrix.translation() << -center_mod(0, 0), -center_mod(1, 0), -center_mod(2, 0);
			pcl::transformPointCloud(*ExtractedCloud, *ExtractedCloud, transformMatrix);
			pcl::transformPointCloud(*ModelTran, *ModelTran, transformMatrix);

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr ModelTran_Buffer(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::copyPointCloud(*ModelTran, *ModelTran_Buffer);
			// Because of kz = 1, object might be symmtrical along axis Ox or Oy, so need be checked by rotX 180 and rotY 180
			// If kx approximates ky, axis Ox and axis Oy will be ambiguous, so need be checked by rotZ by increment 90
			double increaZ = 90;
			if (abs(lengthOBB_object[0] - lengthOBB_object[1]) > 0.05*lengthOBB_object[0]) increaZ = 360;
			for (int rotZ = 0; rotZ < 360; rotZ += increaZ)
			for (int rotY = 0; rotY < 360; rotY += 180)
			for (int rotX = 0; rotX < 360; rotX += 180)
			{
				transformMatrix = Eigen::Affine3f::Identity();
				transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (rotX / (double)180), Eigen::Vector3f::UnitX()));
				transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (rotY / (double)180), Eigen::Vector3f::UnitY()));
				transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (rotZ / (double)180), Eigen::Vector3f::UnitZ()));
				pcl::transformPointCloud(*ObjectCloud_Buffer, *ObjectCloud, transformMatrix);
				pcl::copyPointCloud(*ModelTran_Buffer, *ModelTran);

				//pcl::copyPointCloud(*ObjectCloud, output);
				//output.operator+=(*ExtractedCloud);

				Inverse_ICP_Registration(*ObjectCloud, *ModelTran, *ModelTran, *ModelTran, 100);
				double meandist;
				PCL_MeanDistance_CloudToCloud(*ObjectCloud, *ModelTran, meandist);
				onlineFile << "meandist: " << meandist << "\n";
				if (meandist < pass)
				{
					pcl::transformPointCloud(*ModelTran, *ModelTran, transformMatrix.inverse());
					transformMatrix = Eigen::Affine3f::Identity();
					transformMatrix.translation() << center_obj(0, 0), center_obj(1, 0), center_obj(2, 0);
					pcl::transformPointCloud(*ModelTran, *ModelTran, transformMatrix);
					Eigen::Matrix4f inverse_matr_obj = Eigen::Matrix4f::Identity();
					inverse_matr_obj = matr_obj.inverse();
					pcl::transformPointCloud(*ModelTran, *ModelTran, inverse_matr_obj);

					pcl::copyPointCloud(*ModelTran, output);
					//output.operator+=(*ObjectCloud);
					onlineFile.close();
					return 0;
				}
			}
		}
	}
	onlineFile.close();
	return 0;
}

unsigned char cinspection::RG_SubOBB_Recognition(pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output,
	double toler, double pass)
{
	return 1;
}

unsigned char cinspection::model_Driven_Registration(pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output,
	double toler, double pass)
{
	reportCount = 0;
	ofstream onlineFile;
	onlineFile.open("Report/Report_online.txt");
	QElapsedTimer timer;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ObjectCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ModelCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ModelDescriptor(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr VirtualPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCLBestView(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCLBestRota(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCLBestNCC(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCLnumOfSubOBB(new pcl::PointCloud<pcl::PointXYZ>);

	Eigen::Matrix4f matr_mod = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f matr_regis = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f matr_obj = Eigen::Matrix4f::Identity();
	Eigen::Vector4f center_obj;
	Eigen::Vector4f center_mod;

	Eigen::Affine3f transformMatrix = Eigen::Affine3f::Identity();

	//double center_obj[3];
	double center[3];
	double corner_OBB[3], max_OBB[3], mid_OBB[3], min_OBB[3];
	double NCCBest = -1 * DBL_MAX;

	// Get Object Cloud from segmentation
	OBB_Calculation(input, corner_OBB, max_OBB, mid_OBB, min_OBB);
	double lengthOBB_object[3];
	lengthOBB_object[0] = sqrt(max_OBB[0] * max_OBB[0] + max_OBB[1] * max_OBB[1] + max_OBB[2] * max_OBB[2]); // length of max
	lengthOBB_object[1] = sqrt(mid_OBB[0] * mid_OBB[0] + mid_OBB[1] * mid_OBB[1] + mid_OBB[2] * mid_OBB[2]); // length of mid
	lengthOBB_object[2] = sqrt(min_OBB[0] * min_OBB[0] + min_OBB[1] * min_OBB[1] + min_OBB[2] * min_OBB[2]); // length of min

	//Transform object cloud to origin and real coordinate --> 2 purposes: easy to compute descriptor; easy to align its OBB with model's OBB
	//OBB_Calculation(input, corner_OBB, max_OBB, mid_OBB, min_OBB);
	RealcoordinateToOBBcoordinateXY(input, *ObjectCloud, corner_OBB, max_OBB, mid_OBB, min_OBB);
	matr_obj = matr.replicate(4, 4);
	pcl::compute3DCentroid(*ObjectCloud, center_obj);
	transformMatrix.translation() << -center_obj(0, 0), -center_obj(1, 0), -center_obj(2, 0);
	pcl::transformPointCloud(*ObjectCloud, *ObjectCloud, transformMatrix);

	int countLine = 0;
	// Read Points from model
	ReadPCLPointCloud(*ModelCloud);
	if (ModelCloud->size() == 0) return 3;
	// Read Descriptor of model
	ReadPCLDescriptor(countLine, countLine, *ModelDescriptor);
	// Compute mean distance every point to its closest point

	timer.start();

	while (ModelDescriptor->size() != 0)
	{
		double p_ModelOrientation[3];
		p_ModelOrientation[0] = ModelDescriptor->points[0].x; //get Rx Ry Rz of model from txt file
		p_ModelOrientation[1] = ModelDescriptor->points[0].y;
		p_ModelOrientation[2] = ModelDescriptor->points[0].z;
		double lengthOBB_model[3];
		lengthOBB_model[0] = ModelDescriptor->points[1].x; //get length of OBB of model with Max Mid Min from txt file
		lengthOBB_model[1] = ModelDescriptor->points[1].y;
		lengthOBB_model[2] = ModelDescriptor->points[1].z;
		double NumOfSubOBB_model[3];
		NumOfSubOBB_model[0] = ModelDescriptor->points[2].x; //get Number of SubOBBs of model  along OX and OY from txt file
		NumOfSubOBB_model[1] = ModelDescriptor->points[2].y;
		NumOfSubOBB_model[2] = ModelDescriptor->points[2].z;

		if (abs(lengthOBB_object[0] - lengthOBB_model[0]) < toler*lengthOBB_model[0] &
			abs(lengthOBB_object[1] - lengthOBB_model[1]) < toler*lengthOBB_model[1])
		{
			//Estimate number of subOBBs of measured point cloud based on length of its OBB and number of subOBBs of OBB of model
			int NumOfSubOBBX = NumOfSubOBB_model[0] * ((double)lengthOBB_object[0] / lengthOBB_model[0]) + 0.5;//0.5 means if 1.7->2, if 1.3->1
			int NumOfSubOBBY = NumOfSubOBB_model[1] * ((double)lengthOBB_object[1] / lengthOBB_model[1]) + 0.5;//0.5 means if 1.7->2, if 1.3->1
			if (NumOfSubOBBX > NumOfSubOBB_model[0]) NumOfSubOBBX = NumOfSubOBB_model[0];
			if (NumOfSubOBBY > NumOfSubOBB_model[1]) NumOfSubOBBY = NumOfSubOBB_model[1];
			if (NumOfSubOBBX == 0) NumOfSubOBBX = 1;
			if (NumOfSubOBBY == 0) NumOfSubOBBY = 1;
			onlineFile << "NumOfSubOBBX: " << NumOfSubOBBX << "\n";
			onlineFile << "NumOfSubOBBY: " << NumOfSubOBBY << "\n";

			// Because start position for computing descriptor is random at 4 lower corners of OBB, we need to
			// rotate object cloud to test with different position to choose the best one.
			double NCCmax = -1;
			double BestRota = -1 * DBL_MAX;
			double StartX = -1;
			double StartY = -1;
			int countView = 0;
			timer.start();
			SubOBB_matching_scence_model(*ObjectCloud, *ModelDescriptor, lengthOBB_model[1], lengthOBB_object[0], NumOfSubOBB_model[2], NumOfSubOBBX,
				NumOfSubOBBY, NumOfSubOBB_model[0], NumOfSubOBB_model[1], StartX, StartY, BestRota, NCCmax, countView);

			PCLBestRota->push_back(pcl::PointXYZ(StartX, StartY, BestRota));
			PCLBestView->push_back(pcl::PointXYZ(p_ModelOrientation[0], p_ModelOrientation[1], p_ModelOrientation[2]));
			PCLBestNCC->push_back(pcl::PointXYZ(NumOfSubOBBX, NumOfSubOBBY, NCCmax));
			PCLnumOfSubOBB->push_back(pcl::PointXYZ(NumOfSubOBB_model[0], NumOfSubOBB_model[1], NumOfSubOBB_model[2]));
			if (NCCmax >  NCCBest) NCCBest = NCCmax;
		}

		// Start new view from virtual camera
		countLine++;
		if (ModelDescriptor->size()) ModelDescriptor->clear();
		// Read Descriptor of model
		ReadPCLDescriptor(countLine, countLine, *ModelDescriptor);
	}
	if (PCLBestNCC->size() == 0)
	{
		onlineFile.close();
		return 1;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ObjectCloud_Buffer(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*ObjectCloud, *ObjectCloud_Buffer);

	double maxval = -1 * DBL_MAX;
	int maxIndex;
	while (true)
	{
		maxval = -1 * DBL_MAX;
		for (int numOfBestNCC = 0; numOfBestNCC < PCLBestNCC->size(); numOfBestNCC++)
		{
			if (PCLBestNCC->points[numOfBestNCC].z > maxval)
			{
				maxval = PCLBestNCC->points[numOfBestNCC].z;
				maxIndex = numOfBestNCC;
			}
		}
		double NCC_Ratio = 0.5;
		if (abs(maxval) < abs(NCC_Ratio * NCCBest) || maxval == -1)
		{
			onlineFile.close();
			return 2;
		}
		else
		{
			onlineFile << "\nNCC maxval: " << maxval << "\n";
			PCLBestNCC->points[maxIndex].z = -1;
			double p_numOfSubOBB[3];
			p_numOfSubOBB[0] = PCLnumOfSubOBB->points[maxIndex].x;
			p_numOfSubOBB[1] = PCLnumOfSubOBB->points[maxIndex].y;
			p_numOfSubOBB[2] = PCLnumOfSubOBB->points[maxIndex].z;
			// Read parameters of 3D virtual camera
			int NumOfSamplesX;
			double ZThresh;
			ReaVirtualCameraParameters(NumOfSamplesX, ZThresh);

			// Extract points on model similar with measured points
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr ModelTran(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr ExtractedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			if (VirtualPoints->size() != 0) VirtualPoints->clear();
			transformMatrix = Eigen::Affine3f::Identity();
			transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (PCLBestView->points[maxIndex].x / (double)180), Eigen::Vector3f::UnitX()));
			transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (PCLBestView->points[maxIndex].y / (double)180), Eigen::Vector3f::UnitY()));
			transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (PCLBestView->points[maxIndex].z / (double)180), Eigen::Vector3f::UnitZ()));
			pcl::transformPointCloud(*ModelCloud, *ModelTran, transformMatrix);

			SubOBB_ExtractVitualPoints(*ModelTran, *ExtractedCloud, *VirtualPoints, NumOfSamplesX, ZThresh, PCLBestRota->points[maxIndex].x,
				PCLBestNCC->points[maxIndex].x, PCLBestRota->points[maxIndex].y, PCLBestNCC->points[maxIndex].y, p_numOfSubOBB);
			matr_regis = matr.replicate(4, 4);
			matr_regis = matr_regis * transformMatrix.matrix();

			// Coarse Registration Align OBB of extracted model point cloud with OBB of measured object point cloud
			OBB_Calculation(*ExtractedCloud, corner_OBB, max_OBB, mid_OBB, min_OBB);
			RealcoordinateToOBBcoordinateXY(*ExtractedCloud, *ExtractedCloud, corner_OBB, max_OBB, mid_OBB, min_OBB);
			matr_mod = matr.replicate(4, 4);
			pcl::transformPointCloud(*ModelTran, *ModelTran, matr_mod);
			matr_regis = matr_mod * matr_regis;

			pcl::compute3DCentroid(*ExtractedCloud, center_mod);
			transformMatrix = Eigen::Affine3f::Identity();
			transformMatrix.translation() << -center_mod(0, 0), -center_mod(1, 0), -center_mod(2, 0);
			pcl::transformPointCloud(*ExtractedCloud, *ExtractedCloud, transformMatrix);
			pcl::transformPointCloud(*ModelTran, *ModelTran, transformMatrix);
			matr_regis = transformMatrix.matrix() * matr_regis;

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr ModelTran_Buffer(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::copyPointCloud(*ModelTran, *ModelTran_Buffer);

			// Because of kz = 1, object might be symmtrical along axis Ox or Oy, so need be checked by rotX 180 and rotY 180
			// If kx approximates ky, axis Ox and axis Oy will be ambiguous, so need be checked by rotZ by increment 90
			double increaZ = 90;
			if (abs(lengthOBB_object[0] - lengthOBB_object[1]) > 0.05*lengthOBB_object[0]) increaZ = 360;
			for (int rotZ = 0; rotZ < 360; rotZ += increaZ)
			for (int rotY = 0; rotY < 360; rotY += 180)
			for (int rotX = 0; rotX < 360; rotX += 180)
			{
				transformMatrix = Eigen::Affine3f::Identity();
				transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (rotX / (double)180), Eigen::Vector3f::UnitX()));
				transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (rotY / (double)180), Eigen::Vector3f::UnitY()));
				transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (rotZ / (double)180), Eigen::Vector3f::UnitZ()));
				pcl::transformPointCloud(*ObjectCloud_Buffer, *ObjectCloud, transformMatrix);
				pcl::copyPointCloud(*ModelTran_Buffer, *ModelTran);

				//pcl::copyPointCloud(*ObjectCloud, output);
				//output.operator+=(*ExtractedCloud);

				ICP_Registration(*ObjectCloud, *ModelTran, *ObjectCloud, *ObjectCloud, 100);
				double meandist;
				PCL_MeanDistance_CloudToCloud(*ObjectCloud, *ModelTran, meandist);
				onlineFile << "meandist: " << meandist << "\n";
				if (meandist < pass)
				{
					Eigen::Matrix4f matr_regis_Inver = Eigen::Matrix4f::Identity();
					matr_regis_Inver = matr_regis.inverse();
					pcl::transformPointCloud(*ObjectCloud, *ObjectCloud, matr_regis_Inver);
					pcl::copyPointCloud(*ObjectCloud, output);
					//output.operator+=(*ObjectCloud);
					onlineFile.close();
					return 0;
				}
			}
		}
	}
	onlineFile.close();
	return 0;
}

unsigned char cinspection::Optimized_SubOBB_Recognition(pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output, double NCCThresh,
	double tolerUp, double tolerDown, double pass)
{
	reportCount = 0;
	ofstream onlineFile;
	onlineFile.open("Report/Report_online.txt");
	QElapsedTimer timer;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ObjectCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ObjDescriptor(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ModelCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCLBestView(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCLBestRota(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCLBestNCC(new pcl::PointCloud<pcl::PointXYZ>); 
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCLBestNumOfSubOBB(new pcl::PointCloud<pcl::PointXYZ>);

	Eigen::Matrix4f matr_obj = Eigen::Matrix4f::Identity();
	Eigen::Affine3f transformMatrix = Eigen::Affine3f::Identity();

	int NumOfSamplesX;
	int numOfSubOBB_object[3];
	double ZThresh;

	// Read Points from model and parameters
	ReadPCLPointCloud_Opt(*ModelCloud);
	if (ModelCloud->size() == 0) return 3;
	TransferToOBB_CorrdinateXY(input, *ObjectCloud);
	matr_obj = matr.replicate(4, 4);
	ReaVirtualCameraParameters_Opt(NumOfSamplesX, ZThresh);
	numOfSubOBB_Estimation(*ObjectCloud, *ModelCloud, numOfSubOBB_object);

	double OBB_Length_Obj[3];
	double OBB_Length_Mod[3];
	OBB_Length(*ObjectCloud, OBB_Length_Obj);
	OBB_Length(*ModelCloud, OBB_Length_Mod);

	if (OBB_Length_Obj[0] > ((1 + tolerUp) * OBB_Length_Mod[0]) || OBB_Length_Obj[1] > ((1 + tolerUp) * OBB_Length_Mod[1]))
	{
		onlineFile.close();
		return 1;
	}		
	if (OBB_Length_Obj[0] < ((1 - tolerDown) * OBB_Length_Mod[0]) & OBB_Length_Obj[1] < ((1 - tolerDown) * OBB_Length_Mod[1]))
	{
		onlineFile.close();
		return 1;
	}
	
	if (SubOBB_matching_scence_model_Opt(*ObjectCloud, numOfSubOBB_object, *PCLBestView, *PCLBestRota, *PCLBestNCC, *PCLBestNumOfSubOBB))
	{
			onlineFile.close();
			return 1;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ObjectCloud_Buffer(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*ObjectCloud, *ObjectCloud_Buffer);
	Eigen::Matrix4f matr_obj_Buf = Eigen::Matrix4f::Identity();
	matr_obj_Buf = matr_obj.replicate(4, 4);

	double maxNCC = -1;
	int maxIndex;

	while (true)
	{
		maxNCC = -1;
		for (int numOfBestNCC = 0; numOfBestNCC < PCLBestNCC->size() - 1; numOfBestNCC++)
		{
			if (PCLBestNCC->points[numOfBestNCC].z > maxNCC)
			{
				maxNCC = PCLBestNCC->points[numOfBestNCC].z;
				maxIndex = numOfBestNCC;
			}
		}
		double NCC_Ratio = 0.5;//0.5
		if (abs(maxNCC) < abs(NCC_Ratio * PCLBestNCC->points[PCLBestNCC->size() - 1].z) || maxNCC == -1)
		{
			onlineFile.close();
			return 2;
		}
		else
		{
			onlineFile << "maxNCC: " << maxNCC << "\n";
			PCLBestNCC->points[maxIndex].z = -1;
			if (maxNCC < NCCThresh) continue;
			double bestView[3], bestRota[3], numOfOBB_mod[3], matchingPoint[3];
			bestView[0] = PCLBestView->points[maxIndex].x;
			bestView[1] = PCLBestView->points[maxIndex].y;
			bestView[2] = PCLBestView->points[maxIndex].z;
			bestRota[0] = PCLBestRota->points[maxIndex].x;
			bestRota[1] = PCLBestRota->points[maxIndex].y;
			bestRota[2] = PCLBestRota->points[maxIndex].z;
			numOfOBB_mod[0] = PCLBestNumOfSubOBB->points[maxIndex].x;
			numOfOBB_mod[1] = PCLBestNumOfSubOBB->points[maxIndex].y;
			numOfOBB_mod[2] = PCLBestNumOfSubOBB->points[maxIndex].z;
			matchingPoint[0] = PCLBestNCC->points[maxIndex].x;
			matchingPoint[1] = PCLBestNCC->points[maxIndex].y;
			matchingPoint[2] = PCLBestNCC->points[maxIndex].z;
			// Read parameters of 3D virtual camera

			// Extract points on model similar with measured points
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr ModelTran(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr ExtractedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::copyPointCloud(*ModelCloud, *ModelTran);
			if (SubOBB_ExtractVitualPoints_Opt(*ModelTran, *ExtractedCloud, bestView, bestRota, numOfOBB_mod,
				numOfSubOBB_object, matchingPoint, NumOfSamplesX, ZThresh))
			{
				onlineFile.close();
				return 1;
			}
			pcl::transformPointCloud(*ModelCloud, *ModelTran, matr);

			TransferToOBB_CorrdinateXY(*ExtractedCloud, *ExtractedCloud);
			pcl::transformPointCloud(*ModelTran, *ModelTran, matr);

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr ModelTran_Buffer(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::copyPointCloud(*ModelTran, *ModelTran_Buffer);
			// Because of kz = 1, object might be symmtrical along axis Ox or Oy, so need be checked by rotX 180 and rotY 180
			// If kx approximates ky, axis Ox and axis Oy will be ambiguous, so need be checked by rotZ by increment 90
			double increaZ = 90;
			if (numOfSubOBB_object[0] > numOfSubOBB_object[1]) increaZ = 360;
			for (int rotZ = 0; rotZ < 360; rotZ += increaZ)
			for (int rotY = 0; rotY < 360; rotY += 180)
			for (int rotX = 0; rotX < 360; rotX += 180)
			{
				transformMatrix = Eigen::Affine3f::Identity();
				transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (rotX / (double)180), Eigen::Vector3f::UnitX()));
				transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (rotY / (double)180), Eigen::Vector3f::UnitY()));
				transformMatrix.rotate(Eigen::AngleAxisf(M_PI * (rotZ / (double)180), Eigen::Vector3f::UnitZ()));
				pcl::transformPointCloud(*ObjectCloud_Buffer, *ObjectCloud, transformMatrix);
				matr_obj = transformMatrix.matrix() * matr_obj_Buf;
				pcl::copyPointCloud(*ModelTran_Buffer, *ModelTran);

				Inverse_ICP_Registration(*ObjectCloud, *ModelTran, *ModelTran, *ModelTran, 100);
				double meandist;
				PCL_MeanDistance_CloudToCloud(*ObjectCloud, *ModelTran, meandist);
				onlineFile << "meandist: " << meandist << "\n";
				if (meandist < pass)
				{
					Eigen::Matrix4f matr_obj_inver = Eigen::Matrix4f::Identity();
					matr_obj_inver = matr_obj.inverse();
					pcl::transformPointCloud(*ModelTran, *ModelTran, matr_obj_inver);
					pcl::copyPointCloud(*ModelTran, output);
					//output.operator+=(*ObjectCloud);
					onlineFile.close();
					return 0;
				}
			}
		}
	}

	pcl::copyPointCloud(*ModelCloud, output);
	onlineFile.close();
	return 0;
}

unsigned char cinspection::NonSeg_SubOBB_Recog(pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output, double pass)
{
	reportCount = 0;
	ofstream onlineFile;
	onlineFile.open("Report/Report_online.txt");
	QElapsedTimer timer;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SceneCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SceneDescriptor(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ObjectCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ObjDescriptor(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ModelCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PickCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCLBestView(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCLBestRota_mod(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr bestNCC(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr numOBB_mod(new pcl::PointCloud<pcl::PointXYZ>);

	Eigen::Affine3f transformMatrix = Eigen::Affine3f::Identity();

	int NumOfSamplesX;
	int numOfSubOBB_Scene[3];
	double ZThresh;

	// Read Points from model and parameters
	ReadPCLPointCloud_Nonseg(*ModelCloud);

	TransferToOBB_CorrdinateXY(input, *SceneCloud);
	ReaVirtualCameraParameters_Nonseg(NumOfSamplesX, ZThresh);
	numOfSubOBB_Estimation(*SceneCloud, *ModelCloud, numOfSubOBB_Scene);

	/*RG_SubOBB_StartFind(*SceneCloud, *PickCloud, numOfSubOBB_Scene);
	double MaxDense = -1 * DBL_MAX;
	int MaxIndex;
	int PickX, PickY;
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
	RG_SubOBB_ObjExtraction(*SceneCloud, *ObjectCloud, numOfSubOBB_Scene, PickX, PickY);*/

	double OBB_Length_Scene[3];
	double OBB_Length_Mod[3];
	OBB_Length(*SceneCloud, OBB_Length_Scene);
	OBB_Length(*ModelCloud, OBB_Length_Mod);

	timer.start();
	if (SubOBB_matching_model_scene_nonSeg(*SceneCloud, numOfSubOBB_Scene, *PCLBestView, *PCLBestRota_mod, *bestNCC, *numOBB_mod))
	{
		onlineFile.close();
		return 1;
	}
	
	onlineFile << "SubOBB_matching_model_scene_nonSeg: " << timer.elapsed() << " ms" << "\n";
	onlineFile << "numOfSubOBB_Scene[0]: " << numOfSubOBB_Scene[0] << "\n";
	onlineFile << "numOfSubOBB_Scene[1]: " << numOfSubOBB_Scene[1] << "\n";

	onlineFile << "PCLBestView: " << PCLBestView->size() << "\n";
	onlineFile << "PCLBestRota_mod: " << PCLBestRota_mod->size() << "\n";
	onlineFile << "bestNCC: " << bestNCC->size() << "\n";
	onlineFile << "numOBB_mod: " << numOBB_mod->size() << "\n";

	int NumObb_Obj[3], starPt[3];
	double MaxNCC = -1;
	for (int j = 0; j < SceneCloud->size(); j++)
	{
		SceneCloud->points[j].r = 255;
		SceneCloud->points[j].g = 255;
		SceneCloud->points[j].b = 255;
	}
	pcl::copyPointCloud(*SceneCloud, output);
	for (int i = 1; i < 2; i++)
	{
		MaxNCC = -1; int maxPt = -1;
		for (int k = 0; k < bestNCC->size(); k++)
		{
			if (MaxNCC < bestNCC->points[k].z)
			{
				MaxNCC = bestNCC->points[k].z;
				maxPt = k;
			}
		}
		
		NumObb_Obj[0] = numOBB_mod->points[maxPt].x; NumObb_Obj[1] = numOBB_mod->points[maxPt].y;
		starPt[0] = bestNCC->points[maxPt].x; starPt[1] = bestNCC->points[maxPt].y;
		Nonseg_SubOBB_ObjExtraction(*SceneCloud, *ObjectCloud, NumObb_Obj, starPt);
		onlineFile << "bestNCC: " << starPt[0] << " " << starPt[1] << " " << bestNCC->points[maxPt].z << "\n";
		onlineFile << "NumObb_Obj: " << NumObb_Obj[0] << " " << NumObb_Obj[1] << " " << NumObb_Obj[2] << "\n";
		if (ObjectCloud->size() == 0) continue;
		if (i == 1) for (int j = 0; j < ObjectCloud->size(); j++)
		{
			ObjectCloud->points[j].r = 255; ObjectCloud->points[j].g = 0; ObjectCloud->points[j].b = 0;
		}
		if (i == 2) for (int j = 0; j < ObjectCloud->size(); j++) {
			ObjectCloud->points[j].r = 0; ObjectCloud->points[j].g = 255; ObjectCloud->points[j].b = 0;
		}
		if (i == 3) for (int j = 0; j < ObjectCloud->size(); j++) {
			ObjectCloud->points[j].r = 0; ObjectCloud->points[j].g = 0; ObjectCloud->points[j].b = 255;
		}
		output.operator+=(*ObjectCloud);
		bestNCC->points[maxPt].z = -1;
	}
	
	for (int i = 0; i < ObjectCloud->size(); i++) ObjectCloud->points[i].r = 255;

	onlineFile.close();
	return 0;
}
