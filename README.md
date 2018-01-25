# 3D-Inspection


[Reference](http://iopscience.iop.org/article/10.1088/1361-6501/aa513a/meta)

[Watch video](https://www.youtube.com/watch?v=bAdFVLzUHoU)

[Watch video](https://www.youtube.com/watch?v=61RKefFBRzo)

The project aims to develop a valued-added machine tool controller by integrating intelligent 3-D machine vision and measurement capability for automated workpiece pick & place and 3-D dimension or profile measurement.
Thus, the project aims to achieve the following solid objectives both in academic and industrial benefits:
1/ To develop innovative 3-D optical probes and measuring technologies for in-situ 3-D measurement on workpieces. An algorithm for generation of object 3-D cloud models will be developed for object recognition and automated orientation detection.
2/ To develop a 1+2 intelligent value-added controller for machine tools for in-situ automated workpiece pick & place and 3-D dimension or profile measurement.

Our approach mainly comprises two important elements in the determination of next best probe pose and multiple-view point clouds registration. A novel technique is proposed to register 3-D object scene with overlapped or stacked condition. Under this scenario, conventional registration methods such as the iterative closest point algorithm usually fail to converge to a global minimum when a good initial estimate for image registration does not exist. Our proposed technique uses a 3-D scanner to be mounted on a six degree of freedom-articulated industrial robot. It keeps moving probe continuously in the working
range against the object and autonomously varying the probe with various gestures required for complete object scanning and for achieving best 3-D sensing accuracy. The robot scanning path is determined through a proposed algorithm using information from the latest scanning data and registered result of the object.

![alt text](https://github.com/hoangcuongbk80/3D-Inspection/blob/master/docs/images/bigBlade.png)

![alt text](https://github.com/hoangcuongbk80/3D-Inspection/blob/master/docs/images/blades.png)

![alt text](https://github.com/hoangcuongbk80/3D-Inspection/blob/master/docs/images/golfHead5.png)

The key technical breakthrough of the developed approach can enablerobust object recognition and localization under undesirable condition such as environmentalillumination variation as well as optical occlusion to viewing the object partially. First,the acquired point clouds are segmented into individual object point clouds based on thedeveloped 3D object segmentation for randomly stacked objects. Second, an efficient shape-matching algorithm called Sub-OBB based object recognition by using the proposed orientedbounding box (OBB) regional area-based descriptor is performed to reliably recognize theobject. Then, the 3D position and orientation of the object can be roughly estimated byaligning the OBB of segmented object point cloud with OBB of matched point cloud in adatabase generated from CAD model and 3D virtual camera. To detect accurate pose of theobject, the iterative closest point (ICP) algorithm is used to match the object model with thesegmented point clouds. From the feasibility test of several scenarios, the developed approachis verified to be feasible for object pose recognition and localization.

![alt text](https://github.com/hoangcuongbk80/3D-Inspection/blob/master/docs/images/finalRecognition.png)
