///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2016, Carnegie Mellon University and University of Cambridge,
// all rights reserved.
//
// THIS SOFTWARE IS PROVIDED “AS IS” FOR ACADEMIC USE ONLY AND ANY EXPRESS
// OR IMPLIED WARRANTIES WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY.
// OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Notwithstanding the license granted herein, Licensee acknowledges that certain components
// of the Software may be covered by so-called “open source” software licenses (“Open Source
// Components”), which means any software licenses approved as open source licenses by the
// Open Source Initiative or any substantially similar licenses, including without limitation any
// license that, as a condition of distribution of the software licensed under such license,
// requires that the distributor make the software available in source code format. Licensor shall
// provide a list of Open Source Components for a particular version of the Software upon
// Licensee’s request. Licensee will comply with the applicable terms of such licenses and to
// the extent required by the licenses covering Open Source Components, the terms of such
// licenses will apply in lieu of the terms of this Agreement. To the extent the terms of the
// licenses applicable to Open Source Components prohibit any of the restrictions in this
// License Agreement with respect to such Open Source Component, such restrictions will not
// apply to such Open Source Component. To the extent the terms of the licenses applicable to
// Open Source Components require Licensor to make an offer to provide source code or
// related information in connection with the Software, such offer is hereby made. Any request
// for source code or related information should be directed to cl-face-tracker-distribution@lists.cam.ac.uk
// Licensee acknowledges receipt of notices for the Open Source Components for the initial
// delivery of the Software.

//     * Any publications arising from the use of this software, including but
//       not limited to academic journal and conference publications, technical
//       reports and manuals, must cite at least one of the following works:
//
//       OpenFace: an open source facial behavior analysis toolkit
//       Tadas Baltrušaitis, Peter Robinson, and Louis-Philippe Morency
//       in IEEE Winter Conference on Applications of Computer Vision, 2016  
//
//       Rendering of Eyes for Eye-Shape Registration and Gaze Estimation
//       Erroll Wood, Tadas Baltrušaitis, Xucong Zhang, Yusuke Sugano, Peter Robinson, and Andreas Bulling 
//       in IEEE International. Conference on Computer Vision (ICCV),  2015 
//
//       Cross-dataset learning and person-speci?c normalisation for automatic Action Unit detection
//       Tadas Baltrušaitis, Marwa Mahmoud, and Peter Robinson 
//       in Facial Expression Recognition and Analysis Challenge, 
//       IEEE International Conference on Automatic Face and Gesture Recognition, 2015 
//
//       Constrained Local Neural Fields for robust facial landmark detection in the wild.
//       Tadas Baltrušaitis, Peter Robinson, and Louis-Philippe Morency. 
//       in IEEE Int. Conference on Computer Vision Workshops, 300 Faces in-the-Wild Challenge, 2013.    
//
///////////////////////////////////////////////////////////////////////////////


// FaceTrackingVidMulti.cpp : Defines the entry point for the multiple face tracking console application.
#include "LandmarkCoreIncludes.h"
#include "GazeEstimation.h"
#include <Face_utils.h>
#include <FaceAnalyser.h>

// Boost includes
#include <filesystem.hpp>
#include <filesystem/fstream.hpp>


#include <fstream>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "OpenFace/My_message.h"
#include "OpenFace/pose_message_all.h"


#include <sys/socket.h>


// OpenCV includes
#include <opencv2/videoio/videoio.hpp>  // Video write
#include <opencv2/videoio/videoio_c.h>  // Video write
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>



#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <boost/ref.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/utility/result_of.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <math.h>       /* atan2 */
#include <cmath>        // std::atan(double)
#include <valarray>     // std::valarray, std::atan2
#include <numeric>




#define INFO_STREAM( stream ) \
std::cout << stream << std::endl

#define WARN_STREAM( stream ) \
std::cout << "Warning: " << stream << std::endl

#define ERROR_STREAM( stream ) \
std::cout << "Error: " << stream << std::endl

static void printErrorAndAbort( const std::string & error )
{
    std::cout << error << std::endl;
    abort();
}

#define FATAL_STREAM( stream ) \
printErrorAndAbort( std::string( "Fatal error: " ) + stream )

using namespace std;











vector<string> get_arguments(int argc, char **argv)
{

	vector<string> arguments;

	for(int i = 0; i < argc; ++i)
	{
		arguments.push_back(string(argv[i]));
	}
	return arguments;
}


void NonOverlapingDetections(const vector<LandmarkDetector::CLNF>& clnf_models, vector<cv::Rect_<double> >& face_detections)
{

	// Go over the model and eliminate detections that are not informative (there already is a tracker there)
	for(size_t model = 0; model < clnf_models.size(); ++model)
	{

		// See if the detections intersect
		cv::Rect_<double> model_rect = clnf_models[model].GetBoundingBox();
		
		for(int detection = face_detections.size()-1; detection >=0; --detection)
		{
			double intersection_area = (model_rect & face_detections[detection]).area();
			double union_area = model_rect.area() + face_detections[detection].area() - 2 * intersection_area;

			// If the model is already tracking what we're detecting ignore the detection, this is determined by amount of overlap
			if( intersection_area/union_area > 0.5)
			{
				face_detections.erase(face_detections.begin() + detection);
			}
		}
	}
}



static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher pose_gaze_pub = nh_.advertise<OpenFace::pose_message_all>("pose_gaze", 1000);

  int c=10;
  //int argc;
  //char **argv;
  bool cx_undefined;
  float fx = 600, fy = 600, cx = 0, cy = 0;
  int device = 0;
  vector<string> files, depth_directories, tracked_videos_output, dummy_out;
  vector<string> arguments;
  vector<LandmarkDetector::CLNF> clnf_models;
  vector<bool> active_models;
  int num_faces_max = 4;
  int frame_count = 0;
  int t1,t0 = cv::getTickCount();
  double fps = 10;
  bool use_depth=false;
  vector<LandmarkDetector::FaceModelParameters> det_parameters;

  //INICIALIZACAO ERRADA AQUI!!!!!!!!!!!!!!!!!!!! ??????????????
  LandmarkDetector::FaceModelParameters det_params;
  
  bool done = false;	
  int f_n = -1;
  bool u;
  string output_codec;
  std::map <int, cv::Point3f> gaze0_models;
  std::map <int, cv::Point3f> gaze1_models;
  cv::VideoCapture video_capture;
  cv::VideoWriter writerFace;

  int kk;







public:
  ImageConverter(int argc, char **argv, LandmarkDetector::FaceModelParameters det_params_cop)
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    

    //ORIGINAL !!!!!!!
    image_sub_ = it_.subscribe("/camera/image", 1, 
    &ImageConverter::imageCb, this);

    //VIZZY
  	//image_sub_ = it_.subscribe("/vizzy/l_camera/image_rect_color", 1, 
    //&ImageConverter::imageCb, this);



  	//kinect mongo db
    //image_sub_ = it_.subscribe("BringInAnImage", 1, 
    //&ImageConverter::imageCb, this);

  	//kinect directly
  	//image_sub_ = it_.subscribe("/head/kinect2/k2_rgb/image", 1, 
    //&ImageConverter::imageCb, this);



    kk=10;

    arguments = get_arguments(argc, argv);

    det_params=det_params_cop;

	cx_undefined = false;
	if(cx == 0 || cy == 0)
	{
		cx_undefined = true;
	}




	//vector<LandmarkDetector::FaceModelParameters> det_parameters;
	det_parameters.push_back(det_params);


	LandmarkDetector::get_video_input_output_params(files, depth_directories, dummy_out, tracked_videos_output, u, output_codec, arguments);
	// Get camera parameters
	LandmarkDetector::get_camera_params(device, fx, fy, cx, cy, arguments);



	LandmarkDetector::CLNF clnf_model(det_parameters[0].model_location);
	clnf_model.face_detector_HAAR.load(det_parameters[0].face_detector_location);
	clnf_model.face_detector_location = det_parameters[0].face_detector_location;

	clnf_models.reserve(num_faces_max);

	clnf_models.push_back(clnf_model);
	active_models.push_back(false);

	for (int i = 1; i < num_faces_max; ++i)
	{
		clnf_models.push_back(clnf_model);
		active_models.push_back(false);
		det_parameters.push_back(det_params);
	}

	




 }

  ~ImageConverter()
  {
    cv::destroyWindow("tracking_result");
    
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {


    cv_bridge::CvImagePtr cv_ptr;
    try
    {


        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      

        std::cout <<"the value for gaze track is "<<std::endl;

        
	    if(cx_undefined)
		{
			cx = cv_ptr->image.cols / 2.0f;
			cy = cv_ptr->image.rows / 2.0f;
		}
		

			//det_params.use_face_template = true;
			//det_params.track_gaze = true;
			//det_params.curr_face_detector = LandmarkDetector::FaceModelParameters::HOG_SVM_DETECTOR;
			
			// This is so that the model would not try re-initialising itself
			//det_params.reinit_video_every = -1;

			// Gaze tracking, absolute gaze direction
			cv::Point3f gazeDirection0(0, 0, -1);
			cv::Point3f gazeDirection1(0, 0, -1);

			// Reading the images
			cv::Mat_<float> depth_image;
			cv::Mat_<uchar> grayscale_image;

			cv::Mat disp_image = cv_ptr->image.clone();

			if(cv_ptr->image.channels() == 3)
			{
	
				cv::cvtColor(cv_ptr->image, grayscale_image, CV_BGR2GRAY);
			}
			else
			{	

				grayscale_image = cv_ptr->image.clone();				
			}

			
			// Get depth image
			if(use_depth)
			{
				char* dst = new char[100];
				std::stringstream sstream;

				sstream << depth_directories[f_n] << "\\depth%05d.png";
				sprintf(dst, sstream.str().c_str(), frame_count + 1);
				// Reading in 16-bit png image representing depth
				cv::Mat_<short> depth_image_16_bit = cv::imread(string(dst), -1);

				// Convert to a floating point depth image
				if(!depth_image_16_bit.empty())
				{
					depth_image_16_bit.convertTo(depth_image, CV_32F);
				}
				else
				{
					WARN_STREAM( "Can't find depth image" );
				}
			}

			vector<cv::Rect_<double> > face_detections;

			bool all_models_active = true;
			for(unsigned int model = 0; model < clnf_models.size(); ++model)
			{
				if(!active_models[model])
				{
					all_models_active = false;
				}
			}
			
						
			// Get the detections (every 8th frame and when there are free models available for tracking)
			//int exp = frame_count % 64;
			//std::cout <<exp<<std::endl;

			

			
			// Get the detections (every 8th frame and when there are free models available for tracking)
			if(frame_count % 8 == 0 && !all_models_active)
			{				
				if(det_parameters[0].curr_face_detector == LandmarkDetector::FaceModelParameters::HOG_SVM_DETECTOR)
				{
					vector<double> confidences;
					LandmarkDetector::DetectFacesHOG(face_detections, grayscale_image, clnf_models[0].face_detector_HOG, confidences);
				}
				else
				{
					LandmarkDetector::DetectFaces(face_detections, grayscale_image, clnf_models[0].face_detector_HAAR);
				}

			}

			// Keep only non overlapping detections (also convert to a concurrent vector
			NonOverlapingDetections(clnf_models, face_detections);

			vector<tbb::atomic<bool> > face_detections_used(face_detections.size());

			// Go through every model and update the tracking
			tbb::parallel_for(0, (int)clnf_models.size(), [&](int model){
			//for(unsigned int model = 0; model < clnf_models.size(); ++model)
			//{

				bool detection_success = false;

				// If the current model has failed more than 4 times in a row, remove it
				if(clnf_models[model].failures_in_a_row > 4)
				{				
					active_models[model] = false;
					clnf_models[model].Reset();

				}

				// If the model is inactive reactivate it with new detections
				if(!active_models[model])
				{
					
					for(size_t detection_ind = 0; detection_ind < face_detections.size(); ++detection_ind)
					{
						// if it was not taken by another tracker take it (if it is false swap it to true and enter detection, this makes it parallel safe)
						if(face_detections_used[detection_ind].compare_and_swap(true, false) == false)
						{
					
							// Reinitialise the model
							clnf_models[model].Reset();

							// This ensures that a wider window is used for the initial landmark localisation
							clnf_models[model].detection_success = false;
							detection_success = LandmarkDetector::DetectLandmarksInVideo(grayscale_image, depth_image, face_detections[detection_ind], clnf_models[model], det_parameters[model]);
													
							// This activates the model
							active_models[model] = true;

							

							if (det_params.track_gaze && detection_success && clnf_models[model].eye_model)
							{	
								// Gaze tracking, absolute gaze direction
								cv::Point3f gazeDirection0(0, 0, -1);
								cv::Point3f gazeDirection1(0, 0, -1);
								FaceAnalysis::EstimateGaze(clnf_models[model], gazeDirection0, fx, fy, cx, cy, true);
								gaze0_models[model] = gazeDirection0;
								FaceAnalysis::EstimateGaze(clnf_models[model], gazeDirection1, fx, fy, cx, cy, false);
								gaze1_models[model] = gazeDirection1;
								
							}

							// break out of the loop as the tracker has been reinitialised
							break;
						}

					}
				}
				else
				{
					// The actual facial landmark detection / tracking
					detection_success = LandmarkDetector::DetectLandmarksInVideo(grayscale_image, depth_image, clnf_models[model], det_parameters[model]);

					

					if (det_params.track_gaze && detection_success && clnf_models[model].eye_model)
							{	

								// Gaze tracking, absolute gaze direction
								cv::Point3f gazeDirection0(0, 0, -1);
								cv::Point3f gazeDirection1(0, 0, -1);
								FaceAnalysis::EstimateGaze(clnf_models[model], gazeDirection0, fx, fy, cx, cy, true);
								gaze0_models[model] = gazeDirection0;
								FaceAnalysis::EstimateGaze(clnf_models[model], gazeDirection1, fx, fy, cx, cy, false);
								gaze1_models[model] = gazeDirection1;
								
							}
				}
			});
			int total_models=0;	
			//to send the info of all persons detected
			OpenFace::pose_message_all all_pose_gaze_inf;
				
			// Go through every model and visualise the results
			for(size_t model = 0; model < clnf_models.size(); ++model)
			{
				// Visualising the results
				// Drawing the facial landmarks on the face and the bounding box around it if tracking is successful and initialised
				double detection_certainty = clnf_models[model].detection_certainty;

				double visualisation_boundary = -0.1;
			
				// Only draw if the reliability is reasonable, the value is slightly ad-hoc
				if(detection_certainty < visualisation_boundary)
				{
					LandmarkDetector::Draw(disp_image, clnf_models[model]);

					if(detection_certainty > 1)
						detection_certainty = 1;
					if(detection_certainty < -1)
						detection_certainty = -1;

					detection_certainty = (detection_certainty + 1)/(visualisation_boundary +1);

					// A rough heuristic for box around the face width
					int thickness = (int)std::ceil(2.0* ((double)cv_ptr->image.cols) / 640.0);
					
					// Work out the pose of the head from the tracked model

					cv::Vec6d pose_estimate = LandmarkDetector::GetCorrectedPoseWorld(clnf_models[model], fx, fy, cx, cy);
					//cv::Vec6d pose_estimate = LandmarkDetector::GetCorrectedPoseCamera(clnf_models[model], fx, fy, cx, cy);

					//CAMERA REFERENTIAL
					//Z POINTS TO PERSON , Y POINTS DOWN , X POINTS TO LEFT OF PERSON(RIGHT OF CAMERA)

					//TRYING TO ESTIMATE!!!!!!

					cv::Vec3d orientation_f(pose_estimate(3),pose_estimate(4),pose_estimate(5));


					double alfa= atan(pose_estimate(0)/pose_estimate(2));
					//std::cout << "this is ALFFA" << alfa << std::endl;



					//cv::Matx33d rotation_matrix_pose;
					//cv::Rodrigues(orientation_f, rotation_matrix_pose);
					
					//std::cout << "rotation_matrix of my pose " << rotation_matrix_pose << std::endl;


					
					cv::Matx33d rotation_matrix;
					rotation_matrix(0,0) = cos(alfa);
					rotation_matrix(0,1) = 0;
					rotation_matrix(0,2) = -sin(alfa);
					rotation_matrix(1,0) = 0;
					rotation_matrix(1,1) = 1;
					rotation_matrix(1,2) = 0;
					rotation_matrix(2,0) = sin(alfa);
					rotation_matrix(2,1) = 0;
					rotation_matrix(2,2) = cos(alfa);


					

					//TRYING TO ESTIMATE!!!!!!
					double t= std::atan((pose_estimate(1)*cos(alfa))/pose_estimate(2));

					
					std::cout << "this is t angle" << t<< std::endl;

					
					cv::Matx33d rotation_matrix2;
					rotation_matrix2(0,0) = 1;
					rotation_matrix2(0,1) = 0;
					rotation_matrix2(0,2) = 0;
					rotation_matrix2(1,0) = 0;
					rotation_matrix2(1,1) = cos(t);
					rotation_matrix2(1,2) = -sin(t);
					rotation_matrix2(2,0) = 0;
					rotation_matrix2(2,1) =  sin(t);
					rotation_matrix2(2,2) = cos(t);



					
					
					


					//std::cout << model << "this is the pose in degree" << pose_estimate_deg << std::endl;


					//I CHANGED THIS HERE, It is printing the pose of the persons
					//std::cout << model << "this is the pose" << pose_estimate << std::endl;


					cv::Matx33d rotation_matrix_final;

					orientation_f[0] =pose_estimate(3);
					orientation_f[1] =pose_estimate(4);
					orientation_f[2] =pose_estimate(5);

					cv::Matx33d rotation_matrix_pose;

					cv::Rodrigues(orientation_f, rotation_matrix_pose);

					std::cout << model << "THis is rodrigues matrix" << rotation_matrix_pose << std::endl;


					rotation_matrix_final = rotation_matrix*rotation_matrix2*rotation_matrix_pose;
					cv::Rodrigues(rotation_matrix_final, orientation_f); 

					orientation_f[0]=orientation_f[0]*180/3.14;
					orientation_f[1]=orientation_f[1]*180/3.14;
					orientation_f[2]=orientation_f[2]*180/3.14;
					
					std::cout << model << "THis is the orientation after ajustments in degree" << orientation_f << std::endl;



					//I CHANGED THIS HERE, It is printing the gaze of the persons
					//first calculate it in angles

					

				
					cv::Vec3d vec_gaze_0(acos(gaze0_models[model].x/norm(gaze0_models[model])),acos(gaze0_models[model].y/norm(gaze0_models[model])),acos(gaze0_models[model].z/norm(gaze0_models[model])));
					cv::Vec3d vec_gaze_1(acos(gaze1_models[model].x/norm(gaze1_models[model])),acos(gaze1_models[model].y/norm(gaze1_models[model])),acos(gaze1_models[model].z/norm(gaze0_models[model])));


					cv::Vec3d vec_gaze_00;
					cv::Vec3d vec_gaze_11;

					vec_gaze_00[0]=vec_gaze_0[0]*180/3.14;
					vec_gaze_00[1]=vec_gaze_0[1]*180/3.14;
					vec_gaze_00[2]=vec_gaze_0[2]*180/3.14;
					

					vec_gaze_11[0]=vec_gaze_1[0]*180/3.14;
					vec_gaze_11[1]=vec_gaze_1[1]*180/3.14;
					vec_gaze_11[2]=vec_gaze_1[2]*180/3.14;
					

					std::cout << model << "this is the gaze 0 BEFORE" << vec_gaze_00<< std::endl;
					std::cout << model << "this is the gaze 1 BEFORE" << vec_gaze_11<< std::endl;



					cv::Vec3d gazeVecAxis_ideal(pose_estimate(0),pose_estimate(1),pose_estimate(2));
					//calculate the direction vector that should look to camera, which is the vector from origin to head pose
					cv::Vec3d ideal_gaze_vec(acos(-pose_estimate(0)/norm(gazeVecAxis_ideal)),acos(-pose_estimate(1)/norm(gazeVecAxis_ideal)),acos(-pose_estimate(2)/norm(gazeVecAxis_ideal)));

					
					ideal_gaze_vec[0]=ideal_gaze_vec[0]*180/3.14;
					ideal_gaze_vec[1]=ideal_gaze_vec[1]*180/3.14;
					ideal_gaze_vec[2]=ideal_gaze_vec[2]*180/3.14;
					std::cout << model << "this is the IDEAL" << ideal_gaze_vec<< std::endl;


					//Calculate the dot product between the ideal and the original, close to 1 means they are similar
					/*
					cv::Vec3d vec_gaze_0_dot(gaze0_models[model].x/norm(gaze0_models[model]),gaze0_models[model].y/norm(gaze0_models[model]),gaze0_models[model].z/norm(gaze0_models[model]));
					cv::Vec3d vec_gaze_1_dot(gaze1_models[model].x/norm(gaze1_models[model]),gaze1_models[model].y/norm(gaze1_models[model]),gaze1_models[model].z/norm(gaze0_models[model]));
					cv::Vec3d measured_vec_gaze = (vec_gaze_0_dot + vec_gaze_1_dot)/2;
					cv::Vec3d ideal_dot_gaze =	-gazeVecAxis_ideal/norm(gazeVecAxis_ideal);
	                std::vector<double> a{ideal_dot_gaze[0],ideal_dot_gaze[1],ideal_dot_gaze[2]};
				    std::vector<double> b{measured_vec_gaze[0],measured_vec_gaze[1],measured_vec_gaze[2]};
				    double r1 = std::inner_product(a.begin(), a.end(), b.begin(), 0.0);
				    std::cout << "Inner product of a and b: " << r1 << '\n';
					*/




					// Draw it in reddish if uncertain, blueish if certain
					LandmarkDetector::DrawBox(disp_image, pose_estimate, cv::Scalar((1-detection_certainty)*255.0,0, detection_certainty*255), thickness, fx, fy, cx, cy);

					if (det_params.track_gaze &&  clnf_models[model].eye_model)
					{

						FaceAnalysis::DrawGaze(disp_image, clnf_models[model], gaze0_models[model], gaze1_models[model], fx, fy, cx, cy);
					}
					//I CHANGED THIS HERE, It is printing the gaze of the persons
					//std::cout << model << "this is the gaze 0 AFTER" << vec_gaze_0<< std::endl;
					//std::cout << model << "this is the gaze 1 AFTER" << vec_gaze_1<< std::endl;

					//calculate the difference
					cv::Vec3d difference_gaze(ideal_gaze_vec[0]-(0.5*(vec_gaze_00[0]+vec_gaze_11[0])),ideal_gaze_vec[1]-(0.5*(vec_gaze_00[1]+vec_gaze_11[1])),ideal_gaze_vec[2]-(0.5*(vec_gaze_00[2]+vec_gaze_11[2])));

					std::cout << model << "this is the difference in gaze" <<  difference_gaze << std::endl;


					//estimate the bonding box to print the results

					cv::Rect_<double> model_rect = clnf_models[model].GetBoundingBox();


					//Publish the results to ROS node
					
				    //To send pose
					OpenFace::My_message pose_gaze_inf;
	  				
	  				pose_gaze_inf.pose_tra_x = pose_estimate(0);
	  				pose_gaze_inf.pose_tra_y = pose_estimate(1);
					pose_gaze_inf.pose_tra_z = pose_estimate(2);
					pose_gaze_inf.pose_rot_x = orientation_f[0];
	  				pose_gaze_inf.pose_rot_y = orientation_f[1];
					pose_gaze_inf.pose_rot_z = orientation_f[2];

					pose_gaze_inf.gaze_0_rot_x = gaze0_models[model].x;
	  				pose_gaze_inf.gaze_0_rot_y = gaze0_models[model].y;
					pose_gaze_inf.gaze_0_rot_z = gaze0_models[model].z;
					pose_gaze_inf.gaze_1_rot_x = gaze1_models[model].x;
	  				pose_gaze_inf.gaze_1_rot_y = gaze1_models[model].y;
					pose_gaze_inf.gaze_1_rot_z = gaze1_models[model].z;

					pose_gaze_inf.diff_gaze_x = difference_gaze[0];
					pose_gaze_inf.diff_gaze_y = difference_gaze[1];
					pose_gaze_inf.diff_gaze_z = difference_gaze[2];

					pose_gaze_inf.box_h = model_rect.height;
					pose_gaze_inf.box_w = model_rect.width;
					pose_gaze_inf.box_x =model_rect.x;
					pose_gaze_inf.box_y =model_rect.y;
							  


					pose_gaze_inf.id_model = model;


					all_pose_gaze_inf.person.push_back(pose_gaze_inf);
					total_models=total_models+1;

				}
			}
			//publish everthing to Ros node
			if(total_models>0){
				all_pose_gaze_inf.total_models=total_models;	
				pose_gaze_pub.publish(all_pose_gaze_inf);
			}	
						// Work out the framerate
			if(frame_count % 10 == 0)
			{      
				t1 = cv::getTickCount();
				fps = 10.0 / (double(t1-t0)/cv::getTickFrequency()); 
				t0 = t1;
			}
			
			// Write out the framerate on the image before displaying it
			char fpsC[255];
			sprintf(fpsC, "%d", (int)fps);
			string fpsSt("FPS:");
			fpsSt += fpsC;
			cv::putText(disp_image, fpsSt, cv::Point(10,20), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255,0,0), 1, CV_AA);
			
			int num_active_models = 0;

			for( size_t active_model = 0; active_model < active_models.size(); active_model++)
			{
				if(active_models[active_model])
				{
					num_active_models++;
				}
			}

			char active_m_C[255];
			sprintf(active_m_C, "%d", num_active_models);
			string active_models_st("Active models:");
			active_models_st += active_m_C;
			cv::putText(disp_image, active_models_st, cv::Point(10,60), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255,0,0), 1, CV_AA);
			
			if(!det_parameters[0].quiet_mode)
			{
				cv::namedWindow("tracking_result",1);
				cv::imshow("tracking_result", disp_image);

				if(!depth_image.empty())
				{
					// Division needed for visualisation purposes
					imshow("depth", depth_image/2000.0);
				}
			}

			// output the tracked video
			if(!tracked_videos_output.empty())
			{		
				writerFace << disp_image;
			}

			video_capture >> cv_ptr->image;
		
			// detect key presses
			char character_press = cv::waitKey(1);
			
			// restart the trackers
			if(character_press == 'r')
			{
				for(size_t i=0; i < clnf_models.size(); ++i)
				{
					clnf_models[i].Reset();
					active_models[i] = false;
				}
			}
			// quit the application
			else if(character_press=='q')
			{
				//return(0);
			}

			// Update the frame count
			frame_count++;


    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }


  }
};







int main(int argc, char **argv)
{
	


  	vector<string> arguments;

	arguments = get_arguments(argc, argv);

	LandmarkDetector::FaceModelParameters det_params(arguments);
	
	
	det_params.use_face_template = true;
	det_params.track_gaze = true;
	// This is so that the model would not try re-initialising itself
	det_params.reinit_video_every = -1;
	det_params.curr_face_detector = LandmarkDetector::FaceModelParameters::HOG_SVM_DETECTOR;
	
	ros::init(argc, argv, "image_converter");
	ImageConverter ic(argc,argv,det_params);
	ros::spin();
	return 0;


}









