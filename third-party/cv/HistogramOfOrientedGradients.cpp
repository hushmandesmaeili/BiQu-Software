#include <iostream>
#include <stdexcept>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#include <librealsense2/rs.hpp>
using namespace cv;
using namespace std;
#include <math.h>




//This is drawing the bounding box and return coordinates of center of box

int MOSSE(Mat &frame,Rect &main_rect, rs2::pipeline &p, rs2_intrinsics &depth_intr) {

	// Create tracker, select region-of-interest (ROI) and initialize the tracker
	cv::Ptr<cv::Tracker> tracker = TrackerKCF::create();
	cv::Rect trackingBox = main_rect;
	tracker->init(frame, trackingBox);

  rs2::align align_to_depth(RS2_STREAM_DEPTH);
  rs2::align align_to_color(RS2_STREAM_COLOR);

	// Loop through available frames
	for (;;) {



    rs2::frameset frames = p.wait_for_frames();
    frames = align_to_depth.process(frames);
    auto colored_frame = frames.get_color_frame();
    rs2::depth_frame depth = frames.get_depth_frame();





      const int w = colored_frame.as<rs2::video_frame>().get_width();
      const int h = colored_frame.as<rs2::video_frame>().get_height();
      Mat frame(Size(w, h), CV_8UC3, (void*)colored_frame.get_data(), Mat::AUTO_STEP);
		// Update the tracker and draw the rectangle around target if update was successful
    rs2::depth_frame filtered_depth = depth;



     if (tracker->update(frame, trackingBox)) {


			cv::rectangle(frame, trackingBox, cv::Scalar(255, 0, 0), 2, 8);
		}
    float width = depth.get_width();
    float height = depth.get_height();
    //cout << " rectangle top left " << main_rect.tl() << " rectangle bottom right " << main_rect.br();
    //cout << "width of rectangle " << (main_rect.br().x - main_rect.tl().x) << " height of rectangle " << main_rect.br().y - main_rect.tl().y << "\n";
    // Query the distance from the camera to the object in the center of the image
    float dist_to_center = filtered_depth.get_distance((trackingBox.br().x + trackingBox.tl().x)/2, (trackingBox.br().y + trackingBox.tl().y)/2);

    float pixel[2] = {(trackingBox.br().x + trackingBox.tl().x)/2, (trackingBox.br().y + trackingBox.tl().y)/2 };
    float point[3];
    rs2_deproject_pixel_to_point(point, &depth_intr, pixel, dist_to_center);
    float angle  = atan2(point[2],point[0]);
    angle  = angle *180/3.14;
    //circle(trackingBox, Point2i((trackingBox.br().x - trackingBox.tl().x)), (trackingBox.br().y - trackingBox.tl().y)), 5, Scalar(0,125,230), 4, 3);
    cv::circle(frame, Point2i((trackingBox.br().x + trackingBox.tl().x)/2,(trackingBox.br().y + trackingBox.tl().y)/2), 3, cv::Scalar(255,255,255), -1);

    std::string text = "x and z " + std::to_string(point[0]) + " " + std::to_string(point[2]) + " angle " + std::to_string(angle) +
    " The camera is facing an object " + std::to_string(dist_to_center) + " meters away ";

    putText(frame, text, Point(30,30),
    FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(200,200,250), 1);
    cout << "point x z " <<  point[0] << " " << point[2] << " angle "<< angle <<endl;



    // Print the distance
    //std::cout << " Depth width of frame " << width << " Depth height of frame" << height << endl  ;
    //std::cout << "Color width of frame " << colored_frame.get_width() << " Color height of frame \n" << colored_frame.get_height() << endl;

    std::cout << "The camera is facing an object " << dist_to_center << " meters away \r" <<endl;


    cv::imshow("video feed", frame);
    waitKey(30);
    //cout << " cv imshow error "<<endl;
		// Display the frame
		//cv::imshow("Video feed", frame);

		// Write video frame to output
		//output.write(frame);

		// For breaking the loop

	} // end while (video.read(frame))

	//   // Release video capture and writer
	// output.release();
	// video.release();

	// Destroy all windows
	// cv::destroyAllWindows();

	return 0;
}
 int detectAndDraw(const HOGDescriptor &hog, Mat &img, Rect *main_rect)
{
    vector<Rect> found, found_filtered;
    double t = (double) getTickCount();
    // Run the detector with default parameters. to get a higher hit-rate
    // (and more false alarms, respectively), decrease the hitThreshold and
    // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
    hog.detectMultiScale(img, found, 0.1, Size(8,8), Size(32,32), 1.05, 2);
    t = (double) getTickCount() - t;
    cout << "detection time = " << (t*1000./cv::getTickFrequency()) << " ms " << endl;
    for(size_t i = 0; i < found.size(); i++ )
    {
        Rect r = found[i];
        size_t j;
        // Do not add small detections inside a bigger detection.
        for ( j = 0; j < found.size(); j++ )
            if ( j != i && (r & found[j]) == r )
                break;
        if ( j == found.size() )
            found_filtered.push_back(r);
    }

    if(!found_filtered.empty()){
        Rect r = found_filtered[0];
        // The HOG detector returns slightly larger rectangles than the real objects,
        // so we slightly shrink the rectangles to get a nicer output.
        r.x += cvRound(r.width*0.1);
        r.width = cvRound(r.width*0.8);
        r.y += cvRound(r.height*0.07);
        r.height = cvRound(r.height*0.8);
        rectangle(img, r.tl(), r.br(), cv::Scalar(0,255,0), 3);
        *main_rect = r;
        cout << " DETECTED PERSON = " << main_rect->tl() << " ms" << endl;
        return 1;
    }
    return 0;

}



int main(int argc, char** argv)
{

    HOGDescriptor hog;
    hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
    namedWindow("people detector", 1);
    int camera_id = 0;
    Rect main_rect;
    rs2_intrinsics depth_intr;
    // main_rect (0,0,500,500);


    rs2::pipeline p;
    rs2::config cfg;
  // Declare filters
    //rs2::decimation_filter dec_filter;
    //rs2::spatial_filter spat_filter;

    // Configure filter parameters
    //dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 3);
    //spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.55f);


      cfg.enable_stream(RS2_STREAM_DEPTH);
      cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

      auto profile = p.start(cfg);
      auto depth_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
      depth_intr = depth_stream.get_intrinsics();
      //p.start();

        VideoCapture vc;
        //Mat frame;


      //  vector<String>::const_iterator it_image = frames.begin();

        cv::Mat frame;

        rs2::align align_to_depth(RS2_STREAM_DEPTH);
        rs2::align align_to_color(RS2_STREAM_COLOR);
// ...

        for (;;)
        {


          //wait for frames and get frameset
          rs2::frameset frames = p.wait_for_frames();
          frames = align_to_depth.process(frames);
          auto colored_frame = frames.get_color_frame();

          //std::cout << " Depth width of frame " << width << " Depth height of frame" << height << endl  ;
          //std::cout << "Color width of frame " << colored_frame.get_width() << " Color height of frame \n" << colored_frame.get_height() << endl;


            const int w = colored_frame.as<rs2::video_frame>().get_width();
            const int h = colored_frame.as<rs2::video_frame>().get_height();


            Mat image(Size(w, h), CV_8UC3, (void*)colored_frame.get_data(), Mat::AUTO_STEP);
            frame = image;

            if(detectAndDraw(hog, image, &main_rect)){

              cout << "----------------"<< main_rect.tl()<<endl;
              break;
              //imshow("people detector", image);
          }
}

          MOSSE(frame,main_rect, p, depth_intr);






    return 0;
}
