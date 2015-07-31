#pragma once
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;


namespace look3d {
// Implementation from paper "Local Adaptive Thresholding Techniques Using Integral Images"
// @param  gray_img[in] input image
// @param  binarized_img[out] binarised image
// @param  rect_size[in] in which window size used for calculating local mean & variance
// @param  k[in] regularisation parameter [0.2,0.5], higher k -> lower local threshold
// @param  intensity_deviation[in] maximun of standard deviation (in grayscale R=256/2=128)
inline void binarise_adaptivethreshold(
    cv::Mat& gray_img, cv::Mat& binarized_img,
    int rect_size = 30, float k = 0.25f, int intensity_deviation = 128) {

  if (binarized_img.empty())
    binarized_img.create(gray_img.size(), CV_8UC1);
  int half_rect_size = rect_size/2;
  int sq_rect_size = rect_size*rect_size;
  cv::Mat integral_img, sq_integral_img;
  cv::integral(gray_img, integral_img, sq_integral_img);

  float local_mean=0.0f;
  float local_sq_mean=0.0f;
  float local_sq_variance = 0.0f;
  float local_threshold=0.0f;
  int left, right, top, bottom;
  for(int y=0; y<gray_img.rows; ++y){
    for(int x=0; x<gray_img.cols; ++x){
      cv::Point p(x,y);
      left = (x-half_rect_size>=0)? x-half_rect_size: 0;
      right = (x+half_rect_size)<gray_img.cols ? x+half_rect_size:gray_img.cols-1;
      top = (y-half_rect_size>=0) ? y-half_rect_size:0;
      bottom = (y+half_rect_size<gray_img.rows)? y+half_rect_size:gray_img.rows-1;
      cv::Point p_rb(right,bottom), p_lt(left,top), p_rt(right,top), p_lb(left,bottom);
      local_mean = ( (integral_img.at<float>(p_rb) + integral_img.at<float>(p_lt)) -
                     (integral_img.at<float>(p_rt) +integral_img.at<float>(p_lb)) )/(sq_rect_size);
      local_sq_mean = local_mean*local_mean;
      local_sq_variance = ( (sq_integral_img.at<float>(p_rb) + sq_integral_img.at<float>(p_lt))-
                            (sq_integral_img.at<float>(p_rt)+sq_integral_img.at<float>(p_lb)) )/sq_rect_size -local_sq_mean;
      local_threshold =  local_mean*(1+k*(sqrt(local_sq_variance)/intensity_deviation-1));
      if(gray_img.at<unsigned char>(p) <= local_threshold){
        binarized_img.at<unsigned char>(p) = 0xFF;
      }else{
        binarized_img.at<unsigned char>(p) = 0;
      }
    }
  }
}

// Implementation from paper "Local Adaptive Thresholding Techniques Using Integral Images"
// @param  gray_img[in] input image
// @param  binarized_img[out] binarised image
// @param  rect_size[in] in which window size used for calculating local mean & variance
// @param  sparse_step[in] sparsely step in pixels
// @param  k[in] regularisation parameter [0.2,0.5], higher k -> lower local threshold
// @param  intensity_deviation[in] maximun of standard deviation (in grayscale R=256/2=128)
inline void binarise_adaptivethreshold_sparse(
    cv::Mat& gray_img, cv::Mat& binarized_img,
    int rect_size = 30, int sparse_step = 10, float k = 0.25f,
    int intensity_deviation = 128) {
  if (binarized_img.empty())
    binarized_img.create(gray_img.size(), CV_8UC1);
  int half_rect_size = rect_size/2;
  int sq_rect_size = rect_size*rect_size;
  cv::Mat integral_img, sq_integral_img;
  cv::integral(gray_img, integral_img, sq_integral_img);

  float local_mean=0.0f;
  float local_sq_mean=0.0f;
  float local_sq_variance = 0.0f;
  float local_threshold=0.0f;
  int left, right, top, bottom;
  for(int y=0; y<gray_img.rows; y+=sparse_step){
    for(int x=0; x<gray_img.cols; x+=sparse_step){
      cv::Point p(x,y);
      left = (x-half_rect_size>=0)? x-half_rect_size: 0;
      right = (x+half_rect_size)<gray_img.cols ? x+half_rect_size:gray_img.cols-1;
      top = (y-half_rect_size>=0) ? y-half_rect_size:0;
      bottom = (y+half_rect_size<gray_img.rows)? y+half_rect_size:gray_img.rows-1;
      cv::Point p_rb(right,bottom), p_lt(left,top), p_rt(right,top), p_lb(left,bottom);
      local_mean = ( (integral_img.at<float>(p_rb) + integral_img.at<float>(p_lt)) -
                     (integral_img.at<float>(p_rt) +integral_img.at<float>(p_lb)) )/(sq_rect_size);
      local_sq_mean = local_mean*local_mean;
      local_sq_variance = ( (sq_integral_img.at<float>(p_rb) + sq_integral_img.at<float>(p_lt))-
                            (sq_integral_img.at<float>(p_rt) + sq_integral_img.at<float>(p_lb)) )/sq_rect_size -local_sq_mean;
      local_threshold =  local_mean*(1+k*(sqrt(local_sq_variance)/intensity_deviation-1));
      if(gray_img.at<unsigned char>(p) <= local_threshold){
        binarized_img.at<unsigned char>(p) = 255;
      }else{
        binarized_img.at<unsigned char>(p) = 0;
      }
      for(int iy=top; iy<bottom; iy++)
        for(int ix=left; ix<right; ix++)
        {
          cv::Point p(ix,iy);
          if(gray_img.at<unsigned char>(p) <= local_threshold){
            binarized_img.at<unsigned char>(p) = 255;
          }else{
            binarized_img.at<unsigned char>(p) = 0;
          }
        }
    }
  }
}

// finds connected components given binary image
// adapt code from http://nghiaho.com
// @param binary_img[in] 0 or 1 image
inline void find_blobs(const cv::Mat &binary_img,
                       std::vector < std::vector<cv::Point> > &blobs)  {
  blobs.clear();

  // Fill the label_image with the blobs
  // 0  - background
  // 1  - unlabelled foreground
  // 2+ - labelled foreground

  cv::Mat label_image;
  binary_img.convertTo(label_image, CV_32FC1); // weird it doesn't support CV_32S!

  int label_count = 2; // starts at 2 because 0,1 are used already

  for (int y = 0; y < binary_img.rows; y++) {
    for (int x = 0; x < binary_img.cols; x++) {
      if ((int)label_image.at<float>(y,x) == 0)
        continue;
      cv::Rect rect;
      cv::floodFill(label_image, cv::Point(x,y),
                    cv::Scalar(label_count), &rect,
                    cv::Scalar(0), cv::Scalar(0), 4);

      std::vector <cv::Point2i> blob;

      for (int i = rect.y; i < (rect.y+rect.height); i++) {
        for (int j = rect.x; j < (rect.x+rect.width); j++) {
          if ((int)label_image.at<float>(i,j) != label_count)
            continue;

          blob.push_back(cv::Point(j,i));
        }
      }

      blobs.push_back(blob);

      label_count++;
    }
  }
}

// detect laser point (assuming to be the brightest & closest blob to center in roi)
inline bool detect_laser_dot_click(const cv::Mat& gray_img, const cv::Rect& roi,
                             vector<int> &laser_location,
                             int laser_brightness_threshold =140) {

  laser_location.resize(2);
  cv::Mat m_roi(gray_img, roi);
  cv::Mat binary_img;
  cv::threshold(m_roi, binary_img, laser_brightness_threshold, 255, cv::THRESH_BINARY);

  std::vector<std::vector<cv::Point> > blobs;
  
  find_blobs(binary_img, blobs);
  if(!binary_img.empty())
    {
      imshow("uknow",binary_img);
      // std::cout << " binary_img " << binary_img << std::endl;
    }
  if(blobs.empty())
    return false;


  std::vector<cv::Point> centers(blobs.size());
  // get center location from blob
  for (size_t i = 0; i < blobs.size(); ++i) {
    double sum_x = 0.0, sum_y = 0.0;
    for (size_t j = 0; j < blobs[i].size(); ++j) {
      sum_x += blobs[i][j].x;
      sum_y += blobs[i][j].y;

      // std::cout << " blobs " << blobs[i][j].x << " , " << blobs[i][j].y << std::endl;

    }
    centers[i] = cv::Point(cvRound(sum_x/blobs[i].size()), cvRound(sum_y/blobs[i].size()));
  }
  // laser dot should have size >=25pixels & closest to the center
  double min_dist2center = 1000.0;
  size_t min_id = 0;
  for (size_t i = 0; i < blobs.size(); ++i) {
    double dist2center = cv::norm(centers[i]);
    if (min_dist2center > dist2center && blobs[i].size() > 0) {
      min_dist2center = dist2center;
      min_id=i;
    }
  }
  laser_location[0] = roi.x + centers[min_id].x;
  laser_location[1] = roi.y + centers[min_id].y;
  return true;
}



// detect laser point (assuming to be the brightest & closest blob to center in roi)
inline bool detect_laser_dot(const cv::Mat& gray_img, const cv::Rect& roi,
                             Eigen::Vector2d& laser_location,
                             int laser_brightness_threshold =150) {

  cv::Mat m_roi(gray_img, roi);
  cv::Mat binary_img;
  cv::threshold(m_roi, binary_img, laser_brightness_threshold, 255, cv::THRESH_BINARY);

  std::vector<std::vector<cv::Point> > blobs;
  
  find_blobs(binary_img, blobs);
  if(!binary_img.empty())
    {
      imshow("uknow",binary_img);
      // std::cout << " binary_img " << binary_img << std::endl;
    }
  if(blobs.empty())
    return false;


  std::vector<cv::Point> centers(blobs.size());
  // get center location from blob
  for (size_t i = 0; i < blobs.size(); ++i) {
    double sum_x = 0.0, sum_y = 0.0;
    for (size_t j = 0; j < blobs[i].size(); ++j) {
      sum_x += blobs[i][j].x;
      sum_y += blobs[i][j].y;

      // std::cout << " blobs " << blobs[i][j].x << " , " << blobs[i][j].y << std::endl;

    }
    centers[i] = cv::Point(cvRound(sum_x/blobs[i].size()), cvRound(sum_y/blobs[i].size()));
  }
  // laser dot should have size >=25pixels & closest to the center
  double min_dist2center = 1000.0;
  size_t min_id = 0;
  for (size_t i = 0; i < blobs.size(); ++i) {
    double dist2center = cv::norm(centers[i]);
    if (min_dist2center > dist2center && blobs[i].size() > 25) {
      min_dist2center = dist2center;
      min_id=i;
    }
  }
  laser_location[0] = roi.x + centers[min_id].x;
  laser_location[1] = roi.y + centers[min_id].y;
  return true;
}

typedef Eigen::Matrix< std::complex<double> , 8 , 2 > Matrix82;


// detect laser point (assuming to be the brightest & closest blob to center in roi)
inline bool detect_laser_4dots(const cv::Mat& gray_img, const cv::Rect& roi,
                             vector<vector<int> > &abcd,
                             int laser_brightness_threshold =140) {

  vector<int> laser_point;
  vector<vector<int> > laser_locations;
  // vector<vector<int> > abcd;

  abcd.resize(4);
  laser_point.resize(2);
  

  cv::Mat m_roi(gray_img, roi);
  cv::Mat binary_img;
  cv::threshold(m_roi, binary_img, laser_brightness_threshold, 255, cv::THRESH_BINARY);

  std::vector<std::vector<cv::Point> > blobs;
  
  find_blobs(binary_img, blobs);
  if(!binary_img.empty())
    {
      imshow("uknow",binary_img);
      // std::cout << " binary_img " << binary_img << std::endl;
    }
  if(blobs.empty())
    return false;


  std::vector<cv::Point> centers(blobs.size());
  // get center location from blob
  for (size_t i = 0; i < blobs.size(); ++i) {
    double sum_x = 0.0, sum_y = 0.0;
    for (size_t j = 0; j < blobs[i].size(); ++j) {
      sum_x += blobs[i][j].x;
      sum_y += blobs[i][j].y;

      // std::cout << " blobs " << blobs[i][j].x << " , " << blobs[i][j].y << std::endl;

    }
    centers[i] = cv::Point(cvRound(sum_x/blobs[i].size()), cvRound(sum_y/blobs[i].size()));
  }
  // laser dot should have size >=25pixels & closest to the center
  double min_dist2center = 1000.0;
  double max_dist2center = 100.0;

  // size_t min_id = 0;
  int index = 0;
  double prev_dist = 0;
  std::vector<int> stored_positions(4);

  for (size_t i = 0; i < blobs.size(); ++i) {
    double dist2center = cv::norm(centers[i]);
    // cout << " blobs[i].size() " << blobs[i].size() << endl;

    if (min_dist2center > dist2center && blobs[i].size() > 0) {
      // min_dist2center = dist2center;
      // min_id=i;
      
      // from the closest point to the 4th closest
      if(index < 5)
        {
          if(index == 0)          
            {
              stored_positions[index] = i;
              prev_dist = dist2center;
              // cout << " found one point with a dist "<< dist2center << " i "<< i << endl;
              index++;
            }else if(abs(dist2center - prev_dist) > 10 )
                {

                stored_positions[index] = i;
                prev_dist = dist2center;
                // cout << " found one point with a dist "<< dist2center << " i "<< i << endl;
                index++;
                  }
        }
    }
  }

  
  for(size_t i = 0; i < stored_positions.size(); i++){
  
  // cout << " stored_positions.size()" << stored_positions.size() << " stored_positions[i]" << stored_positions[i] <<  endl;
 
  // size_t min_id;

  // min_id = stored_positions[i];

  laser_point[0] = roi.x + centers[stored_positions[i]].x; 
  laser_point[1] = roi.y + centers[stored_positions[i]].y;
  
  laser_locations.push_back(laser_point);

  }


  // cout << " laser_locations.size " <<  laser_locations.size() << endl;

  // cout << " laser_locations " << laser_locations[0][0] << " " << laser_locations[0][1] << endl;
  // cout << " laser_locations " << laser_locations[1][0] << " " << laser_locations[1][1] << endl;
  // cout << " laser_locations " << laser_locations[2][0] << " " << laser_locations[2][1] << endl;
  // cout << " laser_locations " << laser_locations[3][0] << " " << laser_locations[3][1] << endl;

  // Now let s order the vector in a clockwise form
  // A is the first point D is the last

  bool Afound = false;
  bool Cfound = false;
  std::vector<int> A(2);
  std::vector<int> B(2);
  std::vector<int> C(2);
  std::vector<int> D(2);

  // init phase
  int guess = 0 ;
  int yeah = 0 ;
  int noo = 0;
  int thpx = 30; //if the rect is smaller do it smaller :) 
  int indexa = 0;
  int indexb = 0;
  int tries = 0 ;
  A[0] = laser_locations[guess][0];
  A[1] = laser_locations[guess][1];

  while(!(Afound && Cfound) && tries < 1000 ){
  for(int i=0; i < laser_locations.size(); i++){

    // it means that this is the lowest value for x and y
    if(i != guess && guess < laser_locations.size()){
      if((laser_locations[guess][0] - laser_locations[i][0]) < thpx && 
          (laser_locations[guess][1] - laser_locations[i][1]) < thpx ){

         if(!Afound)
            { yeah++;
                   if(yeah == 3){
                          
                       A[0] = laser_locations[guess][0]; 
                       A[1] = laser_locations[guess][1];
                       
                       indexa = guess;
                       Afound = true;
                       }
                     }
      }
        else // /*why loosing time checking useless stuff */ checking D
        {
          // guess++;
          // yeah = 0 ;      

        if(!Cfound){ 
          noo++;
          if(noo == 3)
          {

            C[0] = laser_locations[guess][0]; 
            C[1] = laser_locations[guess][1];

            indexb = guess;
            Cfound = true;
          }
          }

        }
        } // if guess!=i
    }
    guess++;
    yeah = 0 ;
    noo = 0 ;
    tries++;
  } 

  //looking for B and D
  for(int i=0; i < laser_locations.size(); i++)
  {

    if((i != indexa) && (i != indexb) ){
          if(abs(laser_locations[i][0] - A[0]) < 50)
            {
            //it s D
            D[0] = laser_locations[i][0]; 
            D[1] = laser_locations[i][1];

            } else
            {
     
            //this is B
            B[0] = laser_locations[i][0]; 
            B[1] = laser_locations[i][1];
        
            } 
          }

    

  }

  cout << " A is " << A[0] << " , " << A[1] << " Blue " << endl;
  cout << " B is " << B[0] << " , " << B[1] << " Green " << endl;
  cout << " C is " << C[0] << " , " << C[1] << " Red " << endl;
  cout << " D is " << D[0] << " , " << D[1] << " Cyan " << endl;


  abcd[0] = A;
  abcd[1] = B;
  abcd[2] = C;
  abcd[3] = D;
  laser_locations.resize(4);
  return true;
}

}  // namespace look3d