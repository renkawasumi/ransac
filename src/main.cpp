#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <sys/time.h>
static struct timeval microsSource;
static int64_t offsetSeconds;
static int64_t offsetMicros;

void timer_setup(){
  gettimeofday(&microsSource, NULL);
  offsetSeconds=microsSource.tv_sec;
  offsetMicros=microsSource.tv_usec;
  return;
}

int64_t millis(){
  gettimeofday(&microsSource, NULL);
  return (microsSource.tv_sec-offsetSeconds)*(int64_t)1000+(microsSource.tv_usec-offsetMicros);
}

using namespace cv;

#define TRUE_SLOPE 2.f
#define TRUE_INTERCEPT 300.f
#define PC_NUM 300 // point_cloud_num
#define ITERATE 100
#define DIST 1.f //[mm]
#define NOISE 20.f //[mm]
#define RADIUS 1.f
#define RED Scalar(0, 0, 255)
#define BLUE Scalar(255, 0, 0)
#define GREEN Scalar(0, 255, 0)
#define IMG_X 1000
#define IMG_Y 1000

typedef struct{
    double x;
    double y;
    double distance;
}point_t;

class Ransac{
  private:
    point_t point_cloud[PC_NUM] = {}; 
  public:
    Ransac();
    void calc();
    int getRandom(int min, int max);
    double getNoise(double val);
    void draw();
    int opt_point_num;
    double opt_slope, opt_intercept; 
    Mat img = Mat::zeros(IMG_Y, IMG_X, CV_8UC3);
};

Ransac::Ransac(){
  srand((unsigned int)time(NULL));
}

void Ransac::calc(){
  double ave_x, ave_y;
  double sum_x, sum_y;
  double var_x, var_y;
  double cov;
  double slope, intercept;
  int rand_1, rand_2;
  int point_num = 0;
  int max_point_num = 0;
  for(int i = 0; i < ITERATE; i++){
    rand_1 = getRandom(0, PC_NUM); 
    rand_2 = getRandom(0, PC_NUM);
    if(rand_1 == rand_2) continue;
    //printf("rand_1 %f, %f\n", point_cloud[rand_1].x, point_cloud[rand_1].y);
    //printf("rand_2 %f, %f\n", point_cloud[rand_2].x, point_cloud[rand_2].y);
    
    ave_x = (point_cloud[rand_1].x + point_cloud[rand_2].x) / 2.f; 
    ave_y = (point_cloud[rand_1].y + point_cloud[rand_2].y) / 2.f;
    sum_x = powf(point_cloud[rand_1].x - ave_x, 2) + powf(point_cloud[rand_2].x - ave_x, 2);
    sum_y = powf(point_cloud[rand_1].y - ave_y, 2) + powf(point_cloud[rand_2].y - ave_y, 2);
    var_x = sum_x / 2.f;
    var_y = sum_y / 2.f;
    cov = ((point_cloud[rand_1].x - ave_x) * (point_cloud[rand_1].y - ave_y) + (point_cloud[rand_2].x - ave_x) * (point_cloud[rand_2].y - ave_y)) / 2.f;
    slope = cov / var_y;
    intercept = ave_x - slope * ave_y;
    //printf("slope:%f, intercept:%f\n", slope, intercept); 

    for(int j = 0; j < PC_NUM; j++){
      point_cloud[j].distance = fabs(slope * point_cloud[j].y - point_cloud[j].x + intercept) / sqrtf(powf(slope, 2) + powf(1.f, 2));
      if(point_cloud[j].distance <= DIST){
        point_num += 1; 
      }
    }
    //printf("point_num:%d\n", point_num);

    if(point_num > max_point_num){
      opt_slope = slope;
      opt_intercept = intercept;
      max_point_num = point_num; 
      opt_point_num = point_num;
    }
    point_num = 0;
  } 
}

int Ransac::getRandom(int min,int max){
  return min + (int)(rand()*(max-min+1.0)/(1.0+RAND_MAX));
}

void Ransac::draw(){
  Point p;
  for(int r = 0; r < PC_NUM; r++){
    point_cloud[r].y = -r + getNoise(NOISE);
    point_cloud[r].x = TRUE_SLOPE * point_cloud[r].y + TRUE_INTERCEPT + getNoise(NOISE);
    p.y = point_cloud[r].x + IMG_Y;
    p.x = point_cloud[r].y + IMG_X; 
    //printf("%f %f\n", point_cloud[r].x, point_cloud[r].y);
    circle(img, p, RADIUS, RED, -1, 1);
  }
  Point p1;
  Point p2;
  p1.x = IMG_X / 2;
  p1.y = 0;
  p2.x = IMG_X / 2;
  p2.y = IMG_Y;
  line(img, p1, p2, BLUE, 3, LINE_8);
  p1.x = 0;
  p1.y = IMG_Y / 2;
  p2.x = IMG_X;
  p2.y = IMG_Y / 2;
  line(img, p1, p2, GREEN, 3, LINE_8);
}

double Ransac::getNoise(double val){ 
  return val * (2.f * (double)rand() / RAND_MAX - 1.f); 
}

int main(int argc, char **argv){
    ros::init(argc, argv, "Ransac");
    ros::NodeHandle nh;
    timer_setup();
    int64_t start, end; 
    Ransac ransac;
    ransac.draw();
    start = millis();
    ransac.calc();
    end = millis();
    printf("%d[ms]\n", end - start);
    printf("slope:%f intercept:%f\n", ransac.opt_slope, ransac.opt_intercept);
    //imshow("output", ransac.img);
    //cvWaitKey(0);

}
