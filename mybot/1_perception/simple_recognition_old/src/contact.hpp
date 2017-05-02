#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <time.h>

#define CNUM 10

using namespace cv;
using namespace std;

double GetDistanceTwoPoints(cv::Point p1, cv::Point p2){
	return sqrt(pow((float)p1.x - p2.x,2) + pow((float)p1.y - p2.y,2));
}

void GetMidpoint(cv::Point p1, cv::Point p2, cv::Point *p3){
	p3->x=(p1.x+p2.x)/2.0;
	p3->y=(p1.y+p2.y)/2.0;
}

double computeDistance(std::vector<cv::Point> contour){
	double distance;
	double minX = contour[0].x, maxX = contour[0].x;

	for ( int i=1; i<contour.size(); i++){
		if ( minX > contour[i].x )
			minX = contour[i].x;

		if ( maxX < contour[i].x )
			maxX = contour[i].x;
	}

	return maxX - minX;

}

int computeContours(cv::Mat& crop, std::vector<std::vector<cv::Point>>& contours, std::vector<cv::Vec4i> hierarchy){
	int largest_index = 0;
	findContours(crop, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0,0));  //CV_RETR_TREE

	double largest_area = 0;
	for ( int i=0; i<contours.size(); i++){

		double a = computeDistance(contours[i]);
		//double a = contourArea(contours[i], false);
		if ( a > largest_area ){
			largest_index = i;
			largest_area = a;
		}
	}

	return largest_index;
}


int modifyContours(std::vector<cv::Point> mergecon, std::vector<uchar> mergedepth, cv::Mat& d_crop){
	int size = 0;



	return size;

}

int mergeDepth(std::vector<cv::Point> mergecon, std::vector<uchar> mergedepth, cv::Mat& d_crop){
	int size = 0;

	for ( int i=0; i<mergecon.size(); i++){
		uchar d = d_crop.at<uchar>(mergecon[i].y,mergecon[i].x);

		mergedepth.push_back(d);

		size++;
	}

	return size;
}

int mergeContours(std::vector<std::vector<cv::Point>> contours, cv::Mat& d_crop, std::vector<cv::Point>& mergecon, std::vector<uchar>& mergedepth, int largest_index)
{
	int size=0;

	int d_aver=0;
	int t_hold = 0;

	for( int y = 0; y < d_crop.rows; y++ )
	{
		for( int x = 0; x < d_crop.cols; x++ )
		{

			//	for (int i=0; i<contours[largest_index].size(); i++){


			/*uchar d = d_crop.at<uchar>(y,x);
			if ( contours[largest_index][i].y == y && contours[largest_index][i].x == x){
				d_aver += d;
				exit(1);
			}*/
			//	}
		}

	}

	for (int i=0; i<contours[largest_index].size(); i++){
		uchar d = d_crop.at<uchar>(contours[largest_index][i].y,contours[largest_index][i].x);
		//		printf("\n%d %d\n", contours[largest_index][i].y,contours[largest_index][i].x); int a; scanf("%d", &a);
		//		exit(1);
		d_aver += d;
	}



	d_aver = d_aver/contours[largest_index].size();

	for ( int i=0; i<contours.size(); i++){
		if ( contours[i].size() < 100 ) continue;
		if ( contourArea(contours[i], true) < 1 ) continue;
		for ( int j=1; j<contours[i].size(); j++){
			uchar d = d_crop.at<uchar>(contours[i][j].y,contours[i][j].x);

			//			printf("\n%d %d\n", contours[i][j].y,contours[i][j].x); int a; scanf("%d", &a);
			//		exit(1);
			if ( d > d_aver - t_hold || d < d_aver + t_hold ){

				mergecon.push_back(contours[i][j]);

				size++;
			}
		}
	}

	mergeDepth(mergecon, mergedepth, d_crop);

	//	modifyContours(mergecon, mergedepth, d_crop);

	return size;
}


bool isInside(Point testP, vector<Point> mergecon){

	for (int i=0; i < mergecon.size(); i++){

		//	if ( testP.x >

	}

	return true;

}

int FindTopology(vector<Point> mergecon, vector<uchar> mergedepth, vector<Point> inside, vector<uchar> inside_depth, Mat& rgb, Mat& d_crop){
	int size = 0;

	int d_aver=0;
	int t_hold = 10;

	for (int i=0; i<mergecon.size(); i++){
		uchar d = d_crop.at<uchar>(mergecon[i].y,mergecon[i].x);

		d_aver += (int)d;
	}
	d_aver = d_aver/mergecon.size();

	for( int y = 0; y < rgb.rows; y++ )
	{
		for( int x = 0; x < rgb.cols; x++ )
		{
			Point testP(x, y);
			if ( isInside(testP, mergecon) ){
				return size;
			}

		}
	}
	return size;
}

int decisionType(vector<Point> mergecon, vector<uchar> mergedepth, vector<Point> inside, vector<uchar> inside_depth)
{
	int type;

	if ( inside.empty() )
		return 0;
	else
		return 1;


	return type;
}

vector<Point> computeBottomline(vector<Point> contours, vector<Point> d_contours, vector<Point>& new_contours){
	//	vector<Point> new_contours;
	int new_index = 0;

	for (int i=0; i<contours.size(); i++ ){
		Point i_point = contours[i];
		int j;
		for ( j=0; j<d_contours.size(); j++){
			Point j_point = d_contours[j];

			if ( (i_point.x - j_point.x < 20 && i_point.x - j_point.x > -20) && (i_point.y - j_point.y < 20 && i_point.y - j_point.y > -20 ) )
				break;
		}

		if ( j == d_contours.size() ){
			new_contours.push_back(i_point);
			//	new_contours[new_index] = i_point;
			//	new_index++;
		}
	}

	/*	for (int i=0; i<contours.size(); i++ ){
		printf("11 x: %d\t y: %d\n", contours[i].x, contours[i].y);
	}
	for (int i=0; i<d_contours.size(); i++ ){
		printf("22 x: %d\t y: %d\n", d_contours[i].x, d_contours[i].y);
	}*/
	//exit(1);
	return new_contours;
}

std::vector<cv::RotatedRect> locationSearch(cv::Mat& rgb, const cv::Mat& gray, cv::Point2d topleft, cv::Point2d bottomright)
								{
	Mat original;
	rgb.copyTo(original);

	Point2d optimal_location;

	vector<vector<Point>> contours, d_contours;
	vector<Vec4i> hierarchy, d_hierarchy;
	int largest_index = 0;
	int d_largest_index = 0;
	int topleft_x = topleft.x;
	int topleft_y = topleft.y;
	int width = bottomright.x - topleft.x;
	int height = bottomright.y - topleft.y;
	RNG rng(12345);
	cv::Scalar color = Scalar(0, 255, 255);

	/*Mat result;
	Mat bgModel, fgModel;
	Rect rectangle(topleft.x ,topleft.y , bottomright.x, bottomright.y );
	grabCut(rgb, result, rectangle,bgModel, fgModel, 1, GC_INIT_WITH_RECT);

	compare(result, GC_PR_FGD, result, CMP_EQ);
	Mat foreground(rgb.size(), CV_8UC3, Scalar(255,255,255));
	rgb.copyTo(foreground, result);
	 */

	Mat mask, temp;
	cvtColor(rgb(Rect(topleft.x,topleft.y,width,height)), mask, COLOR_BGR2GRAY);

	dilate(mask, temp, Mat(), Point(-1,-1), 3);
	erode(temp, temp, Mat(), Point(-1,-1), 6);
	dilate(temp, temp, Mat(), Point(-1,-1), 3);
	threshold(temp, temp, 150, 256, CV_THRESH_BINARY);

	vector<vector<Point>> test1;
	vector<Vec4i> test2;
	findContours(temp, test1, test2, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	mask = Scalar(0,0,0);
	for (int i=0; i<test1.size(); i++)
	{
		Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
		drawContours(mask, test1, i, color, 2, 8, test2);
	}

  std::cout << "gclab" << std::endl; 

	Mat edge, threshold_out;
	threshold(rgb, threshold_out, 200, 255, THRESH_BINARY);
	Canny(rgb, edge, 20, 20*3);
	Mat crop = edge(Rect(topleft.x,topleft.y,width,height));
	Mat d_edge;
	//Canny(gray, d_edge, 20, 20*3);
	Mat d_crop = gray(Rect(topleft.x,topleft.y,width,height));
	Mat test = rgb(Rect(topleft.x,topleft.y,width,height));
  std::cout << "gclab2" << std::endl; 
	//crop.copyTo(box);

	//return optimal_location;

	/*findContours(d_crop, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));


	 double largest_area = 0;
	  for ( int i=0; i<contours.size(); i++){

		  double a = contourArea(contours[i], false);
		  if ( a > largest_area ){
			  largest_index = i;
			  largest_area = a;
		  }
	  }
	  drawContours(d_crop, contours, largest_index, color, 2, 8, hierarchy);
	 double largest_area = 0;
	  for ( int i=0; i<contours.size(); i++){
		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( d_crop, contours, i, color, 2, 8, hierarchy, 0, Point() );

	  }
	  d_crop.copyTo(box);*/

	largest_index = computeContours(crop, contours, hierarchy);
	d_largest_index = computeContours(d_crop, d_contours, d_hierarchy);

	Rect bounding_rect;
  std::cout << contours.size() << std::endl; 
  std::cout << contours[largest_index] << std::endl; 
	bounding_rect = boundingRect(contours[largest_index]);

	Point middle((bounding_rect.tl().x + bounding_rect.br().x) / 2, (bounding_rect.tl().y + bounding_rect.br().y) / 2);

	Point leftgrab(bounding_rect.tl());
	Point rightgrab(bounding_rect.br());
  
	for (int i = 0; i < contours[largest_index].size(); i++)
	{
		if (contours[largest_index][i].x < middle.x)
		{
			if (abs(middle.y - leftgrab.y) > abs(contours[largest_index][i].y - middle.y))
				leftgrab = contours[largest_index][i];
		}
		else
		{
			if (abs(middle.y - rightgrab.y) > abs(contours[largest_index][i].y - middle.y))
				rightgrab = contours[largest_index][i];
		}
	}


	double t_dist = GetDistanceTwoPoints(middle, contours[largest_index][0]);
	Point t_point = contours[largest_index][0];
	for (int j=1; j<contours[largest_index].size(); j++){
		double cur_dist = 0;
		cur_dist = GetDistanceTwoPoints(middle, contours[largest_index][j]);
		if ( t_dist > cur_dist ){
			t_dist = cur_dist;
			t_point = contours[largest_index][j];
		}
	}

	t_dist = GetDistanceTwoPoints(t_point, contours[largest_index][0]);
	Point t_point2 = contours[largest_index][0];
	for (int j=1; j<contours[largest_index].size(); j++){
		double cur_dist = GetDistanceTwoPoints(t_point, contours[largest_index][j]);
		double y_test = double(contours[largest_index][j].y-t_point.y)/double(contours[largest_index][j].x-t_point.x)*(double)(middle.x-t_point.x)+t_point.y;

		if ( (int)y_test >= middle.y - 1 &&  (int)y_test <= middle.y + 1 ){
			if ( cur_dist < t_dist ){
				t_dist = cur_dist;
				t_point2 = contours[largest_index][j];
			}
		}
	}

	//rightgrab = t_point2;
	//leftgrab = t_point;

	leftgrab.x += topleft.x;
	leftgrab.y += topleft.y;
	rightgrab.x += topleft.x;
	rightgrab.y += topleft.y;
  
	vector<vector<Point>> contours_bottom;
	vector<Point> bottom;
	computeBottomline(contours[largest_index], d_contours[d_largest_index], bottom);
	contours_bottom.push_back(bottom); //largest_index = 0;

	/*	 for (int i=0; i<contours[largest_index].size(); i++){  //r
		  Point temp = contours[largest_index][i];
		  temp.x += topleft.x;
		  temp.y += topleft.y;

		  contours[largest_index][i] = temp;
	  }*/
	color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
	//	drawContours(rgb, contours, largest_index, color, 2, 8, hierarchy);

	color = Scalar(0, 0, 255);
	//	circle(rgb, leftgrab, 2, color, 2);
	//	circle(rgb, leftgrab, 8, color, 2);

	//	circle(rgb, rightgrab, 2, color, 2);

	vector<Point> mergecon, inside;
	vector<uchar> mergedepth, inside_depth;
	int size_merge = mergeContours(contours, d_crop, mergecon, mergedepth, largest_index);
	//	int size_inside = FindTopology(mergecon, mergedepth, inside, inside_depth, crop, d_crop);

	//	int o_type = decisionType(mergecon, mergedepth, inside, inside_depth);

	vector<Point> approx;
	//approxPolyDP(mergecon, approx, arcLength(mergecon, true)*0.1, false);

	vector<Point> hull;
	convexHull(contours[largest_index], hull, true);
	vector<vector<Point>> merge_contours;

	merge_contours.push_back(hull);
	for (int i=0; i<merge_contours[0].size(); i++){  //r
		Point temp = merge_contours[0][i];
		temp.x += topleft.x;
		temp.y += topleft.y;

		merge_contours[0][i] = temp;
	}
	//drawContours(rgb, contours, largest_index, color, 2, 8, hierarchy);
	//drawContours(rgb, merge_contours, 0, color, 2, 8, hierarchy);

	for (int i=0; i<contours[largest_index].size(); i++){  //r
		Point temp = contours[largest_index][i];
		temp.x += topleft.x;
		temp.y += topleft.y;

		contours[largest_index][i] = temp;
	}

	Point highest_y = contours[largest_index][0];
	for (int j=1; j<contours[largest_index].size(); j++){
		if ( highest_y.y < contours[largest_index][j].y )
			highest_y = contours[largest_index][j];
	}
	double l_dist = GetDistanceTwoPoints(leftgrab, highest_y);
	double r_dist = GetDistanceTwoPoints(rightgrab, highest_y);
	int diff_r=0, diff_l=0;
	if ( l_dist > r_dist ){
		diff_l = l_dist -r_dist;
	}
	else
		diff_r = r_dist - l_dist;

	//circle(rgb, leftgrab, 2, color, 2);
	double sht_dist = GetDistanceTwoPoints(leftgrab, contours[largest_index][0]);
	Point sht_point = contours[largest_index][0];
	for (int j=1; j<contours[largest_index].size(); j++){
		double cur_dist = 0;
		cur_dist = GetDistanceTwoPoints(leftgrab, contours[largest_index][j]);
		if ( sht_dist > cur_dist ){
			sht_dist = cur_dist;
			sht_point = contours[largest_index][j];
		}
	}

	int modify;
	if ( leftgrab.y > sht_point.y )
		modify = leftgrab.y - sht_point.y;
	else
		modify = sht_point.y - leftgrab.y;

	vector<Point> left_set;
	vector<Point> right_set;

	for (int i=0; i < CNUM; i++){

		int modx = (rand()%10)+1;
		int mody = (rand()%10)+1;

		Point new_grab;

		int expression = (rand()%2)+1;

		if ( expression == 1 ){
			new_grab.x = leftgrab.x + modx;
			new_grab.y = leftgrab.y + mody;
		}
		else
		{
			new_grab.x = leftgrab.x - modx;
			new_grab.y = leftgrab.y - mody;
		}

		//if ( sht_point.y > new_grab.y )
		//	new_grab.y += diff_l;
		//else
		left_set.push_back(new_grab);

		//circle(rgb, new_grab, 2, color, 2);
	}
	RotatedRect r;
	//circle(rgb, t_point, 2, color, 8);
	//circle(rgb, t_point2, 2, color, 8);

	sht_dist = GetDistanceTwoPoints(rightgrab, contours[largest_index][0]);
	sht_point = contours[largest_index][0];
	for (int j=1; j<contours[largest_index].size(); j++){
		double cur_dist = 0;
		cur_dist = GetDistanceTwoPoints(rightgrab, contours[largest_index][j]);
		if ( sht_dist > cur_dist ){
			sht_dist = cur_dist;
			sht_point = contours[largest_index][j];
		}
	}
	if ( rightgrab.y > sht_point.y )
		modify = rightgrab.y - sht_point.y;
	else
		modify = sht_point.y - rightgrab.y;
	for (int i=0; i < CNUM; i++){

		int modx = (rand()%10)+1;
		int mody = (rand()%10)+1;

		Point new_grab;

		int expression = (rand()%2)+1;

		if ( expression == 1 ){
			new_grab.x = rightgrab.x + modx;
			new_grab.y = rightgrab.y + mody;
		}
		else
		{
			new_grab.x = rightgrab.x - modx;
			new_grab.y = rightgrab.y - mody;
		}
		right_set.push_back(new_grab);
	}

	vector<RotatedRect> output;

	for ( int i=0; i < 7; i++){
		int first = (rand()%10);
		int second = (rand()%10);

		Point point1(left_set[first].x, left_set[first].y);
		Point point2(right_set[second].x, right_set[second].y);

		Point p3;
		int angle = (rand()%10);
		int row = (rand()%5), col = (rand()%3);
		GetMidpoint(point1, point2, &p3);
		RotatedRect rRect = RotatedRect(p3, Size2f(row*10+40,30), angle*10);

		Point2f vertices[4];
		rRect.points(vertices);

		output.push_back(rRect);

	}

	return output;
}

void DrawRotatedRects(Mat& rgb, vector<RotatedRect> rRects){
	cv::Scalar color = cv::Scalar(0, 255, 255);

	for ( int i=0; i < rRects.size(); i++){
		cv::Point2f vertices[4];
		rRects[i].points(vertices);

		for (int i=0; i<4; i++)
			cv::line(rgb, vertices[i], vertices[(i+1)%4], color);
	}

}
