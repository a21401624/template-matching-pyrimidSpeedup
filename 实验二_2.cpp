#include<opencv.hpp>
#include<stdio.h>

using namespace cv;
using namespace std;

double cal_NC(Mat im0, Mat T);
vector<Mat> buildpri(Mat im, int floor);
int main()
{
	clock_t startTime, endTime;
	startTime = clock();//计时开始
	
	int i, j,k;
	Mat im = imread("C:\\Users\\hp\\Desktop\\目标探测与识别\\实验\\实验二\\im0.bmp", 1);
	Mat im0 = imread("C:\\Users\\hp\\Desktop\\目标探测与识别\\实验\\实验二\\im0.bmp", 0);
	Mat T = imread("C:\\Users\\hp\\Desktop\\目标探测与识别\\实验\\实验二\\t3.bmp", 0);
	//Laplacian(T, T, CV_8UC1, 9);
	//Laplacian(im0, im0, CV_8UC1, 9);
	//imshow("im0", im0);
	waitKey(0);
	Mat ROI;
	Mat result(im0.rows - T.rows, im0.cols - T.cols, CV_64FC1, Scalar(0));

	vector<Mat> ori, temp;
	int floor = 3;
	ori = buildpri(im0,floor);
	temp = buildpri(T,floor);

	double max = 0;
	int maxi, maxj;
	for (k =floor-1; k >= 0; k--)
	{
		if (k == floor-1) 
		{
			for (i = 0; i < ori[k].rows - temp[k].rows; i++)
			{
				double *result_data = result.ptr<double>(i);
				for (j = 0; j < ori[k].cols - temp[k].cols; j++)
				{
					ROI = ori[k](Rect(j, i, temp[k].cols, temp[k].rows));
					result_data[j] = cal_NC(ROI, temp[k]);
					if (max < result_data[j])
					{
						max = result_data[j];
						maxi = i;
						maxj = j;
					}
				}
			}
		}
		else
		{
			max = 0;
			int refi = 2*maxi-2, refj = 2*maxj-2;//存放上一层最优点在这一层的5*5邻域左上角点在这一层图片里的坐标
			Mat roi = (ori[k])(Rect(refj, refi, 4 + temp[k].cols, 4 + temp[k].rows));//最优点5*5的邻域
			for (i = 0; i < roi.rows - temp[k].rows; i++)
			{
				double *result_data = result.ptr<double>(i);
				for (j = 0; j < roi.cols - temp[k].cols; j++)
				{
					ROI = roi(Rect(j, i, temp[k].cols, temp[k].rows));
					result_data[j] = cal_NC(ROI, temp[k]);
					if (max < result_data[j])
					{
						max = result_data[j];
						maxi = i+refi;
						maxj = j+refj;//maxi和maxj存放的是这25个点中最好的匹配位置在这一层图片里的坐标
					}
				}
			}
		}
	}
	rectangle(im, Rect(maxj, maxi, T.cols, T.rows), Scalar(0, 0, 255));
	endTime = clock();//计时结束
	printf("time:%lf\n", (double)(endTime - startTime) / CLOCKS_PER_SEC);
	printf("%d %d\n", maxj, maxi);
	imshow("im", im);
	waitKey(0);
	return 0;
}

vector<Mat> buildpri(Mat im, int floor)
{
	int i, j, k;
	vector<Mat> ori;
	for (k = 1; k <= floor; k++)
	{
		Mat temp(im.rows / (int)pow(2,k-1), im.cols / (int)pow(2, k - 1), CV_8UC1);
		if (k == 1)
		{
			temp = im;
			ori.push_back(temp);
		}
		else
		{
			for (i = 0; i < im.rows/ (int)pow(2, k - 1); i++)
			{
				uchar *temp_data = temp.ptr<uchar>(i);
				uchar *ori_data1 = ori.back().ptr<uchar>(2 * i);
				uchar *ori_data2 = ori.back().ptr<uchar>(2 * i+1);
				for (j = 0; j < im.cols/ (int)pow(2, k - 1); j++)
				{
					temp_data[j]= (ori_data1[2*j] + ori_data1[2 * j+1]+ ori_data2[2 * j] +ori_data1[2 * j+1] ) / 4;
				}
			}
			ori.push_back(temp);
		}
	}
	return ori;
}

double cal_NC(Mat im0, Mat T)
{
	int i, j;
	int width = T.cols;
	int height = T.rows;
	double aveT = 0, aveim = 0;
	uchar* Tdata, *im0_data;
	for (i = 0; i < height; i++)
	{
		Tdata = T.ptr<uchar>(i);
		im0_data = im0.ptr<uchar>(i);
		for (j = 0; j < width; j++)
		{
			aveim += im0_data[j];
			aveT += Tdata[j];
		}
	}
	aveim = aveim / (width*height);
	aveT = aveT / (width*height);
	double numerator = 0;
	double sumsqim = 0, sumsqT = 0;
	double nc;
	for (i = 0; i < height; i++)
	{
		Tdata = T.ptr<uchar>(i);
		im0_data = im0.ptr<uchar>(i);
		for (j = 0; j < width; j++)
		{
			numerator += (Tdata[j] - aveT)*(im0_data[j] - aveim);
			sumsqim += pow((im0_data[j] - aveim), 2);
			//sumsqT += pow((T.at<uchar>(i, j)-aveT),2);
		}
	}
	nc = numerator / (sqrt(sumsqim));
	return nc;
}
