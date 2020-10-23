// Sonar_Object.cpp : Defines the entry point for the console application.
//
//#pragma comment(lib, "ws2_32.lib")

#include "stdafx.h"
#include <stdio.h>

#include <iostream>
#include <fstream>
//#include <winsock2.h>
#include <WS2tcpip.h>

#include <opencv2\opencv.hpp>
#include "opencv2\highgui\highgui.hpp"
#include "opencv2\imgproc\imgproc.hpp"
#include "BlueView.h"
#include <thread>
#include <Windows.h>
#include <string>
#include "SerialPort.h"

#define SERVER_IP "127.0.0.1"

using namespace std;
using namespace cv;
int DELAY_CAPTION = 1500;
int DELAY_BLUR = 100;
int MAX_KERNEL_LENGTH = 31;
Mat dst; Mat src0;
Mat src3 = Mat::zeros(900, 1700, CV_8UC1);
void BlueView_Thread();
void GPS_Thread();
const double pi = 3.14159265358979f;

CBlueView m_pSonar;
int m_nCNT = 0;
int frame = 0;
int setrange = 0;
int thr = 0;
string Name;
string swch = "n";
string save = "n";
string gps_ = "n";

float mapping_x, mapping_y;
float GPS_x = 0, GPS_y = 0;
float rot_angle = 0, mtoPix = 0.1588;

cv::Mat result2;
cv::Mat result3;

int gps_CNT = 0;

double D_latitude = 0, D_longitude = 0, d_head;
double dlat_D, dlat_M, dlon_D, dlon_M;

//위도경도
double a = 6378137; // 적도반경
double b = 6356752.3142; //극점 반경 
double f = (a - b) / a; //flattening
double f1 = 1 / f; //inverse flattening
double rm = sqrt(a * b);  //mean radius
double K0 = 0.9996;  //스케일 팩터
double e = sqrt(1 - pow((b / a), 2));  //이심률
double e2 = e * e / (1 - e * e);
double n = (a - b) / (a + b);

////////////calculate meridional arc length
double A0 = a * (1 - n + (5 * n*n / 4)*(1 - n) + (81 * pow(n, 4) / 64)*(1 - n));
double B0 = (3 * a*n / 2) * (1 - n - (7 * n*n / 8)*(1 - n) + 55 * pow(n, 4) / 64);
double C0 = (15 * a*n*n / 16)*(1 - n + (3 * n*n / 4)*(1 - n));
double D0 = (35 * a*pow(n, 3) / 48)*(1 - n + 11 * n*n / 16);
double E0 = (315 * a*pow(n, 4) / 51)*(1 - n);

double init_x, init_y, init_head;
double N, E;

CSerialPort m_pSerial;

//////// tcp/ip//////////

string tcpip = "n";
WSADATA wsaData;
SOCKET hSocket;
SOCKADDR_IN servAddr;
const int port = 5556;

std::ifstream myfile;
std::string file_name("Result.jpg");
char *file_data;
char sign;
char file_size_ch[4] = { 0, };
int file_size;


double aaa = 0;
double bbb = 0;


int _tmain(int argc, _TCHAR* argv[])
{
	std::cout << "Using GPS? [y/n] : ";
	std::cin >> gps_;
	std::cout << "Save into (.son) File? [y/n] : ";
	std::cin >> save;
	std::cout << "Send Image With TCP/IP? [y/n] : ";
	std::cin >> tcpip;
	std::cout << "Using In Real Time? [y/n] : ";
	std::cin >> swch;
	std::cout << "Set The Threshold : ";
	std::cin >> thr;
	std::cout << "Set The Range : ";
	std::cin >> setrange;
	if (swch == "n"){
		std::cout << "Set The File Name : ";
		std::cin >> Name;
		Name.append(".son");
		const char *fName = Name.c_str();
		
		m_pSonar.SetName(fName);
	}

	m_pSonar.SetRange(setrange);
	m_pSonar.SetSwitch(swch);
	m_pSonar.Savefile(save);

	m_pSonar.Initialize();

	
	////////////////// GPS
	if (gps_ == "y")
	{
		if (m_pSerial.OpenPort(L"COM1"))   // 실제 사용될 COM Port 를 넣어야합니다.  
			{
				std::cout << "Serial Open Success!!";

				// BaudRate, ByteSize, fParity, Parity, StopBit 정보를 설정해줍니다.  
				m_pSerial.ConfigurePort(CBR_9600, 8, FALSE, NOPARITY, ONESTOPBIT);
				// Timeout 설정입니다. 별다른거 없으면 전부 zero 설정해도 되구요.  
				m_pSerial.SetCommunicationTimeouts(0, 0, 0, 0, 0);
			}
		else
			{
				std::cout << "Serial Open Failed!!";
			}
	}	
	
	/////////////////


	std::cout << "\n Completed!";

	std::thread t1(&BlueView_Thread);
	t1.join();

	if (gps_ == "y")
	{
		std::thread t2(&GPS_Thread);
		t2.join();
	}
	
	

	return 0;	
}

void BlueView_Thread() {

	while (1)
	{
		////////GPS
		if (gps_ == "y" && gps_CNT >= 1)
		{
			if (m_nCNT == 0)
			{
				init_x = E/mtoPix;
				init_y = N/mtoPix;
				init_head = d_head;
			}
			else if (m_nCNT == 1)
			{
				GPS_x = init_x - E/mtoPix;
				GPS_y = init_y - N/mtoPix;
				rot_angle = init_head - d_head;
			}
			else
			{
				GPS_x = GPS_x - E/mtoPix;
				GPS_y = GPS_y - N/mtoPix;
			}

			rot_angle = rot_angle - d_head;
		}

		//std::cout << "\nN:" << D_latitude << "\n";
		//std::cout << "\nE:" << D_longitude << "\n";
		//std::cout << "\nH:" << d_head << "\n";
		std::cout << "\nN:" << N << "\n";
		std::cout << "\nE:" << E << "\n";

		/////////////////////////

		m_pSonar.SetPing();

		std::vector<float> vRanges, vAngles, vIntensities;
		std::vector<int> vPixels;
		std::vector<int> vPixels2;

		m_pSonar.GetRangeData(vRanges); //call by reference
		m_pSonar.GetAngleData(vAngles);
		m_pSonar.GetIntensityData(vIntensities);

		int nHeight, nWidth;
		m_pSonar.GetImgHeight(nHeight);
		m_pSonar.GetImgWidth(nWidth);
		m_pSonar.GetPixel(vPixels);
		m_pSonar.GetPixel2(vPixels2);

		//////GPS
		//m_pSonar.GetGps(DM_latitude, DM_longitude);

		Mat result;
		Mat src0 = Mat(vPixels);
		Mat src1 = Mat(vPixels2);

		Mat src = src0.reshape(1, nHeight);
		Mat src2 = src1.reshape(1, nHeight);

		int height = src.size().height;
		int width = src.size().width;

		double min, max;
		minMaxLoc(src, &min, &max);
		minMaxLoc(src2, &min, &max);

		resize(src, src, Size(src.cols / 4, src.rows / 4), 0, 0, CV_INTER_NN);
		resize(src2, src2, Size(src2.cols / 4, src2.rows / 4), 0, 0, CV_INTER_NN);

		src = src * 100;
		src2 = src2 * 100;


		//gabor
		Mat dest;
		Mat src_f;
		src.convertTo(src_f, CV_32F);

		int kernel_size = 3;

		double sig = 1.5, th = pi / 4, lm = pi / 16, gm = 1, ps = 0;
		//double sig = 1.5, th = pi, lm = pi, gm = 1, ps = 0;

		Mat kernel = getGaborKernel(Size(kernel_size, kernel_size), sig, th, lm, gm, ps);
		filter2D(src_f, dest, CV_32F, kernel);
		//cerr << dest(Rect(30, 30, 10, 10)) << endl; // peek into the data
		Mat viz;
		dest.convertTo(viz, CV_8U, 1.0 / 255.0);     // move to proper[0..255] range to show it


		//Custom Filtering

		float kernel_vec[] = { -1, 0, -1, 0, 0, 0, 1, 0, 1 };
		Mat Custom_kernel(3, 3, CV_32F, kernel_vec);

		filter2D(viz, result, CV_8U, Custom_kernel);

		//gaussian
		GaussianBlur(result, result, Size(kernel_size, kernel_size), 1);


		//Binary
		threshold(result, result, thr, 255, THRESH_BINARY);


		// Clustering

		//Get all Non Black Points
		vector<Point> pts;
		findNonZero(result, pts);

		if (pts.size() != 0)
		{
			// Define the Distance will belong to the same Cluster
			int euclidean_distance = 10;


			// Apply Partition
			// All Pixels Within the Given Distance Will Belong to the Same Cluster

			vector<int> labels;

			//With Functor

			// With Lambda Function
			int th2 = euclidean_distance * euclidean_distance;
			int n_labels = partition(pts, labels, [th2](const Point& lhs, const Point& rhs) {return (((lhs.x - rhs.x)*(lhs.x - rhs.x) + (lhs.y - rhs.y)*(lhs.y - rhs.y)) < th2); });

			// Store all Points in same Cluster, and Compute Centroids
			vector<vector<Point>> clusters(n_labels);
			vector<Point> centeroids(n_labels, Point(0, 0));

			for (int i = 0; i < pts.size(); ++i)
			{
				clusters[labels[i]].push_back(pts[i]);
				centeroids[labels[i]] += pts[i];
			}
			for (int i = 0; i < n_labels; ++i)
			{
				centeroids[i].x /= clusters[i].size();
				centeroids[i].y /= clusters[i].size();
			}

			// Draw Results

			// Build a Vector of Random Color, One for Each Class (label)
			vector<Vec3b> colors;
			for (int i = 0; i < n_labels; ++i)
			{
				colors.push_back(Vec3b(rand() * 255, rand() & 255, rand() & 255));
			}

			// Draw the Points
			Mat3b res(result.rows, result.cols, Vec3b(0, 0, 0));
			for (int i = 0; i < pts.size(); i++)
			{
				if (clusters[labels[i]].size() < 20)
				{
					res(pts[i]) = 0;
				}
				else
				{
					res(pts[i]) = colors[labels[i]];
				}
			}

			// Draw Centroids

			/*
			for (int i = 0; i < n_labels; i++)
			{
			circle(res, centeroids[i], 3, Scalar(colors[i][0], colors[i][1], colors[i][2]), CV_FILLED);
			circle(res, centeroids[i], 6, Scalar(255 - colors[i][0], 255 - colors[i][1], 255 - colors[i][2]));
			}
			*/

			//imshow("Clustered Image", res);
			//imshow("Binarized Image", result);
			//imshow("original Image", src2);


			///Drawing Global Map///

			//결과 이미지 이진화

			Mat res_bi;
			Mat res_border;
			Mat res_rotate;

			cvtColor(res, res_bi, COLOR_BGR2GRAY);
			threshold(res_bi, res_bi, 1, 255, THRESH_BINARY);

			//이미지 회전

			int angle = atan2(res_bi.rows, res_bi.cols / 2);
			int aa = sqrt(pow(res_bi.cols / 2, 2) + pow(res_bi.rows, 2));

			//cv::copyMakeBorder(res_bi, res_border, aa - res_bi.rows, aa, aa - res_bi.cols / 2, aa - res_bi.cols / 2, cv::BORDER_CONSTANT);
			cv::copyMakeBorder(res_bi * 100, res_border, aa - res_bi.rows, aa, aa - res_bi.cols / 2, aa - res_bi.cols / 2, cv::BORDER_CONSTANT);

			cv::Mat rotate = cv::getRotationMatrix2D(cv::Point2f(res_border.cols / 2, aa), rot_angle, 1);

			cv::warpAffine(res_border, res_rotate, rotate, cv::Size(res_border.cols, res_border.rows));


			//원점 좌표
			int org_x = (src3.cols / 2) - res_border.cols / 2;
			int org_y = src3.rows - res_border.rows / 2;

			mapping_x = GPS_x + org_x;
			mapping_y = GPS_y + org_y;

			cv::Rect rect(mapping_x, mapping_y, res_border.cols, res_border.rows);

			//ROI 설정

			cv::Rect src3_rect(0, 0, src3.cols, src3.rows); //전역맵의 RECT
			cv::Rect rect_map = rect & src3_rect; // 전역맵과 바이너리맵의 교집합 rect

			cv::Mat imageROI = src3(rect_map); //전역맵에서 교집합 rect 만큼을 ROI 설정
			//cv::Mat res_ = res_border(rect_map - cv::Point(rect_map.x, rect_map.y)); //바이너리 맵에서 교집합 만큼만 자른다
			cv::Mat res_ = res_rotate(rect_map - cv::Point(rect_map.x, rect_map.y)); //바이너리 맵에서 교집합 만큼만 자른다
			cv::Mat mask(res_); //마스크 설정
			res_.copyTo(imageROI, mask); //전역맵으로 복사

			//Drawing Global Map///


			imshow("MAP", src3);
			//imshow("Clustered Image", res);*/
		}
		else
		{
			printf("\n Nothing To Cluster!!\n");
			//imshow("Binarized Image", result);
			//imshow("original Image", src2);
			imshow("MAP", src3);
		}

		//Sleep(100);
		waitKey(100);

		vRanges.clear();
		vAngles.clear();
		vIntensities.clear();

		m_pSonar.SetCNT(m_nCNT);
		m_nCNT++;


		//////// tcp/ip//////////

		if (tcpip == "y")
		{
			//Save IMG
			imwrite("Result.jpg", src3);

			myfile.open(file_name, std::ios::in | std::ios::binary);

			if (myfile.fail())
			{
				std::cout << "\n전송할 파일 읽기 실패" << std::endl;
			}
			myfile.seekg(0, std::ios::end);
			file_size = myfile.tellg();
			myfile.seekg(0, std::ios::beg);
			*((int*)file_size_ch) = file_size;

			file_data = new char[file_size];
			myfile.read(file_data, file_size);

			if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0){
				std::cout << "WSAStartup() error" << std::endl;
			}

			hSocket = socket(PF_INET, SOCK_STREAM, 0);

			if (hSocket == INVALID_SOCKET){
				std::cout << "socket() error" << std::endl;
			}

			memset(&servAddr, 0, sizeof(servAddr));
			servAddr.sin_family = AF_INET;
			servAddr.sin_port = htons(port);

			inet_pton(AF_INET, SERVER_IP, &servAddr.sin_addr.s_addr);

			if (connect(hSocket, (SOCKADDR*)&servAddr, sizeof(servAddr)) == SOCKET_ERROR){
				std::cout << "connect() error" << std::endl;
			}

			std::cout << "메시지 대기중." << std::endl;
			recv(hSocket, &sign, 1, 0);

			std::cout << "파일 크기 값 보내는 중 (" << file_size << ")" << std::endl;
			send(hSocket, file_size_ch, sizeof(file_size_ch), 0);

			recv(hSocket, &sign, 1, 0);

			std::cout << "파일 데이터 보내는 중" << std::endl;
			send(hSocket, file_data, file_size, 0);

			myfile.close();
			closesocket(hSocket);
			delete[] file_data;
		}
		////// tcp/ip
	}
}
void GPS_Thread() {

	while (1)
	{
		BYTE* pByte = new BYTE[150];
		CHAR* GPS_ch = new CHAR[150];
		DOUBLE* GPS_db = new DOUBLE[150];
		
		if (m_pSerial.ReadByte(pByte, 150))
		{
			pByte[149] = '\0';
			GPS_ch = (CHAR*)(pByte);
			//_insertData(CString(reinterpret_cast<char*>(pByte)));
			for (int i = 0; i < 150; ++i)
			{
				if (GPS_ch[i] == char('$'))
				{
					if (GPS_ch[i + 2] == char('P') && GPS_ch[i + 3] == char('G') && i < 104)
					{
						//std::cout << "N:" << pByte[i-14] << pByte[i - 13] << pByte[i - 12] << pByte[i - 11] << "\n";
						//d_latitude = int(pByte[i + 17] - '0') * 10.0 + int(pByte[i + 18] - '0') + int(pByte[i + 19] - '0') * 0.1 + int(pByte[i + 20] - '0') * 0.01;
						dlat_D = int(pByte[i + 17] - '0') * 10.0 + int(pByte[i + 18] - '0');
						dlat_M = int(pByte[i + 19] - '0') * 10.0 + int(pByte[i + 20] - '0');
						for (int j = 0; j < 8; j++)
						{
							dlat_M = dlat_M + int(pByte[i + 22 + j] - '0') / (pow(10, j + 1));
						}
						D_latitude = dlat_D + (dlat_M / 60);
						std::cout << "\nN:" << D_latitude << "\n";

						//std::cout << "W:" << pByte[i +2] << pByte[i +3] << pByte[i +4] << pByte[i +5] << pByte[i + 6] << "\n";
						//d_longitude = (pByte[i + 33] - '0')*100.0 + (pByte[i + 34] - '0')*10.0 + (pByte[i + 35] - '0') + (pByte[i + 36] - '0')*0.1 + (pByte[i + 37] - '0')*0.01;
						dlon_D = int(pByte[i + 33] - '0')*100.0 + int(pByte[i + 34] - '0')*10.0 + int(pByte[i + 35] - '0');
						dlon_M = int(pByte[i + 36] - '0')*10.0 + int(pByte[i + 37] - '0');
						for (int j = 0; j < 8; j++)
						{
							dlon_M = dlon_M + int(pByte[i + 39 + j] - '0') / (pow(10, j + 1));
						}
						D_longitude = dlon_D + (dlat_M / 60);
						std::cout << "W:" << D_longitude << "\n";
					}
					else if (GPS_ch[i + 2] == char('P') && GPS_ch[i + 3] == char('V'))
					{
						if (GPS_ch[i + 9] == char('.') && i < 140)
						{
							//std::cout << "H:" << pByte[i +7] << pByte[i +8] << '.'<< pByte[i + 10] << "\n";
							d_head = int(pByte[i + 7] - '0')*10.0 + int(pByte[i + 8] - '0') + int(pByte[i + 10] - '0')*0.1;
						}
						else if (i < 139)
						{
							//std::cout << "H:" << pByte[i +7] << pByte[i +8] << pByte[i +9] << '.'<< pByte[i + 11] << "\n";
							d_head = int(pByte[i + 7] - '0')*100.0 + int(pByte[i + 8] - '0')*10.0 + int(pByte[i + 9] - '0') + int(pByte[i + 11] - '0')*0.1;
						}
						std::cout << "\nH:" << d_head << "\n";
					}
				}
			}


			// 삭제해주면 되는 간편한 놈이죠... 클래스를 보시면 알겠지만
			// 1byte씩 읽거나 그 이상도 읽을 수 있습니다.
			// Write 해주는 부분도 비슷합니다. 응용해보세요 ~

			delete[] pByte, GPS_ch, GPS_db;
		}

		m_pSonar.GetGps(D_latitude, D_longitude, d_head);
		//_insertData(CString(reinterpret_cast<char*>(pByte)));

		
		//좌표계 변환

		double r_latitude = D_latitude*pi / 180;
		int numl = (D_longitude / 6) + 31;
		double a = 6378137; // 적도반경
		double b = 6356752.3142; //극점 반경
		double f = (a - b) / a; //flattening
		double f1 = 1 / f; //inverse flattening
		double rm = sqrt(a * b);  //mean radius
		double K0 = 0.9996;  //스케일 팩터
		double e = sqrt(1 - pow((b / a), 2));  //이심률
		double e2 = e * e / (1 - e * e);
		double n = (a - b) / (a + b);
		////////////////////
		double rho = a * (1 - e * e) / sqrt((pow((1 - pow(e*sin(r_latitude), 2)), 3)));   //R curv1
		double nuu = a / sqrt(1 - pow(e*sin(r_latitude), 2));  //R curv2
		////////////calculate meridional arc length
		double A0 = a * (1 - n + (5 * n*n / 4)*(1 - n) + (81 * pow(n, 4) / 64)*(1 - n));
		double B0 = (3 * a*n / 2) * (1 - n - (7 * n*n / 8)*(1 - n) + 55 * pow(n, 4) / 64);
		double C0 = (15 * a*n*n / 16)*(1 - n + (3 * n*n / 4)*(1 - n));
		double D0 = (35 * a*pow(n, 3) / 48)*(1 - n + 11 * n*n / 16);
		double E0 = (315 * a*pow(n, 4) / 51)*(1 - n);
		double S = (A0 * r_latitude) - (B0 * sin(2 * r_latitude)) + (C0 * sin(4 * r_latitude)) - (D0 * sin(6 * r_latitude)) + (E0 * sin(8 * r_latitude));
		///calculation constants
		double jj = 6 * numl - 183;
		double J = D_longitude - jj;
		double P = J * pi / 180; //delta long
		/////////utm 좌표 계수
		double K1 = S * K0;
		double K2 = nuu * sin(r_latitude)*cos(r_latitude)*K0 / 2;
		double K3 = ((nuu*sin(r_latitude)*pow(cos(r_latitude), 3)) / 24)*(5 - pow(tan(r_latitude), 2) + 9 * e2*pow(cos(r_latitude), 2) + 4 * pow(e2, 2)*pow(cos(r_latitude), 4))*K0;
		double K4 = nuu * cos(r_latitude)*K0;
		double K5 = (pow(cos(r_latitude), 3)*(nuu / 6)) * (1 - pow(tan(r_latitude), 2) + e2 * pow(cos(r_latitude), 2))*K0;
		double s1 = sin(1)*pi / 180;
		double A6 = (pow((P*s1), 6)*nuu*sin(r_latitude)*pow(cos(r_latitude), 5 / 720))*(61 - 58 * pow(tan(r_latitude), 2) + pow(tan(r_latitude), 4) + 270 * e2*pow(cos(r_latitude), 2) - 330 * e2*pow(sin(r_latitude), 2))*K0 *(1E+24);
		/////////
		N = (K1 + K2*P*P + K3*pow(P, 4)); ///UTM-N
		E = 500000 + (K4*P + K5 * pow(P, 3));//UTM-E

		gps_CNT++;

		return(void)D_latitude, (void)D_longitude, (void)d_head, (void)N, (void)E;

	}
	
}




	/*
	if (m_pSerial.ReadByte(pByte, 150))
	{
	pByte[149] = '\0';
	//_insertData(CString(reinterpret_cast<char*>(pByte)));
	for (int i = 0; i < 150; ++i)
	{
	if (pByte[i] == '$')
	{
	if (pByte[i + 2] == 'P' && pByte[i + 3] == 'G')
	{
	//std::cout << "N:" << pByte[i-14] << pByte[i - 13] << pByte[i - 12] << pByte[i - 11] << "\n";
	//d_latitude = (pByte[i + 17] - '0') * 10.0 + (pByte[i + 18] - '0') + (pByte[i + 19] - '0') * 0.1 + (pByte[i + 20] - '0') * 0.01;
	dlat_D = (pByte[i + 17] - '0') * 10.0 + (pByte[i + 18] - '0');
	dlat_M = (pByte[i + 19] - '0') * 10.0 + (pByte[i + 20] - '0');
	for (int j = 0; j < 8; j++)
	{
	dlat_M = dlat_M + pByte[i + 22 + j] / (pow(10, j + 1));
	}
	DM_latitude = dlat_D * 100 + dlat_M;
	std::cout << "\nN:" << DM_latitude << "\n";

	//std::cout << "W:" << pByte[i +2] << pByte[i +3] << pByte[i +4] << pByte[i +5] << pByte[i + 6] << "\n";
	//d_longitude = (pByte[i + 33] - '0')*100.0 + (pByte[i + 34] - '0')*10.0 + (pByte[i + 35] - '0') + (pByte[i + 36] - '0')*0.1 + (pByte[i + 37] - '0')*0.01;
	dlon_D = (pByte[i + 33] - '0')*100.0 + (pByte[i + 34] - '0')*10.0 + (pByte[i + 35] - '0');
	dlon_M = (pByte[i + 36] - '0')*10.0 + (pByte[i + 37] - '0');
	for (int j = 0; j < 8; j++)
	{
	dlon_M = dlon_M + pByte[i + 39 + j] / (pow(10, j + 1));
	}
	DM_longitude = dlon_D * 100 + dlon_M;
	std::cout << "W:" << DM_longitude << "\n";
	}
	else if (pByte[i + 2] == 'P' && pByte[i + 3] == 'V')
	{
	if (pByte[i + 9] == '.')
	{
	//std::cout << "H:" << pByte[i +7] << pByte[i +8] << '.'<< pByte[i + 10] << "\n";
	d_head = (pByte[i + 7] - '0')*10.0 + (pByte[i + 8] - '0') + (pByte[i + 10] - '0')*0.1;
	}
	else
	{
	//std::cout << "H:" << pByte[i +7] << pByte[i +8] << pByte[i +9] << '.'<< pByte[i + 11] << "\n";
	d_head = (pByte[i + 7] - '0')*100.0 + (pByte[i + 8] - '0')*10.0 + (pByte[i + 9] - '0') + (pByte[i + 11] - '0')*0.1;
	}
	std::cout << "\nH:" << d_head << "\n";
	}
	}
	}

	// 삭제해주면 되는 간편한 놈이죠... 클래스를 보시면 알겠지만
	// 1byte씩 읽거나 그 이상도 읽을 수 있습니다.
	// Write 해주는 부분도 비슷합니다. 응용해보세요 ~

	delete[] pByte;
	*/

/*
if (_serial.OpenPort(L"COM4"))   // 실제 사용될 COM Port 를 넣어야합니다.
{
// BaudRate, ByteSize, fParity, Parity, StopBit 정보를 설정해줍니다.
_serial.ConfigurePort(CBR_9600, 8, FALSE, NOPARITY, ONESTOPBIT);
// Timeout 설정입니다. 별다른거 없으면 전부 zero 설정해도 되구요.
_serial.SetCommunicationTimeouts(0, 0, 0, 0, 0);

// Buffer를 잡고
BYTE* pByte = new BYTE[512];

// 읽는데 성공하면 처리하고
if (_serial.ReadByte(pByte, 512))
{
pByte[511] = '\0';
_insertData(CString(reinterpret_cast<char*>(pByte)));
}

// 삭제해주면 되는 간편한 놈이죠... 클래스를 보시면 알겠지만
// 1byte씩 읽거나 그 이상도 읽을 수 있습니다.
// Write 해주는 부분도 비슷합니다. 응용해보세요 ~
delete[] pByte;
}

// 다 썼으니 닫아주면 끝~
_serial.ClosePort();


출처: https ://mnlt.tistory.com/119 [MNLT's life story]

*/