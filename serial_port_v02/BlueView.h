#pragma once

#ifdef _WIN32
#	define _CRT_SECURE_NO_WARNINGS
#endif
#include <string.h>
#include <stdio.h>
#include <bvt_sdk.h>

#include <vector>
using namespace std;

class CBlueView
{
public:
	CBlueView();
	~CBlueView();

	void Initialize();
	void SetPing();
	void GetPixel(std::vector<int>& vPixels);
	void GetPixel2(std::vector<int>& vPixels2);
	void GetRangeData(std::vector<float>& vRanges);
	void GetAngleData(std::vector<float>& vAngles);
	void GetIntensityData(std::vector<float>& vIntensities);
	void GetImgHeight(int& nHeight);
	void GetImgWidth(int& nWidth);
	void SetSize(int nWidth, int nHeight);
	void TakeSonarRange(BVTPing pPing);
	bool GetFlag();
	void SetCNT(int nCNT);
	void SetRange(int nRange);
	void SetName(const char * nName);
	void SetSwitch(string nSwitch);
	void Savefile(string nSave);

	///GPS Son TEST
	void printNavData(BVTNavData data);
	void GetGps(double nlatitude, double nlongitude, double nhead);
	void CBlueView::Gps_(string nGps_);


private:
	BVTSonar m_pSon;
	BVTHead m_pHead;
	BVTSonar m_pFile;
	BVTHead m_pOutHead;
	BVTPing m_pPing;
	BVTMagImage m_pImg;
	BVTNavData m_pNav;

	int m_nHeads;
	int m_nRet;
	int m_nWidth;
	int m_nHeight;
	int m_nRange;

	///GPS Son TEST
	double m_nLatitude;
	double m_nLongitude;
	double m_nHead;
	string m_nGps_;

	//float* m_fAngle;
	//float* m_fRange;
	//float* m_fIntensity;
	//double* m_fRangeData;

	bool m_bFlag;
	int m_nCNT;
	const char* m_nFName;
	string m_nSwitch;
	string m_nSave;

	std::vector<float> m_vRanges;
	std::vector<float> m_vAngles;
	std::vector<float> m_vIntensities;
	std::vector<int> m_vPixels;
	std::vector<int> m_vPixels2;
};


