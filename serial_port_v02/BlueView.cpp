#include "stdafx.h"
#include "BlueView.h"
#include <windows.h> 

CBlueView::CBlueView()
{
	m_pHead = NULL;
	m_bFlag = true;
	m_nCNT = 0;
}


CBlueView::~CBlueView()
{
	if (!m_vAngles.empty())
	{
		m_vAngles.erase(m_vAngles.begin(), m_vAngles.end());
		m_vRanges.erase(m_vRanges.begin(), m_vRanges.end());
		m_vIntensities.erase(m_vIntensities.begin(), m_vIntensities.end());
	}

	if (!m_vPixels.empty())
		m_vPixels.erase(m_vPixels.begin(), m_vPixels.end());


	//delete[] m_fRangeData;

	BVTMagImage_Destroy(m_pImg);
	BVTSonar_Destroy(m_pFile);
	BVTSonar_Destroy(m_pSon);
}

bool CBlueView::GetFlag()
{
	return m_bFlag;
}

void CBlueView::SetCNT(int nCNT)
{
	m_nCNT = nCNT;
}

void CBlueView::Initialize()
{
	m_pSon = BVTSonar_Create();
	if (m_pSon == NULL)
		MessageBox(nullptr, TEXT("BVTSonar_Create: failed!!"), TEXT("Message"), MB_OK);

	if (m_nSwitch == "y")
	{
		m_nRet = BVTSonar_Open(m_pSon, "NET", "192.168.1.45");
	}
	else
		m_nRet = BVTSonar_Open(m_pSon, "FILE", m_nFName);


	if (m_nRet != 0)
		MessageBox(nullptr, TEXT("BVTSonar_Open: Ret!!"), TEXT("Message"), MB_OK);

	m_nHeads = BVTSonar_GetHeadCount(m_pSon);
	m_nRet = BVTSonar_GetHead(m_pSon, 0, &m_pHead);
	if (m_nRet != 0)
	{
		m_nRet = BVTSonar_GetHead(m_pSon, 1, &m_pHead);
		if (m_nRet != 0)
		{
			MessageBox(nullptr, TEXT("BVTSonar_GetHead: Ret!!"), TEXT("Message"), MB_OK);
		}
	}

	BVTHead_SetRange(m_pHead, 0, m_nRange);


	m_pFile = BVTSonar_Create();
	if (m_pFile == NULL)
		MessageBox(nullptr, TEXT("BVTSonar_File_Create: failed!!"), TEXT("Message"), MB_OK);


	/////颇老 历厘//////

	if (m_nSave == "y")
	{
		m_nRet = BVTSonar_CreateFile(m_pFile, "out.son", m_pSon, "");
		if (m_nRet != 0)
		{
			BVTSonar_Destroy(m_pFile);
			MessageBox(nullptr, TEXT("out.son file was not cleared!!!"), TEXT("Message"), MB_OK);
			Sleep(5000);
		}
		m_pOutHead = NULL;
		m_nRet = BVTSonar_GetHead(m_pFile, 0, &m_pOutHead);
		if (m_nRet != 0)
			MessageBox(nullptr, TEXT("BVTSonar_GetOutHead: Ret!!"), TEXT("Message"), MB_OK);
	}

	///////////


	m_pNav = BVTNavData_Create();
	if (m_pNav == NULL)
		MessageBox(nullptr, TEXT("BVTNavdata_Create: failed!!"), TEXT("Message"), MB_OK);

	MessageBox(nullptr, TEXT("BVTSonar_Initialiezation Completed!!"), TEXT("Message"), MB_OK);
}

void CBlueView::SetPing()
{
	m_bFlag = false;
	BVTPing pPing = NULL;
	int ret;

	//m_nRet = BVTHead_GetPing(m_pHead, -1, &pPing);
	m_nRet = BVTHead_GetPing(m_pHead, m_nCNT, &pPing);
	if (m_nRet != 0)
		MessageBox(nullptr, TEXT("BVTHead_GetPing: Ret!!"), TEXT("Message"), MB_OK);

	TakeSonarRange(pPing);
	BVTPing_GetImageXY(pPing, &m_pImg);
	m_nHeight = BVTMagImage_GetHeight(m_pImg);
	m_nWidth = BVTMagImage_GetWidth(m_pImg);

	if (!m_vPixels.empty())
	{
		m_vPixels.erase(m_vPixels.begin(), m_vPixels.end());
		m_vPixels2.erase(m_vPixels2.begin(), m_vPixels2.end());
	}

	m_vPixels.reserve(m_nHeight*m_nWidth);
	m_vPixels2.reserve(m_nHeight*m_nWidth);
	//m_vPixels = BVTMagImage_CopyBits(m_pImg);


	/*for (int i = 0; i < m_nHeight; i = i + 1) //+2
	{
	for (int j = 0; j < m_nWidth; j = j + 1)//+2
	{
	//m_nPixel[j][i] = BVTMagImage_GetPixel(m_pImg, i, j);

	m_vPixels.push_back(BVTMagImage_GetPixel(m_pImg, i, j));

	//m_vPixels.push_back(0);
	//BVTMagImage_GetPixel(m_pImg, i, j);
	}
	}*/

	for (int i = 0; i < m_nHeight; i = i + 1) //+2
	{
		if ((m_nHeight - 150) < i)
		{
			for (int j = 0; j < m_nWidth; j = j + 1)//+2
			{
				m_vPixels.push_back(0);
				m_vPixels2.push_back(BVTMagImage_GetPixel(m_pImg, i, j));
			}
		}
		else
		{
			for (int j = 0; j < m_nWidth; j = j + 1)//+2
			{
				//m_nPixel[j][i] = BVTMagImage_GetPixel(m_pImg, i, j);
				m_vPixels.push_back(BVTMagImage_GetPixel(m_pImg, i, j));
				m_vPixels2.push_back(BVTMagImage_GetPixel(m_pImg, i, j));
			}
		}
	}
	/////////////////GPS + son Test ///////////////////////////

	if (m_nSave == "y")
	{
		// create new nav data object
		BVTNavData nav_data;
		nav_data = BVTNavData_Create();
		if (nav_data == NULL)
			printf("BVTNavData_Create : faild\n");

		//grab navdata from ping
		m_nRet = BVTNavData_SetLatitude(nav_data, m_nLatitude);
		m_nRet = BVTNavData_SetLongitude(nav_data, m_nLongitude);
		m_nRet = BVTNavData_SetHeading(nav_data, m_nHead);

		//store navdata in to ping object
		m_nRet = BVTPing_PutNavData(pPing, nav_data);
		if (m_nRet != 0)
			printf("BVTPing_PutNavData : ret = %d\n",m_nRet);

		///////颇老 历厘///////
		m_nRet = BVTHead_PutPing(m_pOutHead, pPing);
		if (m_nRet != 0)
			MessageBox(nullptr, TEXT("BVTHead_PutPing: Ret!!"), TEXT("Message"), MB_OK);

		//destroy th pre ping and navdata
		BVTPing_Destroy(pPing);
		BVTNavData_Destroy(nav_data);
		m_bFlag = true;
	}
	/////////////////GPS + son Test ///////////////////////////


	/*
	double dNav;
	m_nRet = BVTNavData_GetLatitude(m_pNav, &dNav);
	if (m_nRet == BVT_NAV_NO_DATA)
	MessageBox(nullptr, TEXT("BVTNavData_GetLatitude: NoData!!"), TEXT("Message"), MB_OK);
	*/
	///////

}

void CBlueView::GetPixel(std::vector<int>& vPixels)
{
	vPixels = m_vPixels;
}

void CBlueView::GetPixel2(std::vector<int>& vPixels)
{
	vPixels = m_vPixels2;
}

void CBlueView::GetRangeData(std::vector<float>& vRanges)
{
	vRanges = m_vRanges;
}

void CBlueView::GetAngleData(std::vector<float>& vAngles)
{
	vAngles = m_vAngles;
}

void CBlueView::GetIntensityData(std::vector<float>& vIntensities)
{
	vIntensities = m_vIntensities;
}

void CBlueView::GetImgHeight(int& nHeight)
{
	nHeight = m_nHeight;
}

void CBlueView::GetImgWidth(int& nWidth)
{
	nWidth = m_nWidth;
}

void CBlueView::SetSize(int nWidth, int nHeight)
{
	m_nWidth = nWidth;
	m_nHeight = nHeight;
}

void CBlueView::TakeSonarRange(BVTPing pPing)
{
	BVTRangeData pRangeData;

	m_nRet = BVTPing_GetRangeData(pPing, &pRangeData);
	if (m_nRet != 0)
		MessageBox(nullptr, TEXT("BVTHead_GetRangeData: Ret!!"), TEXT("Message"), MB_OK);

	int nNumberOfRanges = BVTRangeData_GetCount(pRangeData);

	if (!m_vAngles.empty())
	{
		m_vAngles.erase(m_vAngles.begin(), m_vAngles.end());
		m_vRanges.erase(m_vRanges.begin(), m_vRanges.end());
		m_vIntensities.erase(m_vIntensities.begin(), m_vIntensities.end());
	}

	m_vAngles.reserve(nNumberOfRanges);
	m_vRanges.reserve(nNumberOfRanges);
	m_vIntensities.reserve(nNumberOfRanges);

	for (int i = 0; i < nNumberOfRanges; i++)
	{
		m_vAngles.push_back(BVTRangeData_GetBearingValue(pRangeData, i));
		m_vRanges.push_back(BVTRangeData_GetRangeValue(pRangeData, i));
		m_vIntensities.push_back(BVTRangeData_GetIntensityValue(pRangeData, i));
	}
}

void CBlueView::SetSwitch(string nSwitch)
{
	m_nSwitch = nSwitch;
}

void CBlueView::SetRange(int nRange)
{
	m_nRange = nRange;
}

void CBlueView::SetName(const char * nName)
{
	m_nFName = nName;
}

void CBlueView::Savefile(string nSave)
{
	m_nSave = nSave;
}

///GPS Son

void CBlueView::GetGps(double nlatitude, double nlongitude, double nhead)
{
	m_nLatitude = nlatitude;
	m_nLongitude = nlongitude;
	m_nHead = nhead;
}