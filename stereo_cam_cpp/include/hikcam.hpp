#ifndef HIKCAM_HPP
#define HIKCAM_HPP

#include <string>
#include <vector>
#include <thread>
#include <opencv2/opencv.hpp>

#include "MvCameraControl.h"

#define MAX_CAMERA_NUM     4 // 1 for monocular, 2 for stereo, 4 for stereo down-facing and stereo foward camera


class HikCam
{
public:
	long int id;
	HikCam();
	HikCam(int num);
	bool init();
	~HikCam();
	bool getImage(unsigned char **imgs, const bool bconvert=false);
	bool getImage(std::vector<cv::Mat> &imgs);
	void getSingleImage(unsigned char *img, bool &success, const bool bconvert=false, const int flag = 0);
	void getSingleImage(cv::Mat &img, bool &success, const int flag = 0);
	int getCamCount();
	MV_FRAME_OUT_INFO_EX getImgInfo(int flag=0);
	bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);

private:
	int nRet;
	int bstart;
	int camera_count = MAX_CAMERA_NUM;
	void* mhandle[MAX_CAMERA_NUM] = {nullptr};
    unsigned char* mImg[MAX_CAMERA_NUM];
    unsigned int mDataSize[MAX_CAMERA_NUM];
    MV_FRAME_OUT_INFO_EX mImageInfo[MAX_CAMERA_NUM];
    std::string mSN[MAX_CAMERA_NUM];
    void readImage(void* handle, unsigned char* img, unsigned int dataSize, 
    					MV_FRAME_OUT_INFO_EX& imgInfo, bool &success);
	void readConvertImage(void* handle, unsigned char* img, unsigned int dataSize, 
							MV_FRAME_OUT_INFO_EX& imgInfo, bool &success);
	void readMat(void* handle, cv::Mat &img, unsigned int dataSize, 
							MV_FRAME_OUT_INFO_EX& imgInfo, bool &success);

};

// ********************************************************************** //
HikCam::HikCam()
{
	id = 0;
	bstart = false;
}

HikCam::HikCam(int num):camera_count(num)
{
	assert (camera_count <= MAX_CAMERA_NUM);
	id = 0;
	bstart = false;
}

HikCam::~HikCam()
{	
	printf("Stop capturing......\n");
    for(int i = 0; i < camera_count; i++)
    {
        // end grab image
        if (mhandle[i] == nullptr)
        	continue;
        nRet = MV_CC_StopGrabbing(mhandle[i]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
        }

        // close device
        nRet = MV_CC_CloseDevice(mhandle[i]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
        }

        // destroy handle
        nRet = MV_CC_DestroyHandle(mhandle[i]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
        }
    }
}

bool HikCam::init()
{
	MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    // enum device
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet)
    {
        printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
        return false;
    }
    unsigned int nIndex = 0;
    if (stDeviceList.nDeviceNum > 0)
    {
        for (int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            printf("[device %d]:\n", i);
            MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo)
            {
                break;
            } 
            PrintDeviceInfo(pDeviceInfo);            
        }  
    } 
    else
    {
        printf("Find No Devices!\n");
        return false;
    }

    if(stDeviceList.nDeviceNum < camera_count)
    {
        printf("only have %d camera\n", stDeviceList.nDeviceNum);
        return false;
    }

	// Tips for multicamera testing
	printf("Start %d camera\n", camera_count);
    for(int i = 0; i < camera_count; i++)
    {

        // select device and create mhandle
        nRet = MV_CC_CreateHandle(&mhandle[i], stDeviceList.pDeviceInfo[i]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CreateHandle fail! [device %d]\n", i);
            MV_CC_DestroyHandle(mhandle[i]);            
            return false;
        }

        // open device
        nRet = MV_CC_OpenDevice(mhandle[i]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_OpenDevice fail! [device %d]\n", i);
            MV_CC_DestroyHandle(mhandle[i]);
            return false;
        }

        // set the camera as trigger model while the defualt of right is trigger
        nRet = MV_CC_SetBoolValue(mhandle[i], "AcquisitionFrameRateEnable", false);
        if (MV_OK != nRet)
        {
            printf("set AcquisitionFrameRateEnable fail! nRet [%x]\n", nRet);
            return false;
        }
        // set trigger mode as on
        nRet = MV_CC_SetEnumValue(mhandle[i], "TriggerMode", 1);
        if (MV_OK != nRet)
        {
            printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
            return false;
        }
        // set trigger source
        nRet = MV_CC_SetEnumValue(mhandle[i], "TriggerSource", MV_TRIGGER_SOURCE_SOFTWARE);
        if (MV_OK != nRet)
        {
            printf("MV_CC_SetTriggerSource fail! nRet [%x]\n", nRet);
            return false;
        }
        nRet = MV_CC_SetFloatValue(mhandle[i], "AutoGainUpperLimit", 9.99f);
        if (MV_OK != nRet)
        {
            printf("MV_CC_SetTriggerSource fail! nRet [%x]\n", nRet);
            return false;
        }
        
        // set Gain Auto once "GainAuto"
        nRet = MV_CC_SetEnumValue(mhandle[i], "GainAuto", 1);
        if (MV_OK != nRet)
        {
            printf("MV_CC_SetGainAuto fail! nRet [%x]\n", nRet);
            return false;
        }
        // set white balance once "BalanceWhiteAuto",
        nRet = MV_CC_SetEnumValue(mhandle[i],"BalanceWhiteAuto", 1);
        if (MV_OK != nRet)
        {
            printf("MV_CC_SetBalanceWhiteAuto fail! nRet [%x]\n", nRet);
            return false;
        }
        
    }

	// start grab image
    for(int i = 0; i < camera_count; i++)
    {
        nRet = MV_CC_StartGrabbing(mhandle[i]);
        if (MV_OK != nRet)
        {
            printf("Cam[%d]: MV_CC_StartGrabbing fail! nRet [%x]\n",i, nRet);
            return false;
        }
	    // Get payload size
	    MVCC_INTVALUE stParam;
	    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
	    nRet = MV_CC_GetIntValue(mhandle[i], "PayloadSize", &stParam);
	    if (MV_OK != nRet)
	    {
	        printf("Cam[%d]: Get PayloadSize fail! nRet [0x%x]\n", i, nRet);
	        return false;
	    }

	    mImg[i] = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
	    if (nullptr == mImg[i])
	    {
	    	printf("Cam[%d]: Fail to alloc memory.\n", i);
	        return false;
	    }
	    mDataSize[i] = stParam.nCurValue;
	    // get SN
	    MVCC_STRINGVALUE stStringValue = {0};
    	char camSerialNumber[256] = {0};
	    nRet = MV_CC_GetStringValue(mhandle[i], "DeviceSerialNumber", &stStringValue);
        if (MV_OK == nRet)
        {
            memcpy(camSerialNumber, stStringValue.chCurValue, sizeof(stStringValue.chCurValue));
            mSN[i] = camSerialNumber;
        }
        else
        {
            printf("Cam[%d]: Get DeviceUserID Failed! nRet = [%x]\n", i, nRet);
            return false;
        }
		memset(&mImageInfo[i], 0, sizeof(MV_FRAME_OUT_INFO_EX));
	/*
        // set Gain Auto off "GainAuto"
        nRet = MV_CC_SetEnumValue(mhandle[i], "GainAuto", 0);
        if (MV_OK != nRet)
        {
            printf("MV_CC_SetGainAuto fail! nRet [%x]\n", nRet);
            return false;
        }
        // set white balance off "BalanceWhiteAuto",
        nRet = MV_CC_SetEnumValue(mhandle[i],"BalanceWhiteAuto", 0);
        if (MV_OK != nRet)
        {
            printf("MV_CC_SetBalanceWhiteAuto fail! nRet [%x]\n", nRet);
            return false;
        }

        
        // set Gain Auto once "GainAuto"
        nRet = MV_CC_SetGainMode(mhandle[i], 1);
        if (MV_OK != nRet)
        {
            printf("MV_CC_SetGainAuto fail! nRet [%x]\n", nRet);
            return false;
        }
        // set white balance once "BalanceWhiteAuto",
        nRet = MV_CC_SetBalanceWhiteAuto(mhandle[i], 2);
        if (MV_OK != nRet)
        {
            printf("MV_CC_SetBalanceWhiteAuto fail! nRet [%x]\n", nRet);
            return false;
        }*/
    }
    bstart = true;
    return bstart;
}

void HikCam::readImage(void* handle, unsigned char* img, unsigned int dataSize, 
							MV_FRAME_OUT_INFO_EX& imgInfo, bool &success)
{
	img = (unsigned char *)malloc(sizeof(unsigned char) * dataSize);
	int status = MV_CC_GetOneFrameTimeout(handle, img, dataSize, &imgInfo, 1000);
	success = status == MV_OK;
}


void HikCam::readConvertImage(void* handle, unsigned char* img, unsigned int dataSize, 
							MV_FRAME_OUT_INFO_EX& imgInfo, bool &success)
{
	//unsigned char* pData = (unsigned char *)malloc(sizeof(unsigned char) * dataSize);
	unsigned char* pData = new unsigned char [sizeof(unsigned char) * dataSize];
	int status = MV_CC_GetOneFrameTimeout(handle, pData, dataSize, &imgInfo, 1000);
	int convertImgSize = imgInfo.nWidth * imgInfo.nHeight *  4 + 2048;
	img = new unsigned char [convertImgSize];
	MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};

	stConvertParam.nWidth = imgInfo.nWidth;
    stConvertParam.nHeight = imgInfo.nHeight;
    stConvertParam.pSrcData = pData;
    stConvertParam.nSrcDataLen = imgInfo.nFrameLen;;
    stConvertParam.enSrcPixelType = imgInfo.enPixelType;
    stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    stConvertParam.pDstBuffer = img;
    stConvertParam.nDstBufferSize = convertImgSize;
    status = MV_CC_ConvertPixelType(handle, &stConvertParam);
	success = status == MV_OK;
}

void HikCam::readMat(void* handle, cv::Mat &img, unsigned int dataSize, 
							MV_FRAME_OUT_INFO_EX& imgInfo, bool &success)
{
	unsigned char* pData = new unsigned char [sizeof(unsigned char) * dataSize];
	int status = MV_CC_GetOneFrameTimeout(handle, pData, dataSize, &imgInfo, 1000);
	int convertImgSize = imgInfo.nWidth * imgInfo.nHeight *  4 + 2048;
	unsigned char* imgBuff = new unsigned char [convertImgSize];
	MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};

	stConvertParam.nWidth = imgInfo.nWidth;
    stConvertParam.nHeight = imgInfo.nHeight;
    stConvertParam.pSrcData = pData;
    stConvertParam.nSrcDataLen = imgInfo.nFrameLen;;
    stConvertParam.enSrcPixelType = imgInfo.enPixelType;
    stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    stConvertParam.pDstBuffer = imgBuff;
    stConvertParam.nDstBufferSize = convertImgSize;
    status = MV_CC_ConvertPixelType(handle, &stConvertParam);
    cv::Mat buff(imgInfo.nHeight, imgInfo.nWidth, CV_8UC3, imgBuff); 
	img = buff.clone();
	success = status == MV_OK;
	delete[] pData;
	delete[] imgBuff;
}


bool HikCam::getImage(unsigned char **imgs, const bool bconvert)
{

    bool vbsuccess[camera_count];
    std::thread* vthreads[camera_count];
    for (int i = 0; i < camera_count; ++i)
    {
        nRet = MV_CC_SetCommandValue(mhandle[i], "TriggerSoftware");
        if(MV_OK != nRet)
        {
            printf("failed in TriggerSoftware[%x]\n", nRet);
        }

    	if(!bconvert)
    		vthreads[i]= new std::thread(&HikCam::readImage, this, mhandle[i], imgs[i], mDataSize[i], 
    									std::ref(mImageInfo[i]), std::ref(vbsuccess[i]));
    	else
    		vthreads[i]= new std::thread(&HikCam::readConvertImage, this, mhandle[i], imgs[i], mDataSize[i], 
    									std::ref(mImageInfo[i]), std::ref(vbsuccess[i]));
    }
  	for (auto imgThread: vthreads)
    {
    	imgThread->join();
    }
    int imgID = mImageInfo[0].nFrameNum;
 	for (int i = 0; i < camera_count; ++i)
    {
	    if (!vbsuccess[i] || imgID!=mImageInfo[i].nFrameNum)
	    {
	        printf("Cam[%d] Get One Frame failed!\n", i);
	        return false;
	    }
	}
	return true;
}

bool HikCam::getImage(std::vector<cv::Mat> &imgs)
{
	imgs.clear();
	imgs.resize(camera_count);

    bool vbsuccess[camera_count];
    std::thread* vthreads[camera_count];
    for (int i = 0; i < camera_count; ++i)
    {
        nRet = MV_CC_SetCommandValue(mhandle[i], "TriggerSoftware");
        if(MV_OK != nRet)
        {
            printf("failed in TriggerSoftware[%x]\n", nRet);
        }
    	vthreads[i]= new std::thread(&HikCam::readMat, this, mhandle[i], std::ref(imgs[i]), mDataSize[i], 
    									std::ref(mImageInfo[i]), std::ref(vbsuccess[i]));
    }
  	for (auto imgThread: vthreads)
    {
    	imgThread->join();
    }
    int imgID = mImageInfo[0].nFrameNum;
 	for (int i = 0; i < camera_count; ++i)
    {
	    if (!vbsuccess[i] || imgID!=mImageInfo[i].nFrameNum)
	    {
	        printf("Cam[%d] Get One Frame failed!\n", i);
	        return false;
	    }
	}
	return true;
}

void HikCam::getSingleImage(unsigned char *img, bool &success, const bool bconvert, const int flag)
{
	assert (flag < camera_count);
    nRet = MV_CC_SetCommandValue(mhandle[flag], "TriggerSoftware");
    if(MV_OK != nRet)
    {
        printf("failed in TriggerSoftware[%x]\n", nRet);
    }
	if(!bconvert)
		readImage(mhandle[flag], img, mDataSize[flag], mImageInfo[flag], success);
	else
		readConvertImage(mhandle[flag], img, mDataSize[flag], mImageInfo[flag], success);

	if (!success)
	{
        printf("Cam[%d] Get One Frame failed!\n", flag);
	}
}

void HikCam::getSingleImage(cv::Mat &img, bool &success, const int flag)
{
	assert (flag < camera_count);

	nRet = MV_CC_SetCommandValue(mhandle[flag], "TriggerSoftware");
    if(MV_OK != nRet)
    {
        printf("failed in TriggerSoftware[%x]\n", nRet);
    }

	readMat(mhandle[flag], img, mDataSize[flag], mImageInfo[flag], success);

	if (!success)
	{
        printf("Cam[%d] Get One Frame failed!\n", flag);
	}
}

int HikCam::getCamCount()
{
	return camera_count;
}

MV_FRAME_OUT_INFO_EX HikCam::getImgInfo(int flag)
{
	assert (flag < camera_count);
	return mImageInfo[flag];
}


bool HikCam::PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("%s\n", "The Pointer of pstMVDevInfoList is NULL!");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        printf("%s %x\n", "nCurrentIp:", pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp);                 
        printf("%s %s\n\n", "chUserDefinedName:", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName); 
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }
    return true;
}

#endif //HIKCAM_HPP
