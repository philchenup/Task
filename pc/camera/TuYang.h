/**
 * @file TuYang.h
 * @brief 图漾（Percipio）相机。继承 ICamera，对外槽函数与 MechMind 一致。
 */
#ifndef CT_DEVICE_TUYANG_H
#define CT_DEVICE_TUYANG_H

#include "ICamera.h"

#include "TYApi.h"
#include "TyIsp.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>

class TuYang : public ICamera
{
    Q_OBJECT
public:
    explicit TuYang(QWidget* parent = nullptr);
    ~TuYang();

    void updateCamInfo(int) override;

    bool containsInvalidPoint(const std::vector<TY_VECT_3F>& cloud);

    void convertToPCL(const std::vector<TY_VECT_3F>& points, const cv::Mat& color,
        pcl::PointCloud<pcl::PointXYZRGB>& pclCloud);

    void updatePointCloud(const std::vector<TY_VECT_3F>& points, const cv::Mat& color, float time);

public slots:
    void searchDevice() override;
    void captureDevice() override;
    void connectDevice(std::string id, std::string ip) override;
    void connectDevice(int) override;
    void add() override;
    void reset() override;

private:
    bool ensureLib();
    int selectDevices(const std::string& id, const std::string& ip, uint32_t deviceNum,
        std::vector<TY_DEVICE_BASE_INFO>& out);
    bool openDevice(const TY_DEVICE_BASE_INFO& info);
    void closeDevice();
    bool configureComponents();
    bool prepareBuffers();
    bool getDefaultImageMode(TY_COMPONENT_ID component, TY_IMAGE_MODE& imageMode);
    void colorIspInitSetting();
    void parseFrame(const TY_FRAME_DATA& frame, cv::Mat& depth, cv::Mat& color);
    bool parseColor(const TY_IMAGE_DATA& image, cv::Mat& color);
    void normalizeColor(cv::Mat& color);
    bool undistortDepth(cv::Mat& depth);
    bool doRegister(const cv::Mat& depth, const cv::Mat& color, cv::Mat& undistortColor, cv::Mat& out);
    bool handleFrame(const TY_FRAME_DATA& frame, float time);

    ct::Cloud::Ptr m_captured_cloud;
    std::vector<TY_DEVICE_BASE_INFO> deviceInfoList;
    CamData tuYangCamData;

    TY_INTERFACE_HANDLE m_hIface = nullptr;
    TY_DEV_HANDLE m_hDevice = nullptr;
    TY_ISP_HANDLE m_ispHandle = nullptr;
    TY_CAMERA_CALIB_INFO m_depthCalib{};
    TY_CAMERA_CALIB_INFO m_colorCalib{};
    std::vector<char*> m_frameBuffers;
    uint32_t m_frameSize = 0;
    float m_depthScale = 1.f;
    bool m_isDepthDistortion = false;
    bool m_hasColorCalib = false;
    bool m_mapDepthToColor = false;
    bool m_withColorCam = true;
    bool m_libInited = false;
    bool connected = false;
};

#endif // CT_DEVICE_TUYANG_H
