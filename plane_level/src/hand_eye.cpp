#include "plane_level/plane_level.hpp"

#include <opencv2/calib3d.hpp>

#include <cmath>
#include <stdexcept>

namespace plane_level {
namespace {

cv::Vec3d rotvecOf(const cv::Matx33d& R) {
    cv::Vec3d w;
    cv::Rodrigues(R, w);
    return w;
}

cv::Matx33d kabschOfRotvecs(const std::vector<cv::Vec3d>& beta, const std::vector<cv::Vec3d>& alpha) {
    // alpha ≈ Rx beta. H = sum beta alpha^T, Rx = V U^T from H = U S V^T.
    cv::Matx33d H = cv::Matx33d::zeros();
    for (size_t i = 0; i < beta.size(); ++i) H += beta[i] * alpha[i].t();
    cv::SVD svd(cv::Mat(H), cv::SVD::FULL_UV);
    cv::Matx33d U, Vt;
    svd.u.copyTo(U);
    svd.vt.copyTo(Vt);
    cv::Matx33d V = Vt.t();
    cv::Matx33d R = V * U.t();
    if (cv::determinant(cv::Mat(R)) < 0) {
        V(0, 2) = -V(0, 2);
        V(1, 2) = -V(1, 2);
        V(2, 2) = -V(2, 2);
        R = V * U.t();
    }
    return R;
}

}

HandEyeMotions relativeMotions(const std::vector<HandEyeSample>& samples) {
    if (samples.size() < 2) throw std::invalid_argument("hand-eye needs at least two poses");
    HandEyeMotions motions;
    for (size_t i = 0; i + 1 < samples.size(); ++i) {
        motions.A.push_back(samples[i + 1].gripperInBase.inverse() * samples[i].gripperInBase);
        motions.B.push_back(samples[i + 1].targetInCamera * samples[i].targetInCamera.inverse());
    }
    return motions;
}

Rigid3d solveHandEye(const std::vector<HandEyeSample>& samples) {
    const HandEyeMotions motions = relativeMotions(samples);
    std::vector<cv::Vec3d> alpha;
    std::vector<cv::Vec3d> beta;
    alpha.reserve(motions.A.size());
    beta.reserve(motions.B.size());
    for (size_t i = 0; i < motions.A.size(); ++i) {
        const cv::Vec3d a = rotvecOf(motions.A[i].R);
        const cv::Vec3d b = rotvecOf(motions.B[i].R);
        if (cv::norm(a) < 1e-4 || cv::norm(b) < 1e-4) continue;
        alpha.push_back(a);
        beta.push_back(b);
    }
    if (alpha.size() < 2) throw std::invalid_argument("hand-eye needs two motions with non-parallel rotation");
    const cv::Matx33d Rx = kabschOfRotvecs(beta, alpha);

    cv::Mat M(static_cast<int>(motions.A.size() * 3), 3, CV_64F);
    cv::Mat y(static_cast<int>(motions.A.size() * 3), 1, CV_64F);
    for (size_t i = 0; i < motions.A.size(); ++i) {
        const cv::Matx33d left = motions.A[i].R - cv::Matx33d::eye();
        const cv::Vec3d rhs = Rx * motions.B[i].t - motions.A[i].t;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) M.at<double>(static_cast<int>(i) * 3 + r, c) = left(r, c);
            y.at<double>(static_cast<int>(i) * 3 + r, 0) = rhs[r];
        }
    }
    cv::Mat tx;
    if (!cv::solve(M, y, tx, cv::DECOMP_SVD)) throw std::runtime_error("hand-eye translation failed");
    return Rigid3d::fromRotationTranslation(Rx, cv::Vec3d(tx.at<double>(0), tx.at<double>(1), tx.at<double>(2)));
}

Rigid3d solveHandEyeOpenCv(const std::vector<HandEyeSample>& samples) {
    if (samples.size() < 3) throw std::invalid_argument("OpenCV hand-eye needs at least 3 poses");
    std::vector<cv::Mat> Rg, tg, Rt, tt;
    for (const HandEyeSample& s : samples) {
        Rg.emplace_back(cv::Mat(s.gripperInBase.R));
        tg.emplace_back(cv::Mat(s.gripperInBase.t));
        Rt.emplace_back(cv::Mat(s.targetInCamera.R));
        tt.emplace_back(cv::Mat(s.targetInCamera.t));
    }
    cv::Mat R, t;
    cv::calibrateHandEye(Rg, tg, Rt, tt, R, t, cv::CALIB_HAND_EYE_PARK);
    cv::Matx33d Rx;
    R.copyTo(Rx);
    return Rigid3d::fromRotationTranslation(Rx, cv::Vec3d(t.at<double>(0), t.at<double>(1), t.at<double>(2)));
}

cv::Vec3d cameraToBase(const cv::Vec3d& pCamera, const Rigid3d& gripperInBase, const Rigid3d& cameraInGripper) {
    return gripperInBase.apply(cameraInGripper.apply(pCamera));
}

}  // namespace plane_level
