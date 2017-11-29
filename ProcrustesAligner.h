#pragma once
#include "SimpleMesh.h"

class ProcrustesAligner {
public:
    Matrix4f estimatePose(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
        ASSERT(sourcePoints.size() == targetPoints.size() && "The number of source and target points should be the same, since every source point is matched with corresponding target point.");

        // We estimate the pose between source and target points using Procrustes algorithm.
        // Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
        // from source points to target points.

        auto sourceMean = computeMean(sourcePoints);
        auto targetMean = computeMean(targetPoints);

        Matrix3f rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
        Vector3f translation = computeTranslation(sourceMean, targetMean, rotation);

        Matrix4f estimatedPose = Matrix4f::Identity();
        estimatedPose.block(0, 0, 3, 3) = rotation;
        estimatedPose.block(0, 3, 3, 1) = translation;

        return estimatedPose;
    }

private:
    Vector3f computeMean(const std::vector<Vector3f>& points) {
        // TODO: Compute the mean of input points.
        int size = points.size();
        Vector3f result = Vector3f::Zero();
        for (int i = 0; i < size; i++) {
            result += points[i];
        }
        result /= size;
        return result;
    }

    Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
        // TODO: Estimate the rotation from source to target points, following the Procrustes algorithm. 
        // To compute the singular value decomposition you can use JacobiSVD() from Eigen.     
        Matrix3f covariance = Matrix3f::Zero();
        int size = sourcePoints.size();
        int i;
        Vector3f sourceDifference, targetDifference;
        //MatrixXi sourceMatrix(size, 3), targetMatrix(size, 3);
        for (i = 0; i < size; i++) {
            sourceDifference = sourcePoints[i] - sourceMean;
            targetDifference = targetPoints[i] - targetMean;

            covariance(0, 0) += sourceDifference.x() * targetDifference.x();
            covariance(0, 1) += sourceDifference.x() * targetDifference.y();
            covariance(0, 2) += sourceDifference.x() * targetDifference.z();
            covariance(1, 0) += sourceDifference.y() * targetDifference.x();
            covariance(1, 1) += sourceDifference.y() * targetDifference.y();
            covariance(1, 2) += sourceDifference.y() * targetDifference.z();
            covariance(2, 0) += sourceDifference.z() * targetDifference.x();
            covariance(2, 1) += sourceDifference.z() * targetDifference.y();
            covariance(2, 2) += sourceDifference.z() * targetDifference.z();

            /*sourceMatrix(i, 0) = sourceDifference.x();
            sourceMatrix(i, 1) = sourceDifference.y();
            sourceMatrix(i, 2) = sourceDifference.z();

            targetMatrix(i, 0) = targetDifference.x();
            targetMatrix(i, 1) = targetDifference.y();
            targetMatrix(i, 2) = targetDifference.z();*/
        }
        /*sourceMatrix.transposeInPlace();
        covariance = sourceMatrix * targetMatrix;  //doesn't work*/

        JacobiSVD<MatrixXf> svd(covariance, ComputeFullU | ComputeFullV);

        Matrix3f rotationMatrix;
        rotationMatrix = svd.matrixU() * svd.matrixV().transpose();
        return rotationMatrix;
    }

    Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean, const Matrix3f& rotation) {
        // TODO: Compute the translation vector from source to target opints.
        Vector3f translation;
        translation = targetMean - sourceMean;
        translation = rotation * translation;
        return translation;
    }
};