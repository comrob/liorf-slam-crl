#include "degeneracyDetection/DegeneracyDetector.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

namespace {

float calculateMedianDistance(pcl::PointCloud<PointType>::Ptr cloud_map, bool verbose = false)
{
    if (cloud_map->size() < 2) return 0.0f;

    std::vector<float> distances;
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(cloud_map);

    for (size_t i = 0; i < cloud_map->points.size(); ++i)
    {
        std::vector<int> point_idx_nkn_search(2);
        std::vector<float> point_nkn_squared_distance(2);

        if (kdtree.nearestKSearch(cloud_map->points[i], 2, point_idx_nkn_search, point_nkn_squared_distance) > 1)
        {
            distances.push_back(std::sqrt(point_nkn_squared_distance[1]));
        }
    }

    if (distances.size() < 2) return 0.0f;

    std::sort(distances.begin(), distances.end());
    size_t mid_index = distances.size() / 2;
    float median_distance = (distances.size() % 2 == 0) ? (distances[mid_index - 1] + distances[mid_index]) / 2.0f : distances[mid_index];

    if (verbose)
        std::cout << "Median distance calculated: " << median_distance << std::endl;

    return median_distance;
}

Eigen::Vector3f calculateMedianPoint(pcl::PointCloud<PointType>::Ptr cloud)
{
    std::vector<float> x_coords, y_coords, z_coords;
    for (const auto &point : cloud->points)
    {
        x_coords.push_back(point.x);
        y_coords.push_back(point.y);
        z_coords.push_back(point.z);
    }

    std::sort(x_coords.begin(), x_coords.end());
    std::sort(y_coords.begin(), y_coords.end());
    std::sort(z_coords.begin(), z_coords.end());

    size_t mid_index = cloud->points.size() / 2;
    return Eigen::Vector3f(x_coords[mid_index], y_coords[mid_index], z_coords[mid_index]);
}

float computeMedian(std::vector<float> &values)
{
    if (values.empty()) return 0.0f;
    size_t size = values.size();
    std::nth_element(values.begin(), values.begin() + size / 2, values.end());
    float median = values[size / 2];
    if (size % 2 == 0)
    {
        std::nth_element(values.begin(), values.begin() + (size / 2 - 1), values.end());
        median = (median + values[size / 2 - 1]) / 2.0f;
    }
    return median;
}

float computeQuantile(std::vector<float> &data, float quantile)
{
    if (data.empty()) return 0.0f;
    size_t n = data.size();
    size_t index = static_cast<size_t>(quantile * (n - 1));
    std::nth_element(data.begin(), data.begin() + index, data.end());
    return data[index];
}

float computeMedianDistance(const pcl::PointCloud<PointType>::Ptr &cloud)
{
    if (cloud->empty()) return 0.0f;

    size_t step = std::max(size_t(1), cloud->size() / 20);
    std::vector<float> distances;
    for (size_t i = 0; i < cloud->size(); i += step)
    {
        const auto &point = cloud->points[i];
        float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        distances.push_back(distance);
    }
    return computeMedian(distances);
}

float computeMedianDistance(const pcl::PointCloud<PointType>::Ptr &cloud1,
                             const pcl::PointCloud<PointType>::Ptr &cloud2)
{
    if (cloud1->empty() || cloud2->empty()) return 0.0f;

    size_t step = std::max(size_t(1), std::min(cloud1->size(), cloud2->size()) / 200);
    std::vector<float> dispacements;
    std::vector<float> distances_from_center;
    
    for (size_t i = 0; i < std::min(cloud1->size(), cloud2->size()); i += step)
    {
        const auto &point1 = cloud1->points[i];
        float distance_from_center = std::sqrt(point1.x * point1.x + point1.y * point1.y + point1.z * point1.z);
        distances_from_center.push_back(distance_from_center);
        
        const auto &point2 = cloud2->points[i];
        float displacement = std::sqrt((point1.x - point2.x) * (point1.x - point2.x) +
                                       (point1.y - point2.y) * (point1.y - point2.y) +
                                       (point1.z - point2.z) * (point1.z - point2.z));
        dispacements.push_back(displacement);
    }

    float distance_threshold = computeQuantile(distances_from_center, 0.2f);

    std::vector<float> filtered_dispacements;
    for (size_t i = 0; i < dispacements.size(); ++i)
    {
        if (distances_from_center[i] < distance_threshold)
        {
            filtered_dispacements.push_back(dispacements[i]);
        }
    }
    return computeMedian(filtered_dispacements);
}

float getPerturbationAngleRelativeToMedian(
    pcl::PointCloud<PointType>::Ptr cloud_scan,
    const Eigen::Vector3f &median_point,
    float perturbation_step)
{
    float representative_distance = computeMedianDistance(cloud_scan);
    if(representative_distance < 1e-5f) return 0.0f;
    return std::asin(std::min(1.0f, perturbation_step / representative_distance)) * 2.0f;
}

Eigen::Vector3f crossProduct(const Eigen::Vector3f &a, const Eigen::Vector3f &b)
{
    return Eigen::Vector3f(
        a.y() * b.z() - a.z() * b.y(),
        a.z() * b.x() - a.x() * b.z(),
        a.x() * b.y() - a.y() * b.x());
}

float computeMedianLinearVelocity(
    const pcl::PointCloud<PointType>::Ptr &cloud,
    const TwistVector &twist,
    const Eigen::Matrix4f &initial_pose = Eigen::Matrix4f::Identity())
{
    if (cloud->empty()) return 0.0f;

    Eigen::Vector3f linear_velocity(twist[0], twist[1], twist[2]);
    Eigen::Vector3f angular_velocity(twist[3], twist[4], twist[5]);

    size_t step = std::max(size_t(1), cloud->size() / 20);
    std::vector<float> linear_velocity_magnitudes;
    
    for (size_t i = 0; i < cloud->size(); i += step)
    {
        const auto &point = cloud->points[i];
        Eigen::Vector4f homogeneous_point(point.x, point.y, point.z, 1.0f);
        Eigen::Vector4f transformed_point = initial_pose * homogeneous_point;
        Eigen::Vector3f local_point = transformed_point.head<3>();

        Eigen::Vector3f velocity = linear_velocity + crossProduct(angular_velocity, local_point);
        linear_velocity_magnitudes.push_back(velocity.norm());
    }

    return computeMedian(linear_velocity_magnitudes);
}

std::vector<TwistVector> scaleBasis(const std::vector<TwistVector> &basis,
                                        const pcl::PointCloud<PointType>::Ptr &chunk_cloud,
                                        const float perturbation_amount = 1.0f)
{
    std::vector<TwistVector> basis_scaled;
    for (size_t i = 0; i < basis.size(); ++i)
    {
        TwistVector basis_i = basis[i];
        float median_point_velocity = computeMedianLinearVelocity(chunk_cloud, basis_i);
        if(median_point_velocity < 1e-6f) median_point_velocity = 1e-6f;

        TwistVector scaled_twist = basis_i * perturbation_amount / median_point_velocity;
        basis_scaled.push_back(scaled_twist);
    }
    return basis_scaled;
}

std::pair<int, std::vector<TwistVector>> computeSubspaceBasis(
    const std::vector<TwistVector> &vectors, float tolerance, bool verbose = false)
{
    if (vectors.empty()) return {0, {}};

    Eigen::MatrixXf matrix(6, vectors.size());
    for (size_t i = 0; i < vectors.size(); ++i)
    {
        matrix.col(i) = vectors[i];
    }

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXf singularValues = svd.singularValues();

    int rank = 0;
    for (Eigen::Index i = 0; i < singularValues.size(); ++i)
    {
        if (singularValues[i] > tolerance) ++rank;
    }

    if (verbose)
    {
        std::cout << "Singular values: ";
        for (Eigen::Index i = 0; i < singularValues.size(); ++i) std::cout << singularValues[i] << " ";
        std::cout << std::endl;
    }

    Eigen::MatrixXf basisMatrix = svd.matrixU().leftCols(rank);
    std::vector<TwistVector> basis;
    for (int i = 0; i < rank; ++i)
    {
        basis.push_back(basisMatrix.col(i) * singularValues[i]);
    }

    return {rank, basis};
}

std::vector<TwistVector> sparsifyBasisPreservingSubspace(const std::vector<TwistVector> &basis)
{
    const int dim = 6;
    int rank = basis.size();
    if (rank == 0) return {};

    Eigen::MatrixXf B(dim, rank);
    for (int i = 0; i < rank; ++i) B.col(i) = basis[i];

    Eigen::MatrixXf T = Eigen::MatrixXf::Identity(rank, rank);

    for (int iter = 0; iter < 100; ++iter)
    {
        Eigen::MatrixXf B_prime = B * T;
        Eigen::MatrixXf grad = B.transpose() * B_prime.array().sign().matrix();
        
        float learningRate = 0.01f;
        T -= learningRate * grad;

        Eigen::JacobiSVD<Eigen::MatrixXf> svd(T, Eigen::ComputeFullU | Eigen::ComputeFullV);
        T = svd.matrixU() * Eigen::MatrixXf::Identity(rank, rank) * svd.matrixV().transpose();

        if (grad.norm() < 1e-6f) break;
    }

    Eigen::MatrixXf B_final = B * T;
    std::vector<TwistVector> sparsifiedBasis;
    for (int i = 0; i < rank; ++i)
    {
        sparsifiedBasis.push_back(B_final.col(i));
    }
    return sparsifiedBasis;
}

} // End anonymous namespace

std::vector<TwistVector> DegeneracyDetector::orthonormalizeBasis(const std::vector<TwistVector> &basis) const
{
    if (basis.empty()) return {};

    std::vector<TwistVector> orthonormal_basis;
    for (const auto &v : basis)
    {
        TwistVector u = v;
        for (const auto &b : orthonormal_basis)
        {
            u -= (b.dot(u) / b.dot(b)) * b;
        }

        float norm = u.norm();
        if (norm > 1e-6f)
        {
            orthonormal_basis.push_back(u / norm);
        }
    }
    return orthonormal_basis;
}

DegeneracyDetector::DegeneracyDetector(const DegeneracyParams &parameters) : params(parameters), failed(false) {}

void DegeneracyDetector::evalDegeneracyPerturbation(
    const float poseEulerArray[6],
    pcl::PointCloud<PointType>::Ptr cloud_scan,
    pcl::PointCloud<PointType>::Ptr cloud_map,
    std::shared_ptr<lio::IMappingBackend> mappingBackend)
{
    failed = false;
    fail_reason = "";
    twists_perturbations_degeneration.clear();
    descriptive_numbers.clear();
    last_perturbed_scans.clear();
    last_aligned_scans.clear();
    last_perturbed_poses.clear();
    last_aligned_poses.clear();
    last_optimization_paths.clear();

    float median_distance = calculateMedianDistance(cloud_map, params.verbose);
    if(median_distance < 1e-5f) median_distance = 1.0f; // Prevent div zero
    
    float perturbation_step = median_distance * params.n_multiplier;
    Eigen::Vector3f median_point = calculateMedianPoint(cloud_scan);
    float perturbation_angle = getPerturbationAngleRelativeToMedian(cloud_scan, median_point, perturbation_step);

    float max_perturbation_angle_rad = params.max_perturbation_angle_deg * M_PI / 180.0f;
    if (perturbation_angle > max_perturbation_angle_rad)
    {
        perturbation_angle = max_perturbation_angle_rad;
    }

    std::vector<TwistVector> perturbations;
    perturbations.push_back((TwistVector() << perturbation_step, 0.0f, 0.0f, perturbation_angle, 0.0f, 0.0f).finished());
    perturbations.push_back((TwistVector() << 0.0f, perturbation_step, 0.0f, 0.0f, perturbation_angle, 0.0f).finished());
    perturbations.push_back((TwistVector() << 0.0f, 0.0f, perturbation_step, 0.0f, 0.0f, perturbation_angle).finished());

    Eigen::Matrix4f poseOptimizedMat = pcl::getTransformation(
        poseEulerArray[3], poseEulerArray[4], poseEulerArray[5],
        poseEulerArray[0], poseEulerArray[1], poseEulerArray[2]).matrix();

    for (size_t perturbationIdx = 0; perturbationIdx < perturbations.size(); ++perturbationIdx)
    {
        const auto &perturbation = perturbations[perturbationIdx];
        Eigen::Matrix4f posePerturbedMat = poseOptimizedMat * expMap(perturbation);
        last_perturbed_poses.push_back(posePerturbedMat);

        pcl::PointCloud<PointType>::Ptr cloud_perturbed_map(new pcl::PointCloud<PointType>);
        pcl::transformPointCloud(*cloud_scan, *cloud_perturbed_map, posePerturbedMat);
        last_perturbed_scans.push_back(cloud_perturbed_map);
        
        float posePerturbedEuler[6];
        pcl::getTranslationAndEulerAngles(Eigen::Affine3f(posePerturbedMat), 
            posePerturbedEuler[3], posePerturbedEuler[4], posePerturbedEuler[5],
            posePerturbedEuler[0], posePerturbedEuler[1], posePerturbedEuler[2]);

        lio::AlignmentMetrics metrics = mappingBackend->align(cloud_scan, posePerturbedEuler, true);
        const auto &trace = mappingBackend->getLastAlignmentTrace();
        std::vector<Eigen::Matrix4f> optimizationPath;
        optimizationPath.reserve(trace.iteration_poses.size() + 2);
        optimizationPath.push_back(posePerturbedMat);
        for (const auto &poseStep : trace.iteration_poses)
            optimizationPath.push_back(poseStep);
        
        if (metrics.final_correspondences < 50) { 
            failed = true;
            fail_reason = "Degeneracy Aligner failed: Not enough correspondences (" + std::to_string(metrics.final_correspondences) + 
                          "). Scan size: " + std::to_string(cloud_scan->size()) + 
                          ", Perturbation step: " + std::to_string(perturbation_step);
            last_aligned_poses.push_back(posePerturbedMat);
            last_aligned_scans.push_back(cloud_perturbed_map);
            optimizationPath.push_back(posePerturbedMat);
            last_optimization_paths.push_back(std::move(optimizationPath));
            continue;
        }

        Eigen::Matrix4f matRecovered = pcl::getTransformation(
            posePerturbedEuler[3], posePerturbedEuler[4], posePerturbedEuler[5],
            posePerturbedEuler[0], posePerturbedEuler[1], posePerturbedEuler[2]).matrix();

        Eigen::Matrix4f matDiffRelative = poseOptimizedMat.inverse() * matRecovered;
        last_aligned_poses.push_back(matRecovered);

        pcl::PointCloud<PointType>::Ptr cloud_aligned_map(new pcl::PointCloud<PointType>);
        pcl::transformPointCloud(*cloud_scan, *cloud_aligned_map, matRecovered);
        last_aligned_scans.push_back(cloud_aligned_map);
        optimizationPath.push_back(matRecovered);
        last_optimization_paths.push_back(std::move(optimizationPath));

        pcl::PointCloud<PointType>::Ptr cloud_perturbated(new pcl::PointCloud<PointType>);
        pcl::transformPointCloud(*cloud_scan, *cloud_perturbated, expMap(perturbation));

        pcl::PointCloud<PointType>::Ptr cloud_transformed(new pcl::PointCloud<PointType>);
        pcl::transformPointCloud(*cloud_scan, *cloud_transformed, matDiffRelative);

        float median_perturb_magnitude = computeMedianDistance(cloud_scan, cloud_perturbated);
        float median_translation_magnitude = computeMedianDistance(cloud_scan, cloud_transformed);

        float descriptive_number = median_translation_magnitude / (median_perturb_magnitude + 1e-6f);

        if (descriptive_number > params.descriptive_number_threshold)
        {
            twists_perturbations_degeneration.push_back(matrixToTwist(matDiffRelative));
            descriptive_numbers.push_back(descriptive_number);
        }
    }
}

std::string DegeneracyDetector::getDegeneracyDirectionsString() const
{
    std::stringstream ss;
    for (size_t i = 0; i < twists_perturbations_degeneration.size(); i++)
    {
        TwistVector twist = twists_perturbations_degeneration[i];
        Eigen::Vector3f translation = twist.segment<3>(0);
        Eigen::Vector3f axis = twist.segment<3>(3);
        float angle = axis.norm();
        if (angle > 1e-5f) axis.normalize();
        
        ss << i << ") " << descriptive_numbers[i] << ": \t"
           << "trans xyz: [" << translation.x() << ", " << translation.y() << ", " << translation.z() << "] \t"
           << "angle: " << angle << " axis: [" << axis.x() << ", " << axis.y() << ", " << axis.z() << "]\n";
    }
    return ss.str();
}

std::string DegeneracyDetector::getFinalBasisString() const
{
    std::stringstream ss;
    if (last_sparsified_basis.empty()) {
        return "No final basis extracted.\n";
    }
    
    ss << "Final Sparsified & Scaled Projection Basis:\n";
    for (size_t i = 0; i < last_sparsified_basis.size(); i++) {
        TwistVector twist = last_sparsified_basis[i];
        Eigen::Vector3f trans = twist.segment<3>(0);
        Eigen::Vector3f rot = twist.segment<3>(3);
        
        ss << "  " << i << ") trans: [" << trans.x() << ", " << trans.y() << ", " << trans.z() << "] "
           << "rot: [" << rot.x() << ", " << rot.y() << ", " << rot.z() << "]\n";
    }
    return ss.str();
}

std::vector<TwistVector> DegeneracyDetector::extractBasisFromTwists(
    const std::vector<TwistVector> &twists,
    pcl::PointCloud<PointType>::Ptr cloud_scan)
{
    last_raw_twists = twists; // Store Step A

    float medianDistancefromCenter = computeMedianDistance(cloud_scan);
    if(medianDistancefromCenter < 1e-5f) medianDistancefromCenter = 1.0f;

    std::vector<TwistVector> twistsScaled = twists;
    for (size_t i = 0; i < twistsScaled.size(); i++) {
        twistsScaled[i].segment<3>(3) *= medianDistancefromCenter;
    }

    // Step B: PCA
    auto [rank, pca_basis] = computeSubspaceBasis(twistsScaled, params.eigen_value_threshold, params.verbose);
    
    // Scale PCA back for visualization storage
    last_pca_basis = pca_basis;
    for (size_t i = 0; i < last_pca_basis.size(); i++) {
        last_pca_basis[i].segment<3>(3) /= medianDistancefromCenter;
    }

    // Step C: Sparsification
    std::vector<TwistVector> sparsified_basis;
    if (rank > 0) {
        sparsified_basis = sparsifyBasisPreservingSubspace(pca_basis);
    }
    
    // Scale Sparsified back for visualization storage
    last_sparsified_basis = sparsified_basis;
    for (size_t i = 0; i < last_sparsified_basis.size(); i++) {
        last_sparsified_basis[i].segment<3>(3) /= medianDistancefromCenter;
    }

    ++consistency_stats.eval_count;
    consistency_stats.detected = !last_sparsified_basis.empty();
    if (consistency_stats.detected)
    {
        ++consistency_stats.hit_count;
        ++consistency_stats.hit_streak;
        consistency_stats.miss_streak = 0;
    }
    else
    {
        ++consistency_stats.miss_streak;
        consistency_stats.hit_streak = 0;
    }

    // Return the final, fully scaled basis for actual projection
    return scaleBasis(last_sparsified_basis, cloud_scan);
}