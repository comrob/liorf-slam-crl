/**
 * @file      ProbabilisticKernelOptimizer.hpp
 * @brief     Probabilistic Kernel Optimization for adaptive robust estimation.
 */

#ifndef PROBABILISTIC_KERNEL_OPTIMIZER_HPP
#define PROBABILISTIC_KERNEL_OPTIMIZER_HPP

#include <vector>
#include <string>

namespace lio {

struct PKOConfig {
    bool use_adaptive = true;                    
    double min_scale_factor = 0.01;              
    double max_scale_factor = 10.0;              
    int num_alpha_segments = 100;                
    double truncated_threshold = 10.0;           
    int gmm_components = 3;                      
    int gmm_sample_size = 1000;                  
    
    PKOConfig() = default;
};

class ProbabilisticKernelOptimizer {
public:
    explicit ProbabilisticKernelOptimizer(const PKOConfig& config = PKOConfig());
    ~ProbabilisticKernelOptimizer() = default;
    
    double CalculateScaleFactor(const std::vector<double>& residuals);
    void Reset();
    const PKOConfig& GetConfig() const { return m_config; }

private:
    void InitializePKO();
    void FitGMM(const std::vector<double>& residuals);
    double GaussianPDF(double x, double mean, double variance) const;
    double CalculatePartitionFunction(double alpha) const;
    double HuberKernelWeight(double residual, double delta) const;
    double CalculateJSDivergence(double alpha);

private:
    PKOConfig m_config;
    std::vector<double> m_alpha_candidates;
    std::vector<double> m_partition_functions;
    double m_alpha_star_ref;
    bool m_initialized;
    
    std::vector<double> m_gmm_weights;
    std::vector<double> m_gmm_means;
    std::vector<double> m_gmm_variances;
};

} // namespace lio

#endif // PROBABILISTIC_KERNEL_OPTIMIZER_HPP