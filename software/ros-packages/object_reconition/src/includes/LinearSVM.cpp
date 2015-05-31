#include "LinearSVM.h"    
void LinearSVM::getSupportVector(std::vector<float>& support_vector) const {
    int sv_count = get_support_vector_count();
    const CvSVMDecisionFunc* df = decision_func;
    const double* alphas = df[0].alpha;
    double rho = df[0].rho;
    int var_count = get_var_count();
    support_vector.resize(var_count, 0);
    for (unsigned int r = 0; r < (unsigned)sv_count; r++) {
        float myalpha = alphas[r];
        const float* v = get_support_vector(r);
        for (int j = 0; j < var_count; j++,v++) {
            support_vector[j] += (-myalpha) * (*v);
        }
    }
    support_vector.push_back(rho);
}
