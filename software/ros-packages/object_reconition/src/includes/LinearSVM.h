#ifndef LINEAR_SVM_H_
#define LINEAR_SVM_H_
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>

class LinearSVM: public CvSVM {
    public:
          void getSupportVector(std::vector<float>& support_vector) const;
};  

#endif /* LINEAR_SVM_H_ */
