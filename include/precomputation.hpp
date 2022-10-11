#include <iostream>
#include <math.h>
#include <vector>
#include <cassert>

#include <DataTypes/DataTypes.hpp>
#include <eigen3/Eigen/Core>

template<typename T>
void pop_front(std::vector<T>& vec)
{
    assert(!vec.empty());
    vec.erase(vec.begin());
};

template<typename T>
void pop_2ndback(std::vector<T>& vec)
{
    assert(!vec.empty());
    vec.erase(vec.end()-2);
};

struct dfltValues{
    float len = 10e-3;
    float d = 2e-3;
    float E = 10000e3;
    float v = 0.43;
};

//Evaluation functions
MatrixXd EvaluateK(std::vector<Link> &iLinks);
void DirectKinematics(std::vector<PosOrientation> &iPosVec, std::vector<Joint> &iJoints, std::vector<Link> &iLinks);
MatrixXd EvaluateJacobian(std::vector<PosOrientation> &iPosVec);
MatrixXd MagtoFieldMap(std::vector<Joint> &iJoints);

//Utility functions
MatrixXd StackDiagonals(std::vector<Matrix3d> matrices);
Matrix3d RotationZYX(Matrix3d src, Vector3d jointAngles);
Matrix3d SkewMatrix(Vector3d src);
MatrixXd VerticalStack(MatrixXd M1, MatrixXd M2);
VectorXd StackAngles(std::vector<Joint>& iJoints);

int main(int argc, char* argv[]);

