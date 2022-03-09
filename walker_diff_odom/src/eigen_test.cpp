
#include <eigen3/Eigen/Core>
#include <iostream>

using namespace std;
using namespace Eigen;

// g++ eigen_test.cpp -o my_program; ./my_program
int main (){
    cout << " Eigen version : " << EIGEN_MAJOR_VERSION << " . " << EIGEN_MINOR_VERSION << endl ;
 

    float A, B, C, D, E, F, G, H, I, K;

    A = 0.1; 
    B = 0.1; 

    C = 0.2; 
    D = 0.2; 
    E = 0.2; 
    F = 0.2; 
    G = 0.2; 
    H = 0.2; 

    I = 0.3; 
    K = 0.3;

    
    Matrix<float, 3, 3> Fp;
    Fp(0,0) = 1;
    Fp(0,1) = 0;
    Fp(0,2) = A;
    Fp(1,0) = 0;
    Fp(1,1) = 1;
    Fp(1,2) = B;
    Fp(2,0) = 0;
    Fp(2,1) = 0;
    Fp(2,2) = 1;

    Matrix<float, 3, 2> Fd;
    Fd(0,0) = C;
    Fd(0,1) = D;
    Fd(1,0) = E;
    Fd(1,1) = F;
    Fd(2,0) = G;
    Fd(2,1) = H;

    Matrix<float, 2, 2> Ed;
    Ed(0,0) = I;
    Ed(0,1) = 0;
    Ed(1,0) = 0;
    Ed(1,1) = K;

    Matrix<float, 3, 3> Ep =  MatrixXf::Zero(3, 3);
    Ep =  MatrixXf::Identity(3, 3);
    std::cout << "Ep:\n" << Ep << std::endl;
    std::cout << "0.1*Ep:\n" << 0.1*Ep << std::endl;
    
    //Matrix<float, 3, 3> Epp;
    //Epp = Fp * Ep * Fp.transpose() + Fd * Ed * Fd.transpose();

    /*
    std::cout << "Fp:\n" << Fp << std::endl;
    std::cout << "Fp':\n" << Fp.transpose() << std::endl;
    std::cout << "Ep:\n" << Ep << std::endl;
    std::cout << "Fp * Ep:\n" << (Fp * Ep) << std::endl;
    std::cout << "Fp * Ep * Fp.transpose():\n" << (Fp * Ep * Fp.transpose()) << std::endl;

    std::cout << "Fd:\n"                       << Fd << std::endl;
    std::cout << "Fd':\n"                      << Fd.transpose() << std::endl;
    std::cout << "Ed:\n"                       << Ed << std::endl;
    std::cout << "Fd * Ed:\n"                  << (Fd * Ed) << std::endl;
    std::cout << "Fd * Ed * Fd.transpose():\n" << (Fd * Ed * Fd.transpose()) << std::endl;
    
    std::cout << "Epp:\n"                       << Epp << std::endl; 
    */

    return 0;
}