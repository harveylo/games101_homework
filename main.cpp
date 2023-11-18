#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

const double PI = 3.1415926535897932384626433;

int main(){

    // Basic Example of cpp
    std::cout << "Example of cpp \n";
    float a = 1.0, b = 2.0;
    std::cout << a << std::endl;
    std::cout << a/b << std::endl;
    std::cout << std::sqrt(b) << std::endl;
    std::cout << std::acos(-1) << std::endl;
    std::cout << std::sin(30.0/180.0*std::acos(-1)) << std::endl;

    // Example of vector
    std::cout << "Example of vector \n";
    // vector definition
    Eigen::Vector3f v(1.0f,2.0f,3.0f);
    Eigen::Vector3f w(1.0f,3.0f,4.0f);
    // vector output
    std::cout << "Example of output \n";
    std::cout<< "vector v: \n"<<std::endl;
    std::cout << v << std::endl;
    std::cout<< "vector w: \n"<<std::endl;
    std::cout << w << std::endl;
    // vector add
    std::cout << "Example of add \n";
    std::cout << v + w << std::endl;
    // vector scalar multiply
    std::cout << "Example of scalar multiply \n";
    std::cout << v * 3.0f << std::endl;
    std::cout << 2.0f * v << std::endl;
    // vector dot mutiply
    auto result = v.dot(w);
    std::cout<< "Example of dot multiply \n";
    std::cout << result << std::endl;

    // Example of matrix
    std::cout << "Example of matrix \n";
    // matrix definition
    Eigen::Matrix3f i,j;
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // matrix output
    std::cout << "Example of Matrix output \n";
    std::cout<<"matrix i: \n"<<std::endl;
    std::cout << i << std::endl;
    std::cout<<"matrix j: \n"<<std::endl;
    std::cout << j << std::endl;
    // matrix add i + j
    std::cout << "Example of Matrix add, i+j is: \n";
    std::cout << i + j << std::endl;
    // matrix scalar multiply i * 2.0
    std::cout << "Example of Matrix scalar multiply, i*2.0 is: \n";
    std::cout << i * 2.0 << std::endl;
    // matrix multiply i * j
    std::cout << "Example of Matrix multiply, i*j is: \n";
    std::cout << i * j << std::endl;
    // matrix multiply vector i * v
    std::cout << "Example of Matrix multiply vector, i*v is: \n";
    std::cout << i * v << std::endl;


    /* 
    * PA 0
    */
    // TO DO: Define point P
    Eigen::Vector3f P(2.0f, 1.0f, 1.0f);
    // TO DO: Define rotation matrix M
    Eigen::Matrix3f M;
    M<<std::cos(45.0/180.0*PI), -std::sin(45.0/180.0*PI), 1,
        std::sin(45.0/180.0*PI), std::cos(45.0/180.0*PI), 2,
        0, 0, 1;
    // TO DO: M * P
    std::cout << "The result of rotation M*P is"<<std::endl;
    std::cout << M * P << std::endl;
    return 0;
}