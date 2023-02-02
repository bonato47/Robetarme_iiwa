//#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "vector"

using namespace Eigen;
using namespace std;

void meshgrid(const ArrayXd & x, const ArrayXd & y, ArrayXXd &xx, ArrayXXd &yy);
ArrayXd linspace(double start_in, double end_in, int num_in);

//Define work space
int main()
{

    ArrayXd x_limits(2,1) ;
    x_limits <<  -0.6, 0.6 ;
    ArrayXd y_limits(2,1) ; ;
    y_limits <<  -0.6, 0.6 ;
    ArrayXd z_limits(2,1) ; ;
    z_limits <<   0, 1.2 ;
    int nb_gridpoints = 30 ;
    ArrayXd x(nb_gridpoints+1,1) ; ;
    ArrayXd y(nb_gridpoints+1,1) ; ;
    ArrayXd z(nb_gridpoints+1,1) ; ;


    x  = linspace(-0.6, 0.6,nb_gridpoints);
    y  = linspace(-0.6, 0.6,nb_gridpoints);
    z  = linspace(0, 1,nb_gridpoints);

    ArrayXXd XX, YY;
    meshgrid(x, y, XX, YY);
    cout << "XX = \n" << XX << endl;
    cout << "YY = \n" << YY << endl;

    return 0;
}

void meshgrid(const ArrayXd & x, const ArrayXd & y, ArrayXXd &xx, ArrayXXd &yy)
{
    MatrixXd m_x = x.matrix();
    MatrixXd m_y = y.matrix();
    int Nx = m_x.rows();
    int Ny = m_y.rows();
    m_x.transposeInPlace();
    MatrixXd m_xx = m_x.replicate(Ny, 1);
    MatrixXd m_yy = m_y.replicate(1, Nx);
    xx = m_xx.array();
    yy = m_yy.array();
}

ArrayXd linspace(double start_in, double end_in, int num_in)
{
    ArrayXd linspaced(num_in+1,1);
    double step = (end_in -start_in)/ (double) num_in;
      for(double i =0 ; i <= num_in ; ++i){
        linspaced((int) i) =(start_in +step* i);
    }

    return linspaced;
}