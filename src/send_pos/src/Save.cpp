   
#include <cmath>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <eigen3/Eigen/Dense>


using namespace std;
using namespace Eigen;

//void saveData(string fileName, Eigen::MatrixXd  matrix);

//vector<vector<double>> CSVtoVectorVectorDouble(int T);

int main(int argc, char **argv)
{
    //vector<vector<double>> traj_cart = CSVtoVectorVectorDouble(500);
    //int Taille = traj_cart.size();
    //double* pt = &traj_cart[0][0];
    //Eigen::Map<Eigen::MatrixXd> CSVMatrix(pt, Taille);
}
/* 

vector<vector<double>> CSVtoVectorVectorDouble(int Taille)
{
    string fname = "/home/bonato/catkin_ws/src/send_pos/src/trajectory.csv";
    
    vector<vector<string>> content;
    vector<vector<double>> Traj;

    vector<string> row;
    string line, word;

    fstream file (fname, ios::in);
    if(file.is_open())
    {
        while(getline(file, line))
        {
            row.clear();
            
            stringstream str(line);
            while(getline(str, word, ','))
                row.push_back(word);
            content.push_back(row);
        }
        //ROS_INFO("file well readed");

    }
    else
        //ROS_ERROR("Could not open the file\n");

    for(int i=0;i<Taille;i++) //int(content.size())
    {
        string::size_type sz;     // alias of size_t
        double pos_x = stod(content[i][0],&sz);
        double pos_y = stod(content[i][1],&sz);
        double pos_z = stod(content[i][2],&sz);
        double quad_x = stod(content[i][3],&sz);
        double quad_y = stod(content[i][4],&sz);
        double quad_z = stod(content[i][5],&sz);
        double quad_w = stod(content[i][6],&sz);

        vector<double> Line = {pos_x,pos_y,pos_z,quad_x,quad_y,quad_z,quad_w};//, Pos_y, Pos_z;
        Traj.push_back(Line);
    }
    return Traj;
}
void saveData(string fileName, Eigen::MatrixXd  matrix)
{
    //https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
    const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", "\n");
 
    ofstream file(fileName);
    if (file.is_open())
    {
        file << matrix.format(CSVFormat);
        file.close();
    }
}
 */