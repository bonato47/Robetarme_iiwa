#include <cmath>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>

using namespace std;

vector<vector<double>> CSVtoVectorVectorDouble();

int main(){

    vector<vector<double>> a ;
    a = CSVtoVectorVectorDouble();
    cout << a[23][2];
    return 0;
}

vector<vector<double>> CSVtoVectorVectorDouble()
{
    string fname = "trajectory.csv";
    
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
    }
    else
        cout<<"Could not open the file\n";

    for(int i=0;i<int(content.size());i++)
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