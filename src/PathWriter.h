/*
 * PathWriter.h
 *
 *  Created on: 28.07.2017
 */

#ifndef SRC_PATHWRITER_H_
#define SRC_PATHWRITER_H_

#include <fstream>
#include <vector>

class PathWriter
{
 public:
  void WritePathToFile(std::string pathFileName)
  {
      std::ofstream dataFile;
      dataFile.open(pathFileName, std::ios::app);

        writeSinglePath(_path_X, "X", dataFile);
        writeSinglePath(_path_Y, "Y", dataFile);
        writeSinglePath(_path_S, "S", dataFile);
        writeSinglePath(_path_D, "D", dataFile);

      dataFile.close();
  }

  void AddNewPointToPath(double x, double y, double s, double d )
  {
    _path_X.push_back(x);
    _path_Y.push_back(y);
    _path_S.push_back(s);
    _path_D.push_back(d);
  }

 private:
  //To add values separately 4 vectors are used
  std::vector<double> _path_X;
  std::vector<double> _path_Y;
  std::vector<double> _path_S;
  std::vector<double> _path_D;

  void writeSinglePath(std::vector<double> &path, std::string label, std::ofstream& file)
  {
    file << label << ": ";
    for(size_t i=0; i < path.size()-1; ++i)
    {
      file << path.at(i) << ", ";
    }
    file << path.back() << std::endl;
  }
};



#endif /* SRC_PATHWRITER_H_ */
