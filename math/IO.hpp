#ifndef _IO_HPP_
#define _IO_HPP_

// clang-format off
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <stack>
#include <string>
#include <vector>

#include "DataStruct.hpp"
// clang-format on

using namespace std;
using namespace Eigen;

/*
 * split string and return it's parts
 */
vector<string> SlpitString(const string &str, const char splitCh = ' ') {
  vector<string> splittedWords;
  string word;
  for (auto ch : str) {
    if (ch != splitCh) {
      word += ch;
    } else {
      if (!word.empty()) {
        splittedWords.push_back(word);
        word.clear();
      }
    }
  }
  if (!word.empty()) {
    splittedWords.push_back(word);
    word.clear();
  }

  return splittedWords;
}

vector<string> ReadFileWithLines(const string &filePath) {
  vector<string> lines;
  string line;
  ifstream myfile(filePath);

  if (myfile.is_open()) {
    while (getline(myfile, line)) {
      lines.push_back(line);
    }
    myfile.close();
  } else {
    cout << "Unable to open file";
  }

  return lines;
}

/*
 * Split first word of a line ,
 * and return the word
 */
string SplitFirstWordInLine(string &line) {
  auto itor = line.begin();
  string prefixWord;

  string::iterator lastStart;

  for (; itor != line.end(); ++itor) {
    if (*itor != ' ') {
      // Get word
      for (; itor != line.end(); ++itor) {
        if (*itor != ' ') {
          prefixWord.push_back(*itor);
        } else {
          break;
        }
      }

      // Get last Start itor
      for (; itor != line.end(); ++itor) {
        if (*itor != ' ') {
          lastStart = itor;
          break;
        }
      }
      break;
    }
  }

  string lineLast;
  for (; lastStart != line.end(); ++lastStart) {
    lineLast.push_back(*lastStart);
  }
  line = lineLast;

  return prefixWord;
}

/*
 *Read obj model file
 *Only support Triangle faces
 */
Geometry *ReadOBJ(const string &filePath) {
  auto lines = ReadFileWithLines(filePath);
  vector<string> processedLines;
  // Uncomment and delete empty lines
  for (auto &line : lines) {
    if (line.size() > 0) {
      for (auto ch : line) {
        if (ch != ' ') {
          if (ch != '#') {
            processedLines.push_back(line);
          }
          break;
        }
      }
    }
  }

  auto geo = new Geometry;
  vector<Vector3f> normals;
  vector<Vector2f> uvs;

  // Build new geo
  for (auto &line : lines) {
    auto prefix = SplitFirstWordInLine(line);
    // cout << prefix << endl;
    // cout << line << endl;

    if (prefix == string("v")) {
      auto words = SlpitString(line);
      Vector3f pos = {stof(words[0]), stof(words[1]), stof(words[2])};
      geo->AddPoint(pos);
    } else if (prefix == string("vn")) {
      auto words = SlpitString(line);
      Vector3f normal = {stof(words[0]), stof(words[1]), stof(words[2])};
      normals.push_back(normal);
    } else if (prefix == string("vt")) {
      auto words = SlpitString(line);
      Vector2f uv = {stof(words[0]), stof(words[1])};
      uvs.push_back(uv);
    }
    // faces
    else if (prefix == string("f")) {
      auto words = SlpitString(line);
      vector<int> pointInds;
      vector<int> uvInds;
      vector<int> normalInds;

      for (auto word : words) {
        auto splitted = SlpitString(word, '/');
        pointInds.push_back(stoi(splitted[0]) - 1);
        uvInds.push_back(stoi(splitted[1]) - 1);
        normalInds.push_back(stoi(splitted[2]) - 1);
      }

      Triangle *newTria =
          new Triangle(geo->GetPoint(pointInds[0]), geo->GetPoint(pointInds[1]),
                       geo->GetPoint(pointInds[2]));

      newTria->GetVertex(0)->attributes.uv = uvs[uvInds[0]];
      newTria->GetVertex(0)->attributes.normal = normals[normalInds[0]];

      newTria->GetVertex(1)->attributes.uv = uvs[uvInds[1]];
      newTria->GetVertex(1)->attributes.normal = normals[normalInds[1]];

      newTria->GetVertex(2)->attributes.uv = uvs[uvInds[2]];
      newTria->GetVertex(2)->attributes.normal = normals[normalInds[2]];

      geo->AddPrimitive(newTria);
    }
  }

  return geo;
}

#endif // !_IO_HPP_
