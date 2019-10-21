
#include <ros/ros.h>

#include <iostream>
#include <string>
#include <map>
#include <vector>

#include "route_verbalization/Verbalizers/sentences.h"

void replace(std::vector<std::string>& sentences)
{
  for(auto& s : sentences)
  {
    size_t pose;
    if((pose = s.find("/DY")) != std::string::npos)
      s.replace(s.begin() + pose, s.begin() + pose + 3, "right");
    if((pose = s.find("/D")) != std::string::npos)
      s.replace(s.begin() + pose, s.begin() + pose + 2, "left");
    if((pose = s.find("/I")) != std::string::npos)
      s.replace(s.begin() + pose, s.begin() + pose + 2, "take the elevator");
    if((pose = s.find("/Y")) != std::string::npos)
      s.replace(s.begin() + pose, s.begin() + pose + 2, "\"Y\"");
    if((pose = s.find("/X")) != std::string::npos)
      s.replace(s.begin() + pose, s.begin() + pose + 2, "\"X\"");
  }
}

std::vector<std::string> generateSentences(const std::vector<sentence_t>& parts)
{
  std::vector<std::string> res;

  for(const auto& p : parts)
  {
    std::vector<std::string> tmp_i, tmp_ii;
    tmp_i.push_back("");
    for(const auto& i : p.text_)
    {
      tmp_ii.clear();
      for(const auto& j : i)
      {
        for(const auto& t : tmp_i)
          tmp_ii.push_back(t + j);
      }
      tmp_i = tmp_ii;
    }

    res.insert(res.end(), tmp_i.begin(), tmp_i.end());
  }

  replace(res);

  return res;
}

void display(const std::vector<std::string>& sentences)
{
  std::cout << std::endl << std::endl;
  for(const auto& s : sentences)
    std::cout << s << std::endl;
}

int main(int argc, char**argv)
{
  ros::init(argc, argv, "all_sentences");

  ros::NodeHandle n;

  Sentences s(nullptr);

  display(generateSentences(s.end_));
  display(generateSentences(s.end_begin_));
  display(generateSentences(s.begin_));
  display(generateSentences(s.during_));

  return 0;
}
