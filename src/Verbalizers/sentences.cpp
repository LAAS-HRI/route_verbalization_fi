#include "route_verbalization/Verbalizers/sentences.h"
#include <random>
#include <iostream>

Sentences::Sentences(OntologyManipulator* onto) : onto_(onto)
{
  createEnd();
  createEndBegin();
  createBegin();
  createDuring();
  createPunctuation();

  interface_map["elevator"] = "take the ";
  interface_map["stair"] = "take the ";
  interface_map["escalator"] = "take the ";
}

void Sentences::createEnd()
{
  {
    sentence_t tmp(end_side,
      {{",and ", ",then ", ". ", ". After that ", ". To conclude, "}, {"you will "}, {"see ", "find "},
      {"place /X "}, {"on "}, {"your ", "the "}, {"/D "},
      {"side ", ""}, {", just after place /Y ", ""}});
    end_.push_back(tmp);
  }

  {
    sentence_t tmp(end_side,
      {{",and the place /X is ", ",then ", ",the place /X is then ", ". After that, "}, {"on "}, {"the ", "your "},
      {"/D "}, {"side "}, {", just after place /Y ", ""}});
    end_.push_back(tmp);
  }

  {
    sentence_t tmp(end_side,
      {{",then ", ". After that, ", ". ", ",and ", ". To conclude, ", ". From there on, "}, {"the place /X will be on your /D "}, {"side ", ""}, {", just after place /Y ", ""}});
    end_.push_back(tmp);
  }

  {
    sentence_t tmp(end_side,
      {{",then ", ". After that, ", ",and the ", ". To conclude, ", ". From there on, "}, {"place /X is on the /D there "}, {", just after place /Y ", ""}});
    end_.push_back(tmp);
  }

  {
    sentence_t tmp(end_side,
      {{",then ", ". After that, ", ",and ", ". To conclude, "}, {"on the /D, you will see place /X "}, {", just after place /Y ", ""}});
    end_.push_back(tmp);
  }

  {
    sentence_t tmp(end_here,
      {{",then ", ". After that, ", ",and "}, {"you will see place /X "}, {", just after place /Y ", ""}});
    end_.push_back(tmp);
  }

  {
    sentence_t tmp(end_here,
      {{",then ", ". After that, ", ",and ", ". To conclude, "}, {"you will find place /X "}});
    end_.push_back(tmp);
  }

  {
    sentence_t tmp(end_here,
      {{",then ", ". After that, the ", ",and the ", ". To conclude, the ", ". From there on, "}, {"place /X is "}, {"there, ", ""}, {"on the /DY side of the place /Y "}});
    end_.push_back(tmp);
  }

  {
    sentence_t tmp(end_in_front,
      {{",then ", ". After that, ", ",and ", ". To conclude, ", ". From there on, "}, {"you will "}, {"find ", "see "}, {"place /X "}, {"right away "}});
    end_.push_back(tmp);
  }
}

void Sentences::createEndBegin()
{
  {
    sentence_t tmp(end_side,
      {{"you will "}, {"see ", "find "},
      {"it ", "place /X "}, {"on "}, {"your ", "the "}, {"/D "},
      {"side ", ""}, {", just after place /Y, ", ""}});
    end_begin_.push_back(tmp);
  }

  {
    sentence_t tmp(end_side,
      {{"it will be on your /D side "}, {", just after place /Y, ", ""}});
    end_begin_.push_back(tmp);
  }

  {
    sentence_t tmp(end_side,
      {{"it's on the /D there "}, {", just after place /Y, ", ""}});
    end_begin_.push_back(tmp);
  }

  {
    sentence_t tmp(end_side,
      {{"on the /D you will see place /X "}, {", just after place /Y, ", ""}});
    end_begin_.push_back(tmp);
  }

  {
    sentence_t tmp(end_here,
      {{"you see there place /X "}});
    end_begin_.push_back(tmp);
  }

  {
    sentence_t tmp(end_here,
      {{"you will find it there "}});
    end_begin_.push_back(tmp);
  }

  {
    sentence_t tmp(end_here,
      {{"it's on the /DY side of place /Y "}});
    end_begin_.push_back(tmp);
  }

  {
    sentence_t tmp(end_in_front,
      {{"you will "}, {"find ", "see "}, {"it ", "place /X "}, {"right away "}});
    end_begin_.push_back(tmp);
  }
}

void Sentences::createBegin()
{
  {
    sentence_t tmp(start_corridor,
      {{"just go "},
      {"across ", "on ", "down "}, {"that ", "this "}, {"corridor "}});
    begin_.push_back(tmp);
  }

  {
    sentence_t tmp(start_corridor,
      {{"go to this "}, {"corridor "}});
    begin_.push_back(tmp);
  }

  {
    sentence_t tmp(start_end_of_corridor,
      {{"place /X "}, {"is down "}, {"this ", "that "}, {"corridor "}});
    begin_.push_back(tmp);
  }

  {
    sentence_t tmp(start_end_of_corridor,
      {{"just go almost "}, {"until the end of "}, {"this ", "that "}, {"corridor "}});
    begin_.push_back(tmp);
  }

  {
    sentence_t tmp(start_end_of_corridor,
      {{"place /X "}, {"is at the end of "}, {"this ", "that "}, {"corridor "}});
    begin_.push_back(tmp);
  }

  {
    sentence_t tmp(start_interface,
      {{"/I "}});
    begin_.push_back(tmp);
  }
}

void Sentences::createDuring()
{
  {
    sentence_t tmp(during_end_of_corridor,
      {{"just walk to ", "just go almost at ", "just walk until "}, {"the "}, {"very ", ""}, {"end of the "}, {"corridor "}});
    during_.push_back(tmp);
  }

  {
    sentence_t tmp(during_turn_continu_corridor,
      {{"turn /D"}, {" just after place /Y ", ""}});
    during_.push_back(tmp);
  }

  {
    sentence_t tmp(during_turn,
      {{"turn /D "}});
    during_.push_back(tmp);
  }

  {
    sentence_t tmp(during_turn,
      {{"turn /D "}, {"just after place /Y ", ""}});
    during_.push_back(tmp);
  }

  {
    sentence_t tmp(during_reference,
      {{"you will "}, {"see ", "find "}, {"place /Y "}});
    during_.push_back(tmp);
  }

  {
    sentence_t tmp(during_interface_font,
      {{"/I "}, {"right away "}});
    during_.push_back(tmp);
  }

  {
    sentence_t tmp(during_interface_side,
      {{"/I "}, {"at "}, {"the ", "your "}, {"/D "}, {"side "}});
    during_.push_back(tmp);
  }
}

void Sentences::createPunctuation()
{
  {
    sentence_t tmp(punctuation,
      {{", ", ", and then, ", ", then ", ". Then, ", ". "}});
    punctuation_.push_back(tmp);
  }
}

std::string Sentences::getPunctuation()
{
  return selectOne(punctuation_);
}

std::string Sentences::createInterfaceSentence(std::string word)
{
  std::vector<std::string> interfaces = onto_->individuals.getUp(word, 1);
  std::string interface_name = "";
  std::string verb = "go through the ";
  for(size_t interface = 0; interface < interfaces.size(); interface++)
      if (interface_map.find(interfaces[interface]) != interface_map.end())
      {
        verb = interface_map[interfaces[interface]];
        interface_name = interfaces[interface];
      }

  if(interface_name == "")
    interface_name = onto_->individuals.getName(word);

  return verb + interface_name;
}

std::string Sentences::getSentence(sentence_req_t req, bool begin)
{
  if((req.type_ == end_side) || (req.type_ == end_here) || (req.type_ == end_in_front))
    if(begin == false)
      return getSentence(req, end_);
    else
      return getSentence(req, end_begin_);
  else if((req.type_ == start_corridor) || (req.type_ == start_end_of_corridor) || (req.type_ == start_interface))
    return getSentence(req, begin_);
  else
    return getSentence(req, during_);
}

std::string Sentences::getSentence(sentence_req_t& req, std::vector<sentence_t>& base)
{
  std::cout << "get sentence ref = " << req.reference_ << " at " << req.refrence_side_ << std::endl;
  std::string res;
  size_t timeout = 30;
  do
  {
    std::vector<sentence_t> sentences_base;
    getSentences(sentences_base, base, req.type_);
    res = selectOne(sentences_base);
    timeout--;
  }
  while((replace(res, req) == false) && (timeout != 0));

  if(timeout == 0)
  {
    std::cout << res << std::endl;
    res = "";
  }

  return res;
}

void Sentences::getSentences(std::vector<sentence_t>& sentences, std::vector<sentence_t>& base, sentences_type type)
{
  for(size_t i = 0; i < base.size(); i++)
    if(base[i].type_ == type)
      sentences.push_back(base[i]);
}

std::string Sentences::selectOne(std::vector<sentence_t>& sentences)
{
  std::random_device rd;
  std::mt19937 gen(rd());

  std::vector<size_t> costs;
  size_t total = 0;
  for(size_t i = 0; i < sentences.size(); i++)
  {
    total += sentences[i].getNb();
    costs.push_back(total);
  }
  std::uniform_int_distribution<> global_dis(1, total);
  size_t sentence_index = global_dis(gen);
  sentence_t selected;

  for(size_t i = 0; i < sentences.size(); i++)
    if(sentence_index <= costs[i])
    {
      selected = sentences[i];
      break;
    }

  std::string res;
  for(size_t i = 0; i < selected.text_.size(); i++)
  {
    std::uniform_int_distribution<> dis(0, selected.text_[i].size() - 1);
    size_t index = dis(gen);
    res += selected.text_[i][index];
  }
  return res;
}

bool Sentences::replace(std::string& text, sentence_req_t& req)
{
  std::string left_str = "left";
  std::string right_str = "right";

  std::cout << text << std::endl;

  size_t pose = std::string::npos;
  while((pose = text.find("/DY")) != std::string::npos)
  {
    if(req.refrence_side_ != none_side)
      if(req.refrence_side_ == right)
        text.replace(text.begin() + pose, text.begin() + pose + 3, right_str);
      else
        text.replace(text.begin() + pose, text.begin() + pose + 3, left_str);
    else
      return false;
  }

  while((pose = text.find("/D")) != std::string::npos)
  {
    if(req.side_ != none_side)
      if(req.side_ == right)
        text.replace(text.begin() + pose, text.begin() + pose + 2, right_str);
      else
        text.replace(text.begin() + pose, text.begin() + pose + 2, left_str);
    else
      return false;
  }

  while((pose = text.find("/X")) != std::string::npos)
  {
    if(req.place_ != "")
    {
      std::string name = onto_->individuals.getName(req.place_);
      if(onto_->individuals.isA(req.place_, "interface"))
        name = "the " + name;

      text.replace(text.begin() + pose, text.begin() + pose + 2, name);
    }
    else
      return false;
  }

  while((pose = text.find("/Y")) != std::string::npos)
  {
    if(req.reference_ != "")
    {
      if(onto_->individuals.isA(req.reference_, "pathIntersection"))
        return false;

      std::string name = onto_->individuals.getName(req.reference_);
      if(onto_->individuals.isA(req.reference_, "interface"))
        name = "the " + name;

      text.replace(text.begin() + pose, text.begin() + pose + 2, name);
    }
    else
      return false;
  }

  while((pose = text.find("/I")) != std::string::npos)
  {
    text.replace(text.begin() + pose, text.begin() + pose + 2, createInterfaceSentence(req.place_));
  }

  return true;
}
