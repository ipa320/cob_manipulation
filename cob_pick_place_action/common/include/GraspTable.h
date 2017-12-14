/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 

#ifndef __GRASP_TABLE_H__
#define __GRASP_TABLE_H__

#include <tinyxml.h>
#include <iostream>
#include <vector>

#define MAX_NO_OF_OBJECTS 200


class Grasp
{
  public:
    Grasp(){;}

    std::vector<double> GetTCPPreGraspPose(){return m_TCPPreGraspPose;}
    void SetTCPPreGraspPose(std::vector<double> TCPPreGraspPose){m_TCPPreGraspPose = TCPPreGraspPose;}

    std::vector<double> GetTCPGraspPose(){return m_TCPGraspPose;}
    void SetTCPGraspPose(std::vector<double> TCPGraspPose){m_TCPGraspPose = TCPGraspPose;}

    /////Set and Get Hand Configurations
    std::vector<double>  GetHandPreGraspConfig(){return m_HandPreGraspConfig;}
    void SetHandPreGraspConfig(std::vector<double>  HandPreGraspConfig){m_HandPreGraspConfig=HandPreGraspConfig;}

    std::vector<double>  GetHandGraspConfig(){return m_HandGraspConfig;}
    void SetHandGraspConfig(std::vector<double>  HandGraspConfig){m_HandGraspConfig=HandGraspConfig;}

    std::vector<double>  GetHandOptimalGraspConfig(){return m_HandOptimalGraspConfig;}
    void SetHandOptimalGraspConfig(std::vector<double>  HandOptimalGraspConfig){m_HandOptimalGraspConfig=HandOptimalGraspConfig;}

    void SetGraspId(int graspId){m_GraspId = graspId;}
    int GetGraspId(){return m_GraspId;}

  private:
    std::vector<double> m_TCPGraspPose;
    std::vector<double> m_TCPPreGraspPose;
    std::vector<double> m_HandPreGraspConfig;
    std::vector<double> m_HandGraspConfig;
    std::vector<double> m_HandOptimalGraspConfig;

    int m_GraspId;
};

class GraspTableObject
{
  public:
    GraspTableObject():m_GraspReadPtr(0),m_GraspWritePtr(0),m_ObjectClassId(0)
      {m_GraspTableObject.clear();}

    int Init(int size)
      {m_GraspTableObject.resize(size);return 0;}


    ///set and get mehtods
    std::vector<Grasp*>& Get()
      {return m_GraspTableObject;}

    Grasp * GetNextGrasp()
    {
      if (m_GraspReadPtr <m_GraspWritePtr)
        return m_GraspTableObject[m_GraspReadPtr++];
      else
        return NULL;
    }

    Grasp * GetGrasp(unsigned int graspId)
    {
      if (graspId < m_GraspTableObject.size())
        return m_GraspTableObject[graspId];
      else
        return NULL;
    }

    void AddGrasp(Grasp *  grasp)
      {  if (m_GraspWritePtr < m_GraspTableObject.size()) m_GraspTableObject[m_GraspWritePtr++]=grasp;  }

    unsigned int GetObjectClassId(){  return m_ObjectClassId;  }
    void SetObjectClassId(unsigned int ObjectClassId){  m_ObjectClassId = ObjectClassId;  }

    void ResetGraspReadPtr(){  m_GraspReadPtr=0;  }

  private:
    unsigned int m_GraspReadPtr;
    unsigned int m_GraspWritePtr;
    unsigned int m_ObjectClassId;
    std::vector<Grasp*> m_GraspTableObject;
};


class GraspTable
{
  public:
    GraspTable(){;}

    int Init(char * iniFile, unsigned int table_size=MAX_NO_OF_OBJECTS);
    void AddGraspTableObject(GraspTableObject * graspTableObject);

    Grasp * GetNextGrasp(unsigned int object_class_id);
    Grasp * GetGrasp(unsigned int object_class_id, unsigned int & grasp_id);
    void ResetReadPtr(unsigned int object_class_id);

  private:
    void ReadDoubleValue(TiXmlElement* xml, const char * tag, double * value);
    void ReadJoint(TiXmlElement* xml, const char * tag, std::vector<double> & values);
    void ReadPose(TiXmlElement* xml, const char * tag, std::vector<double> & values);
    int ReadFromFile(const char * filename, GraspTableObject * tableObject);

    std::vector<GraspTableObject*> m_GraspTable;
    unsigned int m_lastObjectClassId;
};

#endif
