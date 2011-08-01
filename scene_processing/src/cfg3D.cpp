/* 
 * File:   main.cpp
 * Author: abhishek
 *
 * Created on July 26, 2011, 7:41 PM
 */

#include <iostream>
#include <vector>
#include<typeinfo>
#include<pcl/point_types.h>
#include<pcl/features/normal_3d.h>
#include<pcl/sample_consensus/sac_model_plane.h>
#include<queue>
#include<pcl/io/pcd_io.h>
#include<pcl/io/io.h>

//sac_model_plane.h
using namespace std;
typedef pcl::PointXYZRGB PointT;

/*
 * 
 */
pcl::PointCloud<PointT> scene;

class Symbol
{
protected:
    /** total weight to derive all these leaves
     * =max(cost  deriving any child) + cost of applying the rule that gave this
     *  required for being a superior CFG 
     */
    double cost; 
public:
    virtual void insertPoints(vector<int> & points)=0;
    bool operator < (const Symbol &  rhs)
    {
        return cost <rhs.cost;
    }

    double getCost() const {
        return cost;
    }
};

class Terminal : public Symbol
{
protected:
    int index;
public:
    void insertPoints(vector<int> & points)
    {
        points.push_back(index);
    }
    
    Terminal()
    {
        cost=0;
    }

    Terminal(int index_)
    {
        index=index_;
        cost=0;
    }

    Terminal(int index_,double cost_)
    {
        index=index_;
        cost=cost_;
    }
    
    int getIndex() const
    {
        return index;
    }

};

class NonTerminal : public Symbol
{
protected:
    vector<Symbol*> children;
    
    /** indices into the pointcloud
     */
    vector<int> pointIndices; // can be replaced with any sufficient statistic
    //will be populated only when extracted as min
    
    /** 
     * compute leaves by concatenating 
     * leaves of children */
public:
    void computePointIndices()
    {
        for(size_t i=0;i<children.size();i++)
        {
            children[i]->insertPoints(pointIndices);
        }
    }

    void insertPoints(vector<int> & points)
    {
        assert(pointIndices.size()>0);
        points.insert(points.end(),pointIndices.begin(),pointIndices.end());
    }    

};

class Rule
{    
public:
    /** eg for A->BCD return 3
     */
    virtual int get_Nof_RHS_symbols()=0;
    
    /**
     * this function will be used to check applicability of a rule
     * @param RHS_symbolIndex 0 based index of the RHS symbol
     * @return typename of this symbol
     */
    virtual void  get_typenames(vector<string> & names)=0;
    
    /**
     *  applies this rule on the given params(RHS of rule)
     * computes the cos of resulting NT
     */
    virtual NonTerminal* applyRule(vector<Symbol*> & RHS)=0;
    
    bool checkApplicabilty(vector<Symbol*> & RHS)
    {
        vector<string> typenames;
        get_typenames(typenames);
        for(int i=0;i<get_Nof_RHS_symbols();i++)
        {
            if(typeid(RHS.at(i)).name()!=typenames.at(i))
                return false;
        }
        return true;
    }
    
    virtual void combineAndPush(Symbol * sym,vector<Symbol *> pqueue)=0;
};

class Plane : public NonTerminal
{
protected:
    friend class RPlane_PlanePoint;
    Eigen::Vector4f planeParams;
    float curvature;
    bool planeParamsComputed;
public:
    Plane()
    {
        planeParamsComputed=false;
    }
    
    void computePlaneParams()
    {
        pcl::NormalEstimation<PointT,PointT> normalEstimator;
        normalEstimator.computePointNormal(scene, pointIndices, planeParams, curvature);
        planeParamsComputed=true;
    }
    
    int getNumPoints()
    {
        return pointIndices.size();
    }
    
    double costOfAddingPoint(PointT p)
    {
        if(pointIndices.size()<3)
            return 0;
        else
        {
            assert(planeParamsComputed);
            return exp(pcl::pointToPlaneDistance<PointT>(p,planeParams))-1;
        }
    }
};

class RPlane_PlanePoint : public Rule
{
public:
    int get_Nof_RHS_symbols()
    {
        return 2;
    }
    
    void  get_typenames(vector<string> & names)
    {
        names.push_back(typeid(Plane *).name());
        names.push_back(typeid(Terminal *).name());
    }
    
    NonTerminal* applyRule(vector<Symbol*> & RHS)
    {
        Plane * LHS=new Plane();
        Plane * RHS_plane=dynamic_cast<Plane *>(RHS.at(0));
        Terminal * RHS_point=dynamic_cast<Terminal *>(RHS.at(1));
        LHS->cost=RHS_plane->costOfAddingPoint(scene.points[RHS_point->getIndex()]);
        LHS->children.push_back(RHS_plane);
        LHS->children.push_back(RHS_point);
        LHS->computePointIndices();
        LHS->computePlaneParams();  
        return LHS;
    }
    
    void combineAndPush(Symbol * sym,vector<Symbol *> pqueue)
    {
        
        if(typeid(sym)==typeid(Terminal*))
        {
            //find nearest plane and add
        }
        else if (typeid(sym)==typeid(Plane *))
        {
            
        }
        else
            assert(1==2);
    }

    
};

class SymbolComparison
{
public:
  bool operator() (Symbol * & lhs, Symbol * & rhs) const
  {
      return lhs->getCost()>rhs->getCost();
  }
};

void runParse()
{
    priority_queue<Symbol *,vector<Symbol *>,SymbolComparison> pq;
    int numPoints=scene.size();
    for(int i=0;i<numPoints;i++)
    {
        pq.push(new Terminal(i));
    }
    
    Symbol *min;
    while(true)
    {
        min=pq.top();
        
        
        pq.pop();
        
    }
}

int main(int argc, char** argv) 
{
    pcl::io::loadPCDFile<PointT>("/home/abhishek/fridge.pcd", scene);
    runParse();
    return 0;
}

