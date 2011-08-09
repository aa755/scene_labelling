/* 
 * File:   main.cpp
 * Author: abhishek
 *
 * Created on July 26, 2011, 7:41 PM
 */

#include <iostream>
//class NonTerminal;
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
/**
 * does in-place set intersection ...O(n) amortized... result is in set_1
 * @param set_1
 * @param set_2
 */
void setIntersection(std::set<int> set_1, std::set<int> set_2)
{
    std::set<int>::iterator it1 = set_1.begin();
    std::set<int>::iterator it2 = set_2.begin();
    while ((it1 != set_1.end()) && (it2 != set_2.end()))
    {
        if (*it1 < *it2)
        {
            set_1.erase(it1++);
        }
        else if (*it2 < *it1)
        {
            ++it2;
        }
        else
        { // *it1 == *it2
            ++it1;
            ++it2;
        }
    };
}

class NonTerminal;
pcl::PointCloud<PointT> scene;
class Symbol
{
protected:
    /** total weight to derive all these leaves
     * =max(cost  deriving any child) + cost of applying the rule that gave this
     *  required for being a superior CFG 
     */
    double cost; 
//    vector<NonTerminal*> parents;
public:
    virtual void insertPoints(vector<int> & points)=0;
    bool operator < (const Symbol &  rhs)
    {
        return cost <rhs.cost;
    }

    double getCost() const {
        return cost;
    }
    
    virtual void getCentroid(pcl::PointXYZ & centroid)=0;
    
    virtual void finalize(vector<set<NonTerminal*> > & ancestors)=0;
    
//    bool checkDuplicate(vector<set<NonTerminal*> > & ancestors)=0;
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
    
    void getCentroid(pcl::PointXYZ & centroid)
    {
        PointT point=scene.points[index];
        centroid.x=point.x;
        centroid.y=point.y;
        centroid.z=point.z;        
    }

    virtual void finalize(vector<set<NonTerminal*> > & ancestors){}
    
    bool checkDuplicate(vector<set<NonTerminal*> > & ancestors)
    {
        return false; // each terminal can be derived in only 1 way
        /* contrast it with a set of terminals which can be derived in multiple way
         * (1 U 2) U 3  or  1 U (2 U 3 )
         */
    }
};
Terminal * terminals;

class NonTerminal : public Symbol
{
protected:
    vector<Symbol*> children;
    pcl::PointXYZ centroid;
    /** indices into the pointcloud
     */
    vector<int> pointIndices; // can be replaced with any sufficient statistic
    //will be populated only when extracted as min
    void computeCentroid()
    {
        PointT point;
        centroid.x=0;
        centroid.y=0;
        centroid.z=0;
        for (size_t i = 0; i < pointIndices.size(); i++)
        {
            point = scene.points[i];
            centroid.x += point.x;
            centroid.y += point.y;
            centroid.z += point.z;
        }
            centroid.x /=pointIndices.size();
            centroid.y /=pointIndices.size();
            centroid.z /=pointIndices.size();                
    }
    
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
    
    /**
     * do the book-keeping work:
     * 1) compute the set of points spanned by this NT
     * 2) add itself to the list of ancestors of all those points
     * @param ancestors: list of ancestor-sets for all poinst in scene 
     */
    virtual void finalize(vector<set<NonTerminal*> > & ancestors)
    {
        computePointIndices();
        std::pair< set<NonTerminal*>::iterator , bool > resIns;
        
        for(size_t i=0;i<pointIndices.size();i++)
        {
            resIns=ancestors[i].insert(this);
            assert(resIns.second);
        }   
                                
        computeCentroid();
        additionalFinalize();
    }
    
    void getCentroid(pcl::PointXYZ & centroid1)
    {
        centroid1=centroid;
    }
    
    virtual void additionalFinalize()
    {
        
    }
    
    bool checkDuplicate(vector<set<NonTerminal*> > & ancestors)
    {
        
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
    
    void additionalFinalize()
    {
        computePlaneParams();
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
//        RHS_plane->parents->push_back(LHS);
//        RHS_point->parents->push_back(LHS);
        // not required ... these will be needed only when this is extracted ... it cannot be combined with others
//        LHS->computePointIndices();
//        LHS->computePlaneParams();  
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
    vector<set<NonTerminal*> > ancestors(numPoints,set<NonTerminal*>());
    
    for(int i=0;i<numPoints;i++)
    {
        pq.push(new Terminal(i));
    }
    
    Symbol *min;
    while(true)
    {
        min=pq.top();
        min->finalize(ancestors);
        
        
        
        pq.pop();
        
    }
}

int main(int argc, char** argv) 
{
    pcl::io::loadPCDFile<PointT>("/home/abhishek/fridge.pcd", scene);
    runParse();
    return 0;
}
