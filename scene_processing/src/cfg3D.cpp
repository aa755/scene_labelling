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
#include <pcl/kdtree/kdtree_flann.h>

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
template<typename T>
void setIntersection(std::set<T> & set_1, std::set<T> & set_2)
{
    typename set<T>::iterator it1 = set_1.begin();
    typename set<T>::iterator it2 = set_2.begin();
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
    }
    
    set_1.erase(it1, set_1.end());
}

template<typename T>
void setDiffernce(std::set<T> & set_1, std::set<T> & set_2)
{
    typename set<T>::iterator it1 = set_1.begin();
    typename set<T>::iterator it2 = set_2.begin();
    while ((it1 != set_1.end()) && (it2 != set_2.end()))
    {
        if (*it1 < *it2)
        {
            //set_1.erase(it1++);
        }
        else if (*it2 < *it1)
        {
            ++it2;
        }
        else
        { // *it1 == *it2
            set_1.erase(it1++);
            ++it2;
        }
    }
    
    set_1.erase(it1, set_1.end());
}
struct null_deleter
{
    void operator()(void const *) const
    {
    }
};


template<typename X>
boost::shared_ptr<X> createStaticShared(X * x)
{
    boost::shared_ptr<X> px(x, null_deleter());
    return px;
}

class NonTerminal;
pcl::PointCloud<PointT> scene;
pcl::PointCloud<PointT>::Ptr scene_ptr;
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
    
    virtual bool finalize_if_not_duplicate(vector<set<NonTerminal*> > & ancestors)=0;
    
    virtual void getComplementPointSet(vector<int> & indices /* = 0 */)=0;
    virtual void getSetOfAncestors(set<NonTerminal*> & thisAncestors , vector<set<NonTerminal*> > & allAncestors)=0;

    void getValidSymbolsForCombination(vector<set<NonTerminal*> > & allAncestors, set<Symbol*> combineCanditates)
    {
        pcl::KdTreeFLANN<PointT> nnFinder;
        vector<int> indices;
        getComplementPointSet(indices);
//        pcl::PointCloud<PointT>::Ptr scene_ptr=new pcl::PointCloud<PointT>::Ptr(scene)
        nnFinder.setInputCloud(scene_ptr,createStaticShared<vector<int> >(&indices));
        vector<int> nearest_index;
        vector<float> nearest_distances;
        pcl::PointXYZ centroidt;
        getCentroid(centroidt);
        PointT centroidTT;
        centroidTT.x=centroidt.x;
        centroidTT.y=centroidt.y;
        centroidTT.z=centroidt.z;
        nnFinder.nearestKSearch(centroidTT,1,nearest_index,nearest_distances);
        combineCanditates.clear();
        combineCanditates.insert(allAncestors[nearest_index[0]].begin(),allAncestors[nearest_index[0]].end());
        set<NonTerminal*> thisAncestors;
        getSetOfAncestors(thisAncestors,allAncestors);
        setDiffernce<NonTerminal*>(combineCanditates,thisAncestors);
        
    }
    
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

    virtual bool finalize_if_not_duplicate(vector<set<NonTerminal*> > & ancestors){return true;}
    
    //bool checkDuplicate(vector<set<NonTerminal*> > & ancestors)    {
//        return false; // each terminal can be derived in only 1 way
        /* contrast it with a set of terminals which can be derived in multiple way
         * (1 U 2) U 3  or  1 U (2 U 3 )
         */
   // }
    void getComplementPointSet(vector<int>& indices /* = 0 */)
    {
        // will be called only once ... when this terminal is extracted
        indices.clear();
        for(int i=0;i<indices.size();i++)
        {
            if(i!=index)
                indices.push_back(i);
        }
    }
    
    void getSetOfAncestors(set<NonTerminal*> & thisAncestors , vector<set<NonTerminal*> > & allAncestors)
    {
        thisAncestors=allAncestors[index];
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
     * @param ancestors: list of ancestor-sets for all points in scene 
     */
    virtual bool finalize_if_not_duplicate(vector<set<NonTerminal*> > & ancestors)
    {
        computePointIndices();
        if(checkDuplicate(ancestors))
            return false;
        std::pair< set<NonTerminal*>::iterator , bool > resIns;
        
        for(size_t i=0;i<pointIndices.size();i++)
        {
            resIns=ancestors[i].insert(this);
            assert(resIns.second);
        }   
                                
        computeCentroid();
        additionalFinalize();
        return true;
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
        set<NonTerminal*> intersect=ancestors[pointIndices[0]];
        for(size_t i=1;i<pointIndices.size();i++)
        {
            setIntersection<NonTerminal *>(intersect,ancestors[pointIndices[i]]);
        }
        if(intersect.size()==0)
            return false;
        else
        {
            set<NonTerminal*>::iterator it;
            for ( it=intersect.begin() ; it != intersect.end(); it++ )
            {
                if(typeid(this)==typeid(*it) && pointIndices.size()==(*it)->pointIndices.size())
                    return true;
            }
            return false;
        }
    }

    void getComplementPointSet(vector<int>& indices /* = 0 */)
    {
        int it1=0;
        int numPoints=scene.size();
        indices.clear();
        sort(pointIndices.begin(),pointIndices.end());
        vector<int>::iterator it2=pointIndices.begin();
        while ((it1 < numPoints ) && (it2 != pointIndices.end()))
        {
            if (it1 < *it2)
            {
                indices.push_back(it1);
                it1++;
            }
            else if (it1==*it2)
            { 
                ++it1; //skip this point
                ++it2;
            }
            else
            {
                assert(1==2);
            }

        
        }
    }
    
    void getSetOfAncestors(set<NonTerminal*> & thisAncestors , vector<set<NonTerminal*> > & allAncestors)
    {
        thisAncestors=allAncestors[pointIndices[0]];
        for(size_t i=1;i<allAncestors.size();i++)
        {
             set<NonTerminal*> & temp=allAncestors[pointIndices[i]];
             thisAncestors.insert(temp.begin(),temp.end());
        }
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
        if(min->finalize_if_not_duplicate(ancestors))
        {
            
        }
        else
        {
            delete min; 
            // since there are no parent links, and it was not a child of anyone
            // deleting does not cause dangling pointers
            
        }
        
        
        
        pq.pop();
        
    }
}

int main(int argc, char** argv) 
{
    pcl::io::loadPCDFile<PointT>("/home/abhishek/fridge.pcd", scene);
    *scene_ptr=scene;
    runParse();
    return 0;
}
