/* 
 * File:   main.cpp
 * Author: abhishek
 *
 * Created on July 26, 2011, 7:41 PM
 */

#include <iostream>
#include <fstream>
#include <vector>
#include<typeinfo>
#include<pcl/point_types.h>
#include "point_struct.h"
#include<pcl/features/normal_3d.h>
#include<pcl/sample_consensus/sac_model_plane.h>
#include<queue>
#include<pcl/io/pcd_io.h>
#include<pcl/io/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <stdlib.h>
#include <time.h>
#include <boost//lexical_cast.hpp>
#define BOOST_DYNAMIC_BITSET_DONT_USE_FRIENDS
#include <boost/dynamic_bitset.hpp>
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
            it1++;
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
class Terminal;

class NTSetComparison
{
public:
  bool operator() (NonTerminal * const & lhs, NonTerminal * const &  rhs);
};

typedef set<NonTerminal *,NTSetComparison> NTSet;

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
        virtual void printData()
        {
            
        }
        
    virtual void insertPoints(vector<int> & points)=0;
    
    virtual void unionMembersip(boost::dynamic_bitset<> & set_membership)=0;
    bool operator < (const Symbol &  rhs)
    {
        return cost <rhs.cost;
    }

    double getCost() const {
        return cost;
    }
    
     virtual size_t getNumPoints()=0;

    virtual void getCentroid(pcl::PointXYZ & centroid)=0;
    
    virtual bool finalize_if_not_duplicate(vector<NTSet> & planeSet)=0;
    
    virtual void getComplementPointSet(vector<int> & indices /* = 0 */)=0;
//    virtual void getSetOfAncestors(set<NonTerminal*> & thisAncestors , vector<set<NonTerminal*> > & allAncestors)=0;
    
    virtual int getId()=0;

    void getValidSymbolsForCombination(vector<Terminal*> & terminals,set<int> & nearPoints)
    {
        // find the nearest point not in this segment
        pcl::KdTreeFLANN<PointT> nnFinder;
        vector<int> indices;
        indices.reserve(scene.size()-getNumPoints()); // +1 is not required
        
        nearPoints.clear();
        
        if(scene.size()==getNumPoints()) // cannot be combined further
		return;

        getComplementPointSet(indices);
        cout<<indices.size()<<" points in complement set ... this has "<<getNumPoints()<<endl;
        nnFinder.setInputCloud(createStaticShared<pcl::PointCloud<PointT> >(&scene),createStaticShared<vector<int> >(&indices));
        vector<int> nearest_indices;
        vector<float> nearest_distances;
        
        
        vector<int> pointIndicess;
        pointIndicess.reserve(getNumPoints());
        insertPoints(pointIndicess);
        
        set<int>::iterator npit;
                nearest_indices.resize(1,0);
                nearest_distances.resize(1,0.0);
        for(size_t i=0;i<pointIndicess.size();i++)
        {
                nnFinder.nearestKSearch(scene.points[pointIndicess[i]],1,nearest_indices,nearest_distances);
                int nearest_index=indices[nearest_indices[0]];
                nearPoints.insert(nearest_index);
        }
        
//                combineNTCanditates.insert(allAncestors[minDistIndex].begin(),allAncestors[minDistIndex].end());
         cout<<nearPoints.size()<<"unique points found"<<endl;
                
    }
    
//    bool checkDuplicate(vector<set<NonTerminal*> > & ancestors)=0;
};

class SymbolComparison
{
public:
  bool operator() (Symbol * & lhs, Symbol * & rhs) const
  {
      return lhs->getCost()>rhs->getCost();
  }
};


typedef priority_queue<Symbol *,vector<Symbol *>,SymbolComparison> SymbolPriorityQueue;


class Terminal : public Symbol
{
protected:
    size_t index;
public:
    int getId()
    {
        return 0;
    }
    
    void unionMembersip(boost::dynamic_bitset<> & set_membership)
    {
        set_membership.set(index,true);
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
    
    const PointT & getPoint()
    {
        return scene.points[index];
    }
    
    void insertPoints(vector<int> & points)
    {
        points.push_back(index);
    }
    
    void getCentroid(pcl::PointXYZ & centroid)
    {
        PointT point=scene.points[index];
        centroid.x=point.x;
        centroid.y=point.y;
        centroid.z=point.z;        
    }

        virtual void printData()
        {
            cout<<"t\t:"<<index<<endl;
        }
        
    virtual bool finalize_if_not_duplicate(vector<NTSet> & planeSet){return true;}
    
    //bool checkDuplicate(vector<set<NonTerminal*> > & ancestors)    {
//        return false; // each terminal can be derived in only 1 way
        /* contrast it with a set of terminals which can be derived in multiple way
         * (1 U 2) U 3  or  1 U (2 U 3 )
         */
   // }
    void getComplementPointSet(vector<int>& indices /* = 0 */)
    {
        // will be called only once ... when this terminal is extracted
        cout<<" terminal complement of index"<<index<<"\n";
        indices.clear();
        for(size_t i=0;i<scene.size();i++)
        {
            if(i!=index)
                indices.push_back(i);
        }
    }
    
    size_t getNumPoints()
    {
        return 1;
    }
    
 /*   void getSetOfAncestors(set<NonTerminal*> & thisAncestors , vector<set<NonTerminal*> > & allAncestors)
    {
        thisAncestors=allAncestors[index];
    }
  */
};
Terminal * terminals;

class NonTerminal : public Symbol
{
protected:
    size_t numPoints;
//    vector<bool> setOfPoints;
    boost::dynamic_bitset<> set_membership;
    vector<Symbol*> children;
    pcl::PointXYZ centroid;
    int id;
    static int id_counter;
    /** indices into the pointcloud
     */
    //will be populated only when extracted as min
/*    void computeCentroid()
    {
        PointT point;
        centroid.x=0;
        centroid.y=0;
        centroid.z=0;
        for (size_t i = 0; i < pointIndices.size(); i++)
        {
            point = scene.points[pointIndices[i]];
            centroid.x += point.x;
            centroid.y += point.y;
            centroid.z += point.z;
        }
            centroid.x /=pointIndices.size();
            centroid.y /=pointIndices.size();
            centroid.z /=pointIndices.size();                
    }
  */
    
   void computePointIndices()
    {
       pointIndices.clear();
       pointIndices.reserve(getNumPoints());

       for(size_t i=0;i<scene.size();i++)
       {
           if(set_membership.test(i))
               pointIndices.push_back(i);
       }
    }
   
   void insertPoints(vector<int> & points)
    {
       points.reserve(points.size()+getNumPoints());

       for(size_t i=0;i<scene.size();i++)
       {
           if(set_membership.test(i))
               points.push_back(i);
       }
    }
   
   /** 
     * compute leaves by concatenating 
     * leaves of children */
    bool costSet;
public:
    friend class RS_PlanePlane;
    vector<int> pointIndices; // can be replaced with any sufficient statistic
    
    bool intersects(NonTerminal * other)
    {
        return set_membership.intersects(other->set_membership);
    }
        int getId()
    {
        return id;
    }

    NonTerminal()
    {
        costSet=false;
        id=id_counter++;
        cout<<"new NT: "<<id<<endl;
        numPoints=0;
    }

    friend class NTSetComparison;
    
        virtual void printData()
        {
            cout<<id<<"\t:"<<set_membership<<endl;
        }
        
    size_t getNumPoints()
    {
        assert(numPoints>0);
        return numPoints;
    }

    void setAdditionalCost(double additionalCost)
    {
        assert(additionalCost>=0);
        cost=0;
        for(size_t i=0;i<children.size();i++)
            cost+=children[i]->getCost();
        cost+=additionalCost;
        costSet=true;
        cout<< "ac: "<<additionalCost<<" tc: "<<cost<<endl;
    }
    
    void addChild(Symbol * child)
    {
        assert(!costSet);
        children.push_back(child);
    }
    void computeSetMembership()
    {
        set_membership.resize(scene.size(),false);
        for(size_t i=0;i<children.size();i++)
        {
            children[i]->unionMembersip(set_membership);
        }
        numPoints=set_membership.count();
    }

    void unionMembersip(boost::dynamic_bitset<> & set_membership)
    {
//        assert(pointIndices.size()>0);
        set_membership|=this->set_membership;
    }
    
    /**
     * do the book-keeping work:
     * 1) compute the set of points spanned by this NT
     * 2) add itself to the list of ancestors of all those points
     * @param ancestors: list of ancestor-sets for all points in scene 
     */
    bool finalize_if_not_duplicate(vector<NTSet> & planeSet)
    {
        assert(costSet); // cost must be set before adding it to pq
        if(checkDuplicate(planeSet))
            return false;
        computePointIndices();
   //     std::pair< set<NonTerminal*>::iterator , bool > resIns;
        
/*        for(size_t i=0;i<pointIndices.size();i++)
        {
            resIns=ancestors[pointIndices[i]].insert(this); // too many duplicate inserts
            assert(resIns.second);
        }   */
        
        planeSet[pointIndices.size()].insert(this);// indexed by size 
        // S wont ever be finalized , ... so it will only contain planes
                                
      //  computeCentroid();
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
    
    bool checkDuplicate(vector<NTSet> & planeSet)
    {
        assert(numPoints>0);
        NTSet & bin=planeSet[numPoints];
        NTSet::iterator lb;
        lb=bin.lower_bound(this);
        if(lb!=bin.end() && (*lb)->set_membership==set_membership)
        {
         //   cout<<"duplicate:\n"<<set_membership<<"\n"<<(*lb)->set_membership<<endl;
            return true;
        }
        else
            return false;
    }
    
    

    void getComplementPointSet(vector<int>& indices /* = 0 */)
    {
        int numSPoints=scene.size();
        indices.clear();
        indices.reserve(numSPoints-getNumPoints());
       for(size_t i=0;i<scene.size();i++)
       {
           if(!set_membership.test(i))
               indices.push_back(i);
       }
        
    }
    
/*    void getSetOfAncestors(set<NonTerminal*> & thisAncestors , vector<set<NonTerminal*> > & allAncestors)
    {
        thisAncestors=allAncestors[pointIndices[0]];
        for(size_t i=1;i<getNumPoints();i++)
        {
             set<NonTerminal*> & temp=allAncestors[pointIndices.at(i)];
             thisAncestors.insert(temp.begin(),temp.end());
        }
    }
  */  
};
  bool NTSetComparison::operator() (NonTerminal * const & lhs, NonTerminal * const &  rhs)
  {
      //start with MSBs
      for(int i=lhs->set_membership.num_blocks()-1;i>=0;i--)
      {
          if(lhs->set_membership.m_bits[i] > rhs->set_membership.m_bits[i])
              return true;
          else if(lhs->set_membership.m_bits[i]  <  rhs->set_membership.m_bits[i])
              return false;
          // else look the lower significant block
                      
      }
      return false; // actually they are equal
  }

int NonTerminal::id_counter=0;
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
            if(typeid(*(RHS.at(i))).name()!=typenames.at(i))
                return false;
        }
        return true;
    }
    
    virtual void combineAndPush(Symbol * sym, set<int> & combineCandidates , SymbolPriorityQueue & pqueue, vector<NTSet> & planeSet /* = 0 */, vector<Terminal*> & terminals /* = 0 */)=0;
    virtual void addToPqueueIfNotDuplicate(NonTerminal * newNT, vector<NTSet> & planeSet, SymbolPriorityQueue & pqueue)
    {
        newNT->computeSetMembership();
        if(!newNT->checkDuplicate(planeSet))
            pqueue.push(newNT);
    }
};

double sqr(double d)
{
    return d*d;
}


class Plane : public NonTerminal
{
protected:
//    friend class RPlane_PlanePoint;
    float curvature;
    bool planeParamsComputed;
    Eigen::Vector4f planeParams;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Plane():NonTerminal()
    {
        planeParamsComputed=false;
    }
    
    double getNorm()
    {
         return (planeParams[0]*planeParams[0]  +  planeParams[1]*planeParams[1]  +  planeParams[2]*planeParams[2]);

    }
    
    void computePlaneParams()
    {
        pcl::NormalEstimation<PointT,pcl::Normal> normalEstimator;
        normalEstimator.computePointNormal(scene, pointIndices, planeParams, curvature);
        double norm=getNorm();
        planeParams/=norm;
        planeParamsComputed=true;
    }
    
    
    double coplanarity(Plane * plane2)
    {
        return  fabs(planeParams[0]*plane2->planeParams[0]  +  planeParams[1]*plane2->planeParams[1]  +  planeParams[2]*plane2->planeParams[2]) ;
    }
    
    double costOfAddingPoint(PointT p)
    {
        if(pointIndices.size()==2)
        {
            /*return it's min distance to either points
             */
            double d1=pcl::euclideanDistance<PointT,PointT>(p,scene.points[pointIndices[0]]);
            double d2=pcl::euclideanDistance<PointT,PointT>(p,scene.points[pointIndices[1]]);
            if(d1>d2)
                return d1;
            else
                return d2;
        }
        else if(pointIndices.size()>=3)
        {
            assert(planeParamsComputed);
//            return exp(100*pcl::pointToPlaneDistance<PointT>(p,planeParams))-1;
            return getNumPoints()*(exp(pcl::pointToPlaneDistance<PointT>(p,planeParams))-1);
        }
        else
            assert(1==2);
    }
    
    void additionalFinalize()
    {
        if(pointIndices.size()>=3)
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
        names.push_back(typeid(Plane ).name());
        names.push_back(typeid(Terminal ).name());
    }
    
    NonTerminal* applyRule(vector<Symbol*> & RHS)
    {
        Plane * LHS=new Plane();
        Plane * RHS_plane=dynamic_cast<Plane *>(RHS.at(0));
        Terminal * RHS_point=dynamic_cast<Terminal *>(RHS.at(1));
        LHS->addChild(RHS_plane);
        LHS->addChild(RHS_point);
        LHS->setAdditionalCost(RHS_plane->costOfAddingPoint(RHS_point->getPoint()));
//        RHS_plane->parents->push_back(LHS);
//        RHS_point->parents->push_back(LHS);
        // not required ... these will be needed only when this is extracted ... it cannot be combined with others
//        LHS->computePointIndices();
//        LHS->computePlaneParams();  
        return LHS;
    }
    
     void combineAndPush(Symbol * sym, set<int> & combineCandidates , SymbolPriorityQueue & pqueue, vector<NTSet> & planeSet /* = 0 */, vector<Terminal*> & terminals /* = 0 */)
    {
        set<int>::iterator it;
            //all terminals have cost=0, all NT's have cost>0,
            //so when a terminal is extracted, no non-terminal(plane)
            //has been extracted yet
            //so, if sym is of type Terminal, it cannot be combined with a plane
        if (typeid(*sym)==typeid(Plane ))
        {
            for(it=combineCandidates.begin();it!=combineCandidates.end();it++)
            {
                    vector<Symbol*> temp;
                    temp.push_back(sym);
                    temp.push_back(terminals.at(*it)); // must be pushed in order
                    addToPqueueIfNotDuplicate(applyRule(temp),planeSet,pqueue);
                    cout<<"applied rule p->pt\n";
            }
            
        }
    }    
};

class RPlane_PointPoint : public Rule
{
public:
    int get_Nof_RHS_symbols()
    {
        return 2;
    }
    
    void  get_typenames(vector<string> & names)
    {
        names.push_back(typeid(Terminal ).name());
        names.push_back(typeid(Terminal ).name());
    }
    
    NonTerminal* applyRule(vector<Symbol*> & RHS)
    {
        Plane * LHS=new Plane();
        Terminal * RHS_point1=dynamic_cast<Terminal *>(RHS.at(0));
        Terminal * RHS_point2=dynamic_cast<Terminal *>(RHS.at(1));
        LHS->addChild(RHS_point1);
        LHS->addChild(RHS_point2);
        cout<<RHS_point1->getIndex()<<","<<RHS_point2->getIndex()<<endl;
        assert(RHS_point1->getIndex()!=RHS_point2->getIndex());
        
        LHS->setAdditionalCost(pcl::euclideanDistance<PointT,PointT>(RHS_point1->getPoint(),RHS_point2->getPoint()));
        return LHS;
    }
    
     void combineAndPush(Symbol * sym, set<int> & combineCandidates , SymbolPriorityQueue & pqueue, vector<NTSet> & planeSet /* = 0 */, vector<Terminal*> & terminals /* = 0 */)
    {
        set<int>::iterator it;
     //   cout<<"my type:"<<typeid(*sym).name()<<endl;
     //   cout<<"my type:"<<typeid(Terminal).name()<<endl;
        if(typeid(*sym)==typeid(Terminal))
        {
            for(it=combineCandidates.begin();it!=combineCandidates.end();it++)
            {
              //  cout<<"cand type:"<<typeid(**it).name()<<endl;
                    vector<Symbol*> temp;
                    temp.push_back(terminals.at(*it)); 
                    temp.push_back(sym);
                    addToPqueueIfNotDuplicate(applyRule(temp),planeSet,pqueue);
                    cout<<"applied rule p->tt\n";
            }
        }
    }    
};

class Goal_S : public NonTerminal
{
protected:
public:
    Goal_S():NonTerminal()
    {
        
    }
    
    void printData()
    {
        pcl::PointCloud<pcl::PointXYZRGBIndex> sceneOut;
        sceneOut.points.resize(scene.size());
        for(size_t i=0;i<scene.size();i++)
        {
            sceneOut.points[i].x=scene.points[i].x;
            sceneOut.points[i].y=scene.points[i].y;
            sceneOut.points[i].z=scene.points[i].z;
            sceneOut.points[i].rgb=scene.points[i].rgb;
        }

        std::ofstream logFile;
        logFile.open("log.txt",ios::out);
        NonTerminal *plane1=dynamic_cast<NonTerminal *>(children[0]);
        NonTerminal *plane2=dynamic_cast<NonTerminal *>(children[1]);
        for(size_t i=0;i<plane1->pointIndices.size();i++)
        {
            logFile<<","<<plane1->pointIndices[i];
            sceneOut.points[plane1->pointIndices[i]].index=1;
        }
        logFile<<endl;
        for(size_t i=0;i<plane2->pointIndices.size();i++)
        {
            logFile<<","<<plane2->pointIndices[i];
            sceneOut.points[plane2->pointIndices[i]].index=2;
        }
        logFile<<endl;
        logFile.close();
    pcl::io::savePCDFile("fridge_out.pcd",sceneOut,true);
    }
};

class RS_PlanePlane : public Rule
{
public:
    void addToPqueueIfNotDuplicate(NonTerminal * newNT, vector<NTSet> & planeSet, SymbolPriorityQueue & pqueue)
    {
        newNT->computeSetMembership();
   //     if(!newNT->checkDuplicate(planeSet)) // no need to check for duplicates
            pqueue.push(newNT);
    }
    
    int get_Nof_RHS_symbols()
    {
        return 2;
    }
    
    void  get_typenames(vector<string> & names)
    {
        names.push_back(typeid(Plane).name());
        names.push_back(typeid(Plane).name());
    }
    
    NonTerminal* applyRule(vector<Symbol*> & RHS)
    {
        Goal_S * LHS=new Goal_S();
        Plane * RHS_plane1=dynamic_cast<Plane *>(RHS.at(0));
        Plane * RHS_plane2=dynamic_cast<Plane *>(RHS.at(1));
        LHS->addChild(RHS_plane1);
        LHS->addChild(RHS_plane2);
        //int deficit=scene.size()-RHS_plane1->getNumPoints()-RHS_plane2->getNumPoints();
        LHS->setAdditionalCost(RHS_plane1->coplanarity(RHS_plane2)/*+exp(10*deficit)*/); // more coplanar => bad
        cout<<"applied rule S->pp\n";        
        cerr<<"applied rule S->pp: cost "<<LHS->cost<<"\n";        
        cerr<<RHS_plane1->set_membership<<"\n";        
        cerr<<RHS_plane2->set_membership<<"\n";        
        return LHS;
    }
    
     void combineAndPush(Symbol * sym, set<int> & combineCandidates , SymbolPriorityQueue & pqueue, vector<NTSet> & planeSet /* = 0 */, vector<Terminal*> & terminals /* = 0 */)
    {
        
        if(typeid(*sym)==typeid(Plane))
        {
            size_t targetSize=scene.size()-sym->getNumPoints();
                NTSet & bin=planeSet[targetSize];
                if(bin.size()==0)
                    return;
                
            boost::dynamic_bitset<> neighbors(scene.size());
            set<int>::iterator nit;
            
            for(nit=combineCandidates.begin();nit!=combineCandidates.end();nit++)
            {
                neighbors.set(*nit,true);
            }
            
                NTSet::iterator it;
                    Plane * RHS_plane1=dynamic_cast<Plane *>(sym);
//                    Plane * RHS_plane2=dynamic_cast<Plane *>(*it);
                assert(!RHS_plane1->set_membership.intersects(neighbors));
                for(it=bin.begin();it!=bin.end();it++)
                {
                    if(  (!RHS_plane1->intersects((*it)))   && (*it)->set_membership.intersects(neighbors)  )
                    {
                        vector<Symbol*> temp;
                        temp.push_back(*it); // must be pushed in order
                        temp.push_back(sym);
                        addToPqueueIfNotDuplicate(applyRule(temp),planeSet,pqueue);
                    }
                }
                    
        }
    }    
};

typedef boost::shared_ptr<Rule>  RulePtr;
void appendRuleInstances(vector<RulePtr> & rules)
{
    rules.push_back(RulePtr(new RPlane_PlanePoint()));    
    rules.push_back(RulePtr(new RPlane_PointPoint()));    
    rules.push_back(RulePtr(new RS_PlanePlane()));    
}

void log(int iter, Symbol *  sym)
{
    if(sym->getNumPoints()==1)
        return;
        pcl::PointCloud<pcl::PointXYZRGBIndex> sceneOut;
        sceneOut.points.resize(scene.size());
        for(size_t i=0;i<scene.size();i++)
        {
            sceneOut.points[i].x=scene.points[i].x;
            sceneOut.points[i].y=scene.points[i].y;
            sceneOut.points[i].z=scene.points[i].z;
            sceneOut.points[i].rgb=scene.points[i].rgb;
        }

        std::ofstream logFile;
        NonTerminal *plane1=dynamic_cast<NonTerminal *>(sym);
        for(size_t i=0;i<plane1->pointIndices.size();i++)
        {
            sceneOut.points[plane1->pointIndices[i]].index=1;
        }
    pcl::io::savePCDFile("fridge_out"+boost::lexical_cast<std::string>(iter)+".pcd",sceneOut,true);
    
}
void runParse()
{
    vector<RulePtr> rules;
    appendRuleInstances(rules);
    SymbolPriorityQueue pq;
    int numPoints=scene.size();
//    vector<set<NonTerminal*> > ancestors(numPoints,set<NonTerminal*>());
    
    vector<NTSet> planeSet(numPoints,NTSet());
    
    vector<Terminal *> terminals;
   
    Terminal * temp;
    for(int i=0;i<numPoints;i++)
    {
        temp=new Terminal(i);
        terminals.push_back(temp);
        pq.push(temp);
    }
    
    Symbol *min;
    int count=0;
    while(true)
    {
        min=pq.top();
        
        cout<<"\n\n\niter: "<<count++<<" cost was:"<<min->getCost()<<" id was "<<min->getId()<<endl;
        
        if(typeid(*min)==typeid(Goal_S))
        {
            cout<<"goal reached!!"<<endl;
            min->printData();
            return;
            
        }
        if(min->finalize_if_not_duplicate(planeSet))
        {
        min->printData();
           // log(count,min);
            set<int> combineCandidates;
            min->getValidSymbolsForCombination(terminals,combineCandidates);
            cout<<combineCandidates.size()<<" candidates found and queue has "<<pq.size()<<" elements: \n------------"<<endl;
            set<int>::iterator it;
            for(it=combineCandidates.begin();it!=combineCandidates.end();it++)
            {
                cout<<*it<<",";
            }
            cout<<"--------\n";
            
            for(size_t i=0;i<rules.size();i++)
            {
                rules[i]->combineAndPush(min, combineCandidates, pq, planeSet, terminals);
            }
            
        }
        else
        {
            delete min; 
            cout<<"duplicate detected"<<endl;
            // since there are no parent links, and it was not a child of anyone
            // deleting does not cause dangling pointers
            
        }
        
        
        
        pq.pop();
        
    }
}
void subsample(pcl::PointCloud<PointT> & inp,pcl::PointCloud<PointT> & out)
{
    out.points.clear();
    out.header=inp.header;
    for(size_t i=0;i<inp.size();i++)
    {
        if(rand() % 5==1)
        {
            out.points.push_back(inp.points[i]);
        }
    }
}
int main(int argc, char** argv) 
{
    pcl::io::loadPCDFile<PointT>(argv[1], scene);
//    pcl::PointCloud<PointT> temp;
//    subsample(scene,temp);
//    pcl::io::savePCDFile("fridge_sub500.pcd",temp,true);
    cout<<"scene has "<<scene.size()<<" points"<<endl;
    
    
   runParse();
    return 0;
}
