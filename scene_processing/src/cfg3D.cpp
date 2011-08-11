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

    void getValidSymbolsForCombination(vector<set<NonTerminal*> > & allAncestors,vector<Terminal*> & terminals,set<Symbol*> & combineCanditates)
    {
        // find the nearest point not in this segment
        pcl::KdTreeFLANN<PointT> nnFinder;
        vector<int> indices;
        getComplementPointSet(indices);
        cout<<indices.size()<<" points in complement set"<<endl;
//        pcl::PointCloud<PointT>::Ptr scene_ptr=new pcl::PointCloud<PointT>::Ptr(scene)
        nnFinder.setInputCloud(createStaticShared<pcl::PointCloud<PointT> >(&scene),createStaticShared<vector<int> >(&indices));
        vector<int> nearest_index;
        vector<float> nearest_distances;
        pcl::PointXYZ centroidt;
        getCentroid(centroidt);
        PointT centroidTT;
        centroidTT.x=centroidt.x;
        centroidTT.y=centroidt.y;
        centroidTT.z=centroidt.z;
        nearest_index.resize(1,0);
        nearest_distances.resize(1,0.0);
        nnFinder.nearestKSearch(centroidTT,1,nearest_index,nearest_distances);
        cout<<"nearest index found was "<<nearest_index[0]<<endl;
        
        //get all it's ancestors which are not in common with ancesstors of this
        set<NonTerminal*> thisAncestors;
        set<NonTerminal*> combineNTCanditates;
        combineNTCanditates.insert(allAncestors[nearest_index[0]].begin(),allAncestors[nearest_index[0]].end());
        getSetOfAncestors(thisAncestors,allAncestors);
        setDiffernce<NonTerminal*>(combineNTCanditates/*at this point, it only contains NT's*/,thisAncestors);
        combineCanditates.clear();
        set<NonTerminal*>::iterator it;
        for(it=combineNTCanditates.begin();it!=combineNTCanditates.end();it++)
        {
            combineCanditates.insert(combineCanditates.end(),(Symbol*)*it);
        }
        
        cout<<"sa "<<combineCanditates.size()<<endl;
        //add itself
        combineCanditates.insert((Symbol*)terminals[nearest_index[0]]);        
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
    
    const PointT & getPoint()
    {
        return scene.points[index];
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
        cout<<" terminal complement of index"<<index<<"\n";
        indices.clear();
        for(size_t i=0;i<scene.size();i++)
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
    bool costSet;
public:
    vector<int> pointIndices; // can be replaced with any sufficient statistic
    NonTerminal()
    {
        costSet=false;
    }
    
    size_t getNumPoints()
    {
        return pointIndices.size();
    }

    void setAdditionalCost(double additionalCost)
    {
        assert(additionalCost>=0);
        cost=0;
        for(size_t i=0;i<children.size();i++)
            cost+=children[i]->getCost();
        cost+=additionalCost;
        costSet=true;
    }
    
    void addChild(Symbol * child)
    {
        assert(!costSet);
        children.push_back(child);
    }
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
    bool finalize_if_not_duplicate(vector<set<NonTerminal*> > & ancestors)
    {
        assert(costSet); // cost must be set before adding it to pq
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
                if(typeid(*this)==typeid(*(*it)) && pointIndices.size()==(*it)->pointIndices.size())
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
            if(typeid(*(RHS.at(i))).name()!=typenames.at(i))
                return false;
        }
        return true;
    }
    
    virtual void combineAndPush(Symbol * sym, set<Symbol*> & combineCandidates , SymbolPriorityQueue & pqueue)=0;
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
         return exp(planeParams[0]*planeParams[0]  +  planeParams[1]*planeParams[1]  +  planeParams[3]*planeParams[3]);

    }
    
    void computePlaneParams()
    {
        pcl::NormalEstimation<PointT,pcl::Normal> normalEstimator;
        normalEstimator.computePointNormal(scene, pointIndices, planeParams, curvature);
        assert(getNorm()>0.9);
        assert(getNorm()<1.1);
        planeParamsComputed=true;
    }
    
    
    double coplanarity(Plane * plane2)
    {
        return exp(planeParams[0]*plane2->planeParams[0]  +  planeParams[1]*plane2->planeParams[1]  +  planeParams[3]*plane2->planeParams[3]);
    }
    
    double costOfAddingPoint(PointT p)
    {
        if(pointIndices.size()==2)
        {
            /*return it's min distance to either points
             */
            double d1=pcl::euclideanDistance<PointT,PointT>(p,scene.points[pointIndices[0]]);
            double d2=pcl::euclideanDistance<PointT,PointT>(p,scene.points[pointIndices[1]]);
            if(d1<d2)
                return d1;
            else
                return d2;
        }
        else if(pointIndices.size()>=3)
        {
            assert(planeParamsComputed);
            return exp(2*pcl::pointToPlaneDistance<PointT>(p,planeParams))-1;
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
    
     void combineAndPush(Symbol * sym, set<Symbol*> & combineCandidates , SymbolPriorityQueue & pqueue)
    {
        set<Symbol*>::iterator it;
        if(typeid(*sym)==typeid(Terminal))
        {
            for(it=combineCandidates.begin();it!=combineCandidates.end();it++)
            {
                if(typeid(*(*it))==typeid(Plane))
                {
                    vector<Symbol*> temp;
                    temp.push_back(*it); // must be pushed in order
                    temp.push_back(sym);
                    pqueue.push(applyRule(temp));
                    
                }
            }
        }
        else if (typeid(*sym)==typeid(Plane ))
        {
            for(it=combineCandidates.begin();it!=combineCandidates.end();it++)
            {
                if(typeid(*(*it))==typeid(Terminal))
                {
                    vector<Symbol*> temp;
                    temp.push_back(sym);
                    temp.push_back(*it); // must be pushed in order
                    pqueue.push(applyRule(temp));
                    
                }
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
    
     void combineAndPush(Symbol * sym, set<Symbol*> & combineCandidates , SymbolPriorityQueue & pqueue)
    {
        set<Symbol*>::iterator it;
     //   cout<<"my type:"<<typeid(*sym).name()<<endl;
     //   cout<<"my type:"<<typeid(Terminal).name()<<endl;
        if(typeid(*sym)==typeid(Terminal))
        {
            for(it=combineCandidates.begin();it!=combineCandidates.end();it++)
            {
                cout<<"cand type:"<<typeid(**it).name()<<endl;
                if(typeid(*(*it))==typeid(Terminal))
                {
                    vector<Symbol*> temp;
                    temp.push_back(*it); // must be pushed in order
                    temp.push_back(sym);
                    pqueue.push(applyRule(temp));
                    cout<<"applied rule p->tt\n";
                }
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
        std::ofstream logFile;
        logFile.open("log.txt",ios::out);
        NonTerminal *plane1=dynamic_cast<NonTerminal *>(children[0]);
        NonTerminal *plane2=dynamic_cast<NonTerminal *>(children[1]);
        for(size_t i=0;i<plane1->pointIndices.size();i++)
            cout<<","<<plane1->pointIndices[i];
        cout<<endl;
        for(size_t i=0;i<plane2->pointIndices.size();i++)
            cout<<","<<plane2->pointIndices[i];
        cout<<endl;
    }
};

class RS_PlanePlane : public Rule
{
public:
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
        LHS->setAdditionalCost(RHS_plane1->coplanarity(RHS_plane2)); // more coplanar => bad
        return LHS;
    }
    
     void combineAndPush(Symbol * sym, set<Symbol*> & combineCandidates , SymbolPriorityQueue & pqueue)
    {
        set<Symbol*>::iterator it;
        if(typeid(*sym)==typeid(Plane))
        {
            for(it=combineCandidates.begin();it!=combineCandidates.end();it++)
            {
                if(typeid(*(*it))==typeid(Plane))
                {
                    Plane * RHS_plane1=dynamic_cast<Plane *>(sym);
                    Plane * RHS_plane2=dynamic_cast<Plane *>(*it);
                    
                    if(RHS_plane1->getNumPoints()+RHS_plane2->getNumPoints()<scene.size())
                        continue;
                    
                    vector<Symbol*> temp;
                    temp.push_back(*it); // must be pushed in order
                    temp.push_back(sym);
                    pqueue.push(applyRule(temp));
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


void runParse()
{
    vector<RulePtr> rules;
    appendRuleInstances(rules);
    SymbolPriorityQueue pq;
    int numPoints=scene.size();
    vector<set<NonTerminal*> > ancestors(numPoints,set<NonTerminal*>());
    
    vector<Terminal *> terminals;
   
    Terminal * temp;
    for(int i=0;i<numPoints;i++)
    {
        temp=new Terminal(i);
        terminals.push_back(temp);
        pq.push(temp);
    }
    
    Symbol *min;
    while(true)
    {
        min=pq.top();
        if(typeid(*min)==typeid(Goal_S))
        {
            cout<<"goal reached!!"<<endl;
            min->printData();
            return;
            
        }
        if(min->finalize_if_not_duplicate(ancestors))
        {
            set<Symbol*> combineCandidates;
            min->getValidSymbolsForCombination(ancestors,terminals,combineCandidates);
            cout<<combineCandidates.size()<<" candidates found and queue has "<<pq.size()<<" elements"<<endl;
            for(size_t i=0;i<rules.size();i++)
            {
                rules[i]->combineAndPush(min,combineCandidates,pq);
            }
            
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
    pcl::io::loadPCDFile<PointT>("transformed_fridge.pcd", scene);
    cout<<"scene has "<<scene.size()<<" points"<<endl;
    runParse();
    return 0;
}
