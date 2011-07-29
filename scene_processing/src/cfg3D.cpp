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
};

class Terminal : public Symbol
{
    Terminal()
    {
        cost=0;
    }
protected:
    PointT pt;
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
    void computeLeaves()
    {
        
    }

};

class Rule
{    
    bool isApplicable(vector<Symbol*> RHS);
    
    /**
     *  applies this rule on the given params(RHS of rule)
     * computes the cos of resulting NT
     */
    virtual NonTerminal* applyRule(vector<Symbol*> RHS)=0;
};

class Plane : public NonTerminal
{
    Eigen::Vector4f planeParams;
    float curvature;
    void computePlaneParams()
    {
        pcl::NormalEstimation<PointT,PointT> normalEstimator;
        normalEstimator.computePointNormal(scene, pointIndices, planeParams, curvature);
    }
};
        
int main(int argc, char** argv) 
{

    vector<int> hello;
    NonTerminal nt1;
    Symbol *sym=&nt1;
    cout<<"type:"<<typeid(sym).name()<<endl;
    cout<<"type:"<<typeid(Symbol *).name()<<endl;
    cout<<"type:"<<(typeid(sym)==typeid(Symbol *))<<endl;
    
    return 0;
}

