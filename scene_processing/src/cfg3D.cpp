/* 
 * File:   main.cpp
 * Author: abhishek
 *
 * Created on July 26, 2011, 7:41 PM
 */

#include <iostream>
#include <vector>
#include<typeinfo>
using namespace std;

/*
 * 
 */
class Terminal;

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
    float x,y,z,rgb;
};

class NonTerminal : public Symbol
{
    vector<Symbol*> children;
    vector<Terminal*> leaves;
    
    /** 
     * compute leaves by concatenating 
     * leaves of children */
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

