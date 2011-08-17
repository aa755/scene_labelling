/* 
 * File:   genericUtils.h
 * Author: aa755
 *
 * Created on August 16, 2011, 11:00 PM
 */

#ifndef GENERICUTILS_H
#define	GENERICUTILS_H
#include <Eigen/Dense>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>

template<typename _Scalar>
void readCSV(std::string filename, int height,int width, std::string separator,Eigen::Matrix<_Scalar,Eigen::Dynamic,  Eigen::Dynamic> & mat)
{
   
    std::ifstream file;
    file.open(filename.data(),std::ios::in);
    std::string line;
    boost::char_separator<char> sep(separator.data());
    for(int r=0;r<height;r++)
    {
        getline(file, line);
        boost::tokenizer<boost::char_separator<char> > tokens1(line, sep);        
        int c=0;
        BOOST_FOREACH(std::string t, tokens1) 
        {
            mat(r,c)=boost::lexical_cast<_Scalar>(t);
            c++;            
        }
        assert(c==width);
    }
}


#endif	/* GENERICUTILS_H */

