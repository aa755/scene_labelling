#include <math.h>
//#include "mex.h"
#include <stdlib.h>
// small value, used to avoid division by zero
#define eps 0.0001

// originally obtained from http://people.cs.uchicago.edu/~pff/
// modified by Abhishek Anand (abhishek.anand.iitg@gmail.com)

class Point2DAbhishek
{
public:
  int x;
  int y;
  Point2DAbhishek(int x_,int y_)
  {
    x=x_;
    y=y_;
  }
  Point2DAbhishek()
  {
  }
};


class HOGFeaturesOfBlock
{
public:
  static int const numDirections=9;
  static int const numFeats=27+4+1;
  double feats[numFeats];
  static void aggregateFeatsOfBlocks(std::vector<HOGFeaturesOfBlock> & featsOfBlocks, HOGFeaturesOfBlock & aggFeats)
  {
 //   cout<<"here goes the HOG feats"<<endl;
    size_t numBlocks=featsOfBlocks.size();
    for(size_t f=0;f<numFeats;f++)
      {
        aggFeats.feats[f]=0;
        for(size_t b=0;b<numBlocks;b++)
          aggFeats.feats[f]+=featsOfBlocks[b].feats[f];
        aggFeats.feats[f]/=numBlocks;
     //   cout<<aggFeats.feats[f]<<endl;
      } 
  }
  
  void pushBackAllFeats(std::vector<float> & featureVector)
  {
    for(size_t i=0;i<numFeats-1;i++) // the last one always seems to be 0
      featureVector.push_back(feats[i]);
  }
  
  void pushBackAllDiffFeats(const HOGFeaturesOfBlock & other,std::vector<float> & featureVector)
  {
    for(size_t i=0;i<numFeats-1;i++) // the last one always seems to be 0
      featureVector.push_back(fabs(feats[i]-other.feats[i]));
  }

  void pushNonContrastFeats(std::vector<float> & featureVector) // the last one always seems to be 0
  {
    for(size_t i=numDirections*2;i<numFeats-1;i++)
      featureVector.push_back(feats[i]);
  }
  void pushTextureFeats(std::vector<float> & featureVector) // the last one always seems to be 0
  {
    for(size_t i=numDirections*3;i<numFeats-1;i++)
      featureVector.push_back(feats[i]);
  }
};

class HOG
{
static  double const uu[9];
static double const vv[9];
// unit vectors used to compute gradient orientation
  double *feat;
  int numBlocksOutX;
  int numBlocksOutY;
  


public :
  HOG()
  {
    feat=NULL;
  }
  ~HOG()
  {
    free( feat);
  }
  
  void getFeatVec(int blockY, int blockX, HOGFeaturesOfBlock & featsB)
  {
    for(int featIndex=0;featIndex<HOGFeaturesOfBlock::numFeats;featIndex++)
        featsB.feats[featIndex]=*(feat + featIndex*numBlocksOutX*numBlocksOutY + blockX*numBlocksOutY+ blockY);
  }
  
  static void saveFloatImage ( const char* filename, const IplImage * image )
{
  IplImage * saveImage = cvCreateImage ( cvGetSize ( image ),
                                             IPL_DEPTH_32F, 3 );
  cvConvertScale ( image, saveImage, 255, 0 );
  cvSaveImage( filename, saveImage);
  cvReleaseImage ( &saveImage );
}

  void saveFeatAsImages()
  {
    /*
    IplImage *featImages[HOGFeaturesOfBlock::numFeats];
    HOGFeaturesOfBlock feats[numBlocksOutY][numBlocksOutX];
    for(size_t y=0;y<numBlocksOutY;y++)
       for(size_t x=0;x<numBlocksOutX;x++)
         {
           getFeatVec (y,x,feats[y][x]);
         }
    
    for(size_t f=0;f<HOGFeaturesOfBlock::numFeats;f++)
      {
        featImages[f] = cvCreateImage (cvSize (numBlocksOutX, numBlocksOutY), IPL_DEPTH_32F, 3);
    for(size_t y=0;y<numBlocksOutY;y++)
       for(size_t x=0;x<numBlocksOutX;x++)
         {
        CvScalar s;
        s.val[0] = feats[y][x].feats[f];
        s.val[1] = feats[y][x].feats[f];
        s.val[2] = feats[y][x].feats[f];        
           cvSet2D(featImages[f],y,x,s); // set the (i,j) pixel value
         }
        char filename[20];
        sprintf (filename,"feat%d.png",f);
       saveFloatImage (filename,featImages[f]); 
      }
    */
    std::vector<Point2DAbhishek> inPoints;
    inPoints.push_back(Point2DAbhishek(420,350));
    inPoints.push_back(Point2DAbhishek(420,350));
    inPoints.push_back(Point2DAbhishek(432,351));
    inPoints.push_back(Point2DAbhishek(432,351));
    HOGFeaturesOfBlock temp;
    getFeatValForPixels (inPoints,temp);
  }
 
  void getFeatValForPixels(const std::vector<Point2DAbhishek> & interestPointsInImage, HOGFeaturesOfBlock & hogFeats)
  {
    // bin pixels into blocksOuts()
//    cout<<interestPointsInImage.size ()<<endl;
    int numPointsInBlock[numBlocksOutY][numBlocksOutX];
    for(int y=0;y<numBlocksOutY;y++)
        for(int x=0;x<numBlocksOutX;x++)
          numPointsInBlock[y][x]=0;
    
    Point2DAbhishek outBlock;
    
    for(int i=0;i<interestPointsInImage.size();i++)
      {
   //     cout<<"in loop"<<endl;
        pixel2BlockOut (interestPointsInImage[i],outBlock);
        if(outBlock.x!=-1) // not out of range ... boundary blocks are out of range
          numPointsInBlock[outBlock.y][outBlock.x]++;
     //   cout<<"out loop"<<endl;
      }
    
    int max=-1;
    //find the block(s) where most pixels lie 
    for(int y=0;y<numBlocksOutY;y++)
        for(int x=0;x<numBlocksOutX;x++)
          {
            if(max<numPointsInBlock[y][x])
                max=numPointsInBlock[y][x];
          }
    HOGFeaturesOfBlock temp;
    std::vector<HOGFeaturesOfBlock> maxBlockFeats;
 //   cout<<"max= "<<max<<endl;
    for(int y=0;y<numBlocksOutY;y++)
        for(int x=0;x<numBlocksOutX;x++)
          {
            if(max==numPointsInBlock[y][x])
              {
   //             cout <<"out block selected" <<x <<","<<y<<endl;
                getFeatVec (y,x,temp);
                maxBlockFeats.push_back(temp);
              }
          }
    HOGFeaturesOfBlock::aggregateFeatsOfBlocks (maxBlockFeats,hogFeats);
    //push all out blocks with max to a vector and aggregate
  }
  
  void pixel2BlockOut(const Point2DAbhishek & p,Point2DAbhishek  & b )
  {
    //return -1 if out of range block
    b.x=((int)round((float)p.x/(float)sbin)) -1 ;
    b.y=((int)round((float)p.y/(float)sbin)) -1;
    //cout <<"block selected"<< b.x<<","<<b.y<<" for pixel "<<p.x<<","<<p.y<<endl;
    if(b.x<0 || b.x>=numBlocksOutX ||b.y<0 || b.y>=numBlocksOutY )
      b.x=-1;
  }
  
static inline double min(double x, double y) { return (x <= y ? x : y); }
static inline double max(double x, double y) { return (x <= y ? y : x); }

static inline int min(int x, int y) { return (x <= y ? x : y); }
static inline int max(int x, int y) { return (x <= y ? y : x); }

static  int const sbin=8;

void computeHog(IplImage * img)
{
  assert(img->nChannels==3);
  double * matlabImage= (double *)calloc(img->width*img->height*img->nChannels,sizeof(double));
 // cout<<"processing an image of size"<<img->width<<","<<img->height<<endl;
  int ndims[3]={img->height,img->width,3};
  for(size_t y=0;y<img->height;y++)
     for(size_t x=0;x<img->width;x++)
       {
         CvScalar s;
         s=cvGet2D(img,y,x); // get the (i,j) pixel value
         for(size_t ch=0;ch<img->nChannels;ch++)
                *(matlabImage+getOffsetInMatlabImage (y,x,ch,img->height,img->width))=s.val[ch];
       }
  process (matlabImage,ndims);
  free(matlabImage);
}

size_t getOffsetInMatlabImage(size_t y, size_t x, size_t channel,size_t dimY, size_t dimX)
{
  assert(channel*dimX*dimY+x*dimY+y<dimX*dimY*3);
  return channel*dimX*dimY+x*dimY+y;
}
// main function:
// takes a double color image and a bin size 
// returns HOG features
void process(const double *im, const int *dims) {

  // memory for caching orientation histograms & their norms
  int numBlocksInX;
  int numBlocksInY;
  int blocks[2];
  blocks[0] = (int)round((double)dims[0]/(double)sbin); 
  numBlocksInY=blocks[0];
  blocks[1] = (int)round((double)dims[1]/(double)sbin);
  numBlocksInX=blocks[1];
  
  double *hist = (double *)calloc(blocks[0]*blocks[1]*18, sizeof(double)); // stores histogram of gradients along each direction in a block
  double *norm = (double *)calloc(blocks[0]*blocks[1], sizeof(double)); // stores for the norm for each block

  // memory for HOG features
  int out[3];
  out[0] = max(blocks[0]-2, 0); // ignore the boundary blocks ?
  numBlocksOutY=out[0];
  
  out[1] = max(blocks[1]-2, 0);
  numBlocksOutX=out[1];
  cout <<"HOG block dims"<<numBlocksOutX<<","<<numBlocksOutY<<endl;
  out[2] = HOGFeaturesOfBlock::numFeats ;//32 dimensional feature for each block
//  mxArray *mxfeat = mxCreateNumericArray(3, out, mxDOUBLE_CLASS, mxREAL);
//  mxArray *mxfeat = mxCreateNumericArray(3, out, mxDOUBLE_CLASS, mxREAL);
   feat = (double *)calloc(out[0]*out[1]*out[2],sizeof(double));
  
  int visible[2];
  visible[0] = blocks[0]*sbin;
  visible[1] = blocks[1]*sbin;
  
  for (int x = 1; x < visible[1]-1; x++) {
    for (int y = 1; y < visible[0]-1; y++) {
      // first color channel
      const double *s = im + min(x, dims[1]-2)*dims[0] + min(y, dims[0]-2);
      double dy = *(s+1) - *(s-1);
      double dx = *(s+dims[0]) - *(s-dims[0]);
      double v = dx*dx + dy*dy;

      // second color channel
      s += dims[0]*dims[1];
      double dy2 = *(s+1) - *(s-1);
      double dx2 = *(s+dims[0]) - *(s-dims[0]); //dims[0]=dimY
      double v2 = dx2*dx2 + dy2*dy2;

      // third color channel
      s += dims[0]*dims[1];
      double dy3 = *(s+1) - *(s-1);
      double dx3 = *(s+dims[0]) - *(s-dims[0]);
      double v3 = dx3*dx3 + dy3*dy3;

      // pick channel with strongest gradient
      if (v2 > v) {
	v = v2;
	dx = dx2;
	dy = dy2;
      } 
      if (v3 > v) {
	v = v3;
	dx = dx3;
	dy = dy3;
      }

      // snap to one of 18 orientations
      double best_dot = 0;
      int best_o = 0;
      for (int o = 0; o < 9; o++) {
	double dot = uu[o]*dx + vv[o]*dy;
	if (dot > best_dot) {
	  best_dot = dot;
	  best_o = o;
	} else if (-dot > best_dot) {
	  best_dot = -dot;
	  best_o = o+9;
	}
      }
      
      // add to 4 histograms(blocks) around pixel using linear interpolation
      double xp = ((double)x+0.5)/(double)sbin - 0.5;
      double yp = ((double)y+0.5)/(double)sbin - 0.5;
      int ixp = (int)floor(xp);
      int iyp = (int)floor(yp);
      double vx0 = xp-ixp;
      double vy0 = yp-iyp;
      double vx1 = 1.0-vx0;
      double vy1 = 1.0-vy0;
      v = sqrt(v);

      if (ixp >= 0 && iyp >= 0) {
	*(hist + ixp*blocks[0] + iyp + best_o*blocks[0]*blocks[1]) += 
	  vx1*vy1*v;
      }

      if (ixp+1 < blocks[1] && iyp >= 0) {
	*(hist + (ixp+1)*blocks[0] + iyp + best_o*blocks[0]*blocks[1]) += 
	  vx0*vy1*v;
      }

      if (ixp >= 0 && iyp+1 < blocks[0]) {
	*(hist + ixp*blocks[0] + (iyp+1) + best_o*blocks[0]*blocks[1]) += 
	  vx1*vy0*v;
      }

      if (ixp+1 < blocks[1] && iyp+1 < blocks[0]) {
	*(hist + (ixp+1)*blocks[0] + (iyp+1) + best_o*blocks[0]*blocks[1]) += 
	  vx0*vy0*v;
      }
    }
  }

  // compute energy in each block by summing over orientations
  for (int o = 0; o < 9; o++) {
    double *src1 = hist + o*blocks[0]*blocks[1];
    double *src2 = hist + (o+9)*blocks[0]*blocks[1];
    double *dst = norm;
    double *end = norm + blocks[1]*blocks[0];
    while (dst < end) {
      *(dst++) += (*src1 + *src2) * (*src1 + *src2);
      src1++;
      src2++;
    }
  }

  // compute features
  for (int x = 0; x < out[1]; x++) {
    for (int y = 0; y < out[0]; y++) {
      double *dst = feat + x*out[0] + y;      
      double *src, *p, n1, n2, n3, n4;

      p = norm + (x+1)*blocks[0] + y+1;
      n1 = 1.0 / sqrt(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + eps);
      p = norm + (x+1)*blocks[0] + y;
      n2 = 1.0 / sqrt(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + eps);
      p = norm + x*blocks[0] + y+1;
      n3 = 1.0 / sqrt(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + eps);
      p = norm + x*blocks[0] + y;      
      n4 = 1.0 / sqrt(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + eps);

      double t1 = 0;
      double t2 = 0;
      double t3 = 0;
      double t4 = 0;

      // contrast-sensitive features
      src = hist + (x+1)*blocks[0] + (y+1);
      for (int o = 0; o < 18; o++) {
	double h1 = min(*src * n1, 0.2);
	double h2 = min(*src * n2, 0.2);
	double h3 = min(*src * n3, 0.2);
	double h4 = min(*src * n4, 0.2);
	*dst = 0.5 * (h1 + h2 + h3 + h4);
	t1 += h1;
	t2 += h2;
	t3 += h3;
	t4 += h4;
	dst += out[0]*out[1];
	src += blocks[0]*blocks[1];
      }

      // contrast-insensitive features
      src = hist + (x+1)*blocks[0] + (y+1);
      for (int o = 0; o < 9; o++) {
        double sum = *src + *(src + 9*blocks[0]*blocks[1]);
        double h1 = min(sum * n1, 0.2);
        double h2 = min(sum * n2, 0.2);
        double h3 = min(sum * n3, 0.2);
        double h4 = min(sum * n4, 0.2);
        *dst = 0.5 * (h1 + h2 + h3 + h4);
        dst += out[0]*out[1];
        src += blocks[0]*blocks[1];
      }

      // texture features
      *dst = 0.2357 * t1;
      dst += out[0]*out[1];
      *dst = 0.2357 * t2;
      dst += out[0]*out[1];
      *dst = 0.2357 * t3;
      dst += out[0]*out[1];
      *dst = 0.2357 * t4;

      // truncation feature
      dst += out[0]*out[1];
      *dst = 0;
    }
  }

  free(hist);
  free(norm);
//  return feat;
}

int
getNumFeatsPerBlock () const
{
  return HOGFeaturesOfBlock::numFeats;
}

int
getNumBlocksY () const
{
  return numBlocksOutY;
}

int
getNumBlocksX () const
{
  return numBlocksOutX;
}

};

double const HOG::uu[9] = {1.0000, 
		0.9397, 
		0.7660, 
		0.500, 
		0.1736, 
		-0.1736, 
		-0.5000, 
		-0.7660, 
		-0.9397};
double const HOG::vv[9] = {0.0000, 
		0.3420, 
		0.6428, 
		0.8660, 
		0.9848, 
		0.9848, 
		0.8660, 
		0.6428, 
		0.3420};
