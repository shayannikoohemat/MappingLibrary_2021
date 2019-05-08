
/*
    Copyright 2010 University of Twente and Delft University of Technology
 
       This file is part of the Mapping libraries and tools, developed
  for research, education and projects in photogrammetry and laser scanning.

  The Mapping libraries and tools are free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation, either version 3 of the License,
                   or (at your option) any later version.

 The Mapping libraries and tools are distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
                GNU General Public License for more details.

      You should have received a copy of the GNU General Public License
          along with the Mapping libraries and tools.  If not, see
                      <http://www.gnu.org/licenses/>.

----------------------------------------------------------------------------*/


#include "containers/vector"
#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <io/syscall_file.h>
#include "algo/scan.h"
#include "LaserPoints.h"
#include "KNNFinder.h"
#include "StlUtilities.h"
#include "ProgressDisplay.h"

#include "MyTypeDefinitions.h"
#include "Partition3D.h"
#include "Vector3DR.h"
#include "LaserPointsXL.h"


//typedef long long int int64;

#define int64 long long int
using namespace stxxl;
using namespace std;

int main_stxxl()
{
	
	typedef int IndexType;
	typedef  Vector3DR PointType;
	typedef std::vector<IndexType> VectorType;
	typedef std::vector<IndexType> IndicesType;
	
#if 0	
	VectorType src(200);
	VectorType sink;
		
	SelectorVector<IndexType> vec;
	vec.push_back(new RangeSelector<IndexType>(0,src.size()/2));
	
	IndicesSelector<IndexType>* inSel = new IndicesSelector<IndexType>();
	for(int i=0;i<src.size();i+=10)
		inSel->push_back(i);
	vec.push_back(inSel);
	vec.push_back(new SelectorVector<IndexType>());
	
	//Add selected indices from src to sink.
	cerr<<"Sink before selection: "<<sink.size()<<endl;
	vec.AddSelection(src,sink);
	cerr<<"Sink after selection: "<<sink.size()<<endl;
		
	const char fileName[] = "test_selectors.tmp";
	ofstream outFile(fileName);
	vec.Save(outFile);
	
	outFile.close();
	
	ifstream inFile(fileName);
	vec.Load(inFile);
	vec.Save(cerr);
#endif	
	typedef Partition3D<IndexType> PartitionType;
	PartitionType partition(0.02);
	
	LaserPoints pts;
	for(double d=0;d<=2;d+=0.001)
		pts.push_back(LaserPoint(d,d+rand()/(double)RAND_MAX*1e-2,d));
	
	partition.Update(pts);
	partition.Print();
	
	
	int64 i;
	std::string filename = "stxxl2";
	std::string filename2 = "stxxl";
	typedef std::vector<PointType> PointVector;
	typedef stxxl::VECTOR_GENERATOR<PointType>::result PointVectorXL;
	const int64 vecSize = ((1024*1024)); 
	
	//Buffer misalignment error we are getting is may be due to vector::element size.
	//cerr<<"Size of PointType is : "<<sizeof(PointType)<<endl;cin.get();
	
	//For loading stxxl from an existing file, the file object must be created first.
	stxxl::syscall_file::syscall_file  file(filename, stxxl::file::RDWR);
	stxxl::syscall_file::syscall_file  file2(filename2, stxxl::file::RDWR);
	
	//Create PointVectorXL objects and size them properly.
	PointVectorXL *pV = new PointVectorXL(&file);
	pV->resize(vecSize);
	PointVectorXL *pU = new PointVectorXL(&file2);
	pU->resize(vecSize);
	
	PointVectorXL &v = *pV;
	PointVectorXL &u = *pU;
	double b,e;

	#define R() (1.00*rand()/(double)RAND_MAX)
	#define R_PT PointType(R(),R(),R())
	
	//Make the v vector.
	STXXL_MSG("write v with "<<(v.end() - v.begin())/1024<< "K elements ...")
	
	for(int i=0;i<v.size();i++)
		v[i] = R_PT;
	
		
	STXXL_MSG("write u with "<<(u.end() - u.begin())/1024<< "K elements ...")
	for(int i=0;i<u.size();i++)
		u[i] = R_PT;
	
	//Update the partition for v.
	partition.Update(v);
	partition.Print(cout);

	//Write arranged elements from the partiton to u, also update the partition for u.	
	cout<<"WriteArrangedStxxl...";
	partition.WriteArrangedStxxl(v, u);
	cout<<"\n";
	partition.Print(cout);
	partition.Save("partition.par");
	partition.Load("partition.par");
	cout<<"\n Partition after loading \n";
	partition.Print(cout);
	
	for(int i=-10;i<10;i++)
	{
		PartitionType::PartitionBoxVector vec = partition.Get27Neighbors(PartitionBox<int>(i,i,i));
		cerr<<"Neighbour count is: "<<vec.size()<<endl;
		cerr<<"Count: "<<partition.Count(vec)<<endl;
		cerr<<"2nd level: "<<partition.Get27Neighbors(vec).size()<<endl;
		
		PointVector pVec;
		partition.AddSelection(u,pVec,vec);
	}
	
	partition.Normals(u, v, 20);
	cerr<<"\n\nExiting to the system...\n";			
exit(0);	
	
	//A Check if the data is really serialized and can be read back.
#if 0
	for(int i=0; i<v.size(); i+= v.size()/10)
		std::cerr<<"before v["<<i<<"] = "<<v[i]<<"\n";
		
	delete pV; delete pU;
	
	pV = new PointVectorXL(&file);
		
	PointVectorXL &vf = *pV;
	const PointVectorXL& vc = *pV;
		
	for(int i=0; i<vf.size(); i+= vf.size()/10)
		std::cerr<<"after vf["<<i<<"] = "<<vf[i]<<"\n";
		
	//Check the difference between linear and random access.
	double sum = 0;
	
	//First with read write access.
	b = stxxl_timestamp();
	for(int i=0;i<vf.size();i++)
		sum += vf[i];
	e = stxxl_timestamp();
	STXXL_MSG("linear RW: "<<(e-b))
	
	//With read only linear access.
	b = stxxl_timestamp();
	for(int i=0;i<vc.size();i++)
		sum += vc[i];
	e = stxxl_timestamp();
	STXXL_MSG("linear R: "<<(e-b))
	
	//With read write random access.
	b = stxxl_timestamp();
	for(int i=0;i<vf.size();i++)
		sum += vf[rand()%vf.size()];
	e = stxxl_timestamp();
	STXXL_MSG("random RW: "<<(e-b))
	
	//With read only linear access.
	b = stxxl_timestamp();
	for(int i=0;i<vc.size();i++)
		sum += vc[rand()%vc.size()];
	e = stxxl_timestamp();
	STXXL_MSG("random R: "<<(e-b))
	
		
	exit(0);
	




	
	
	STXXL_MSG("for_each_m ...")
	b = stxxl_timestamp();
	stxxl::for_each_m(v.begin(),v.end(),square<int64>(),4);
	e = stxxl_timestamp();
	STXXL_MSG("for_each_m time: "<<(e-b))
	
	
	STXXL_MSG("check")
	//It is only read only so a const vector would be more efficient.
	const PointVectorXL& vr = v;
	for(i=0;i<vr.size();++i)
	{
		if(vr[i] != i*i ) STXXL_MSG("Error at position "<<i)
	}

	STXXL_MSG("Pos of value    1023: "<< (stxxl::find(v.begin(),v.end(),1023,4) - v.begin()))
	STXXL_MSG("Pos of value 1048576: "<< (stxxl::find(v.begin(),v.end(),1024*1024,4) - v.begin()))
	STXXL_MSG("Pos of value    1024: "<< (stxxl::find(v.begin(),v.end(),32*32,4) - v.begin())) 
	
	STXXL_MSG("generate ...")
	b = stxxl_timestamp();
	stxxl::generate(v.begin() + 1,v.end() -1,fill_value<int64>(555),4);
	e = stxxl_timestamp();
	STXXL_MSG("generate: "<<(e-b))
	
	
	STXXL_MSG("check")
	if(v[0] != 0) STXXL_MSG("Error at position "<<i)
	if(v[v.size()-1] != (v.size()-1)*(v.size()-1) ) STXXL_MSG("Error at position "<<i)
		
	for(i=1;i<v.size()-1;++i)
	{
		if(v[i] != 555 ) STXXL_MSG("Error at position "<<i)
	}
	
#endif	
	return 0;
};
